"""Live 3D MPU-6500 orientation visualizer for the Arduino text stream."""

import argparse
import math
import re
import time

import matplotlib.pyplot as plt
import numpy as np
import serial
from ahrs.filters import EKF
from ahrs.common.orientation import q2euler
from matplotlib.animation import FuncAnimation


ACCEL_PATTERN = re.compile(
	r"Accel:\s*([-+\d.eE]+),\s*([-+\d.eE]+),\s*([-+\d.eE]+)\s*g"
)
GYRO_PATTERN = re.compile(
	r"Gyro:\s*([-+\d.eE]+),\s*([-+\d.eE]+),\s*([-+\d.eE]+)\s*dps"
)


def parse_vector(pattern: re.Pattern[str], line: str) -> tuple[float, float, float] | None:
	match = pattern.search(line)
	if match is None:
		return None
	return tuple(float(value) for value in match.groups())


def parse_data_record(line: str) -> tuple[tuple[float, float, float], tuple[float, float, float]] | None:
	fields = line.split(",")
	if len(fields) != 8 or fields[0] != "D":
		return None
	try:
		values = tuple(float(value) for value in fields[1:])
	except ValueError:
		return None
	return values[:3], values[3:6]


def rotate_points(points: np.ndarray, roll: float, pitch: float, yaw: float) -> np.ndarray:
	rx, ry, rz = np.radians([roll, pitch, yaw])
	rotation_x = np.array(
		[[1, 0, 0], [0, math.cos(rx), -math.sin(rx)], [0, math.sin(rx), math.cos(rx)]]
	)
	rotation_y = np.array(
		[[math.cos(ry), 0, math.sin(ry)], [0, 1, 0], [-math.sin(ry), 0, math.cos(ry)]]
	)
	rotation_z = np.array(
		[[math.cos(rz), -math.sin(rz), 0], [math.sin(rz), math.cos(rz), 0], [0, 0, 1]]
	)
	return points @ (rotation_z @ rotation_y @ rotation_x).T


def initial_quaternion(accel: np.ndarray) -> np.ndarray:
	ax, ay, az = accel
	roll = math.atan2(ay, az)
	pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
	half_roll = roll / 2
	half_pitch = pitch / 2
	return np.array([
		math.cos(half_roll) * math.cos(half_pitch),
		math.sin(half_roll) * math.cos(half_pitch),
		math.cos(half_roll) * math.sin(half_pitch),
		-math.sin(half_roll) * math.sin(half_pitch),
	])


def main() -> int:
	parser = argparse.ArgumentParser(
		description="Visualize live MPU-6500 orientation from an Arduino serial stream."
	)
	parser.add_argument("port", nargs="?", default="COM16", help="Arduino port (default: COM16)")
	parser.add_argument("--baud", type=int, default=115200)
	parser.add_argument("--calibration-seconds", type=float, default=2.0)
	args = parser.parse_args()
	if args.calibration_seconds < 0:
		parser.error("calibration seconds must be non-negative")

	# The modeled board's +X, +Y, and +Z axes are red, green, and blue.
	half_x, half_y, half_z = 1.15, 0.72, 0.12
	vertices = np.array(
		[
			[-half_x, -half_y, -half_z], [half_x, -half_y, -half_z],
			[half_x, half_y, -half_z], [-half_x, half_y, -half_z],
			[-half_x, -half_y, half_z], [half_x, -half_y, half_z],
			[half_x, half_y, half_z], [-half_x, half_y, half_z],
		]
	)
	axis_endpoints = np.array([[0.0, 0.0, 0.0], [1.7, 0.0, 0.0], [0.0, 1.3, 0.0], [0.0, 0.0, 1.3]])
	axis_colors = ["#e45756", "#54a24b", "#4c78a8"]
	axis_labels = ["+X", "+Y", "+Z"]

	figure = plt.figure("6-DOF IMU Orientation", figsize=(11, 7), facecolor="#f4f5f2")
	axis = figure.add_subplot(111, projection="3d", facecolor="#f4f5f2")
	axis.set_title("MPU-6500  /  LIVE ORIENTATION", loc="left", pad=18, fontsize=15, fontweight="bold")
	axis.set_xlim(-2, 2)
	axis.set_ylim(-2, 2)
	axis.set_zlim(-2, 2)
	axis.set_box_aspect((1, 1, 1))
	axis.set_xlabel("X")
	axis.set_ylabel("Y")
	axis.set_zlabel("Z")
	axis.view_init(elev=24, azim=36)
	axis.grid(True, alpha=0.2)
	for pane in (axis.xaxis.pane, axis.yaxis.pane, axis.zaxis.pane):
		pane.set_facecolor((0.96, 0.97, 0.95, 1.0))

	board = axis.plot([], [], [], color="#252a2c", linewidth=2)[0]
	axis_lines = [axis.plot([], [], [], color=color, linewidth=3)[0] for color in axis_colors]
	axis_text = [axis.text(0, 0, 0, label, color=color, fontsize=10, fontweight="bold")
				  for label, color in zip(axis_labels, axis_colors)]
	readout = figure.text(0.04, 0.06, "Connecting to Arduino...", fontsize=11, family="monospace")
	figure.text(0.96, 0.04, "6-DOF  /  YAW UNREFERENCED", ha="right", fontsize=8, color="#687174")

	quaternion = np.array([1.0, 0.0, 0.0, 0.0])
	filter_rate = 100
	imu_filter = EKF(frequency=filter_rate)
	gyro_bias: tuple[float, float, float] | None = None
	gyro_samples: list[tuple[float, float, float]] = []
	accel: tuple[float, float, float] | None = None
	calibration_started: float | None = None
	previous_time: float | None = None
	last_update = time.monotonic()
	serial_error: str | None = None

	try:
		connection = serial.Serial(args.port, args.baud, timeout=0.001)
		connection.reset_input_buffer()
	except serial.SerialException as error:
		connection = None
		serial_error = str(error)

	def read_sample() -> None:
		nonlocal accel, gyro_bias, calibration_started, previous_time, last_update, serial_error, quaternion
		if connection is None or serial_error:
			return
		try:
			for _ in range(100):
				line = connection.readline().decode("ascii", errors="ignore").strip()
				if not line:
					break
				data_record = parse_data_record(line)
				if data_record is not None:
					accel, gyro = data_record
				else:
					gyro = None
				new_accel = parse_vector(ACCEL_PATTERN, line)
				if data_record is None and new_accel is not None:
					accel = new_accel
					continue
				if data_record is None:
					gyro = parse_vector(GYRO_PATTERN, line)
				if gyro is None or accel is None:
					continue

				now = time.monotonic()
				last_update = now
				if calibration_started is None:
					calibration_started = now
				if gyro_bias is None:
					gyro_samples.append(gyro)
					if now - calibration_started >= args.calibration_seconds:
						gyro_bias = tuple(
							sum(sample[i] for sample in gyro_samples) / len(gyro_samples)
							for i in range(3)
						)
						quaternion = initial_quaternion(np.asarray(accel))
						previous_time = now
					continue

				if previous_time is None:
					previous_time = now
					continue
				dt = min(now - previous_time, 0.1)
				previous_time = now
				if dt <= 0:
					continue

				gyro_radians = np.radians([gyro[i] - gyro_bias[i] for i in range(3)])
				quaternion = imu_filter.update(
					q=quaternion,
					gyr=gyro_radians,
					acc=np.asarray(accel),
					dt=dt,
				)
		except serial.SerialException as error:
			serial_error = str(error)

	def animate(_frame: int):
		read_sample()
		orientation = np.degrees(q2euler(quaternion))
		rotated_vertices = rotate_points(vertices, *orientation)
		# Draw the rectangular board as its twelve edges.
		edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]
		board.set_data_3d(
			[coordinate for edge in edges for coordinate in (rotated_vertices[edge[0], 0], rotated_vertices[edge[1], 0])],
			[coordinate for edge in edges for coordinate in (rotated_vertices[edge[0], 1], rotated_vertices[edge[1], 1])],
			[coordinate for edge in edges for coordinate in (rotated_vertices[edge[0], 2], rotated_vertices[edge[1], 2])],
		)
		rotated_axes = rotate_points(axis_endpoints, *orientation)
		for index, line in enumerate(axis_lines):
			line.set_data_3d(rotated_axes[[0, index + 1], 0], rotated_axes[[0, index + 1], 1], rotated_axes[[0, index + 1], 2])
			axis_text[index].set_position((rotated_axes[index + 1, 0], rotated_axes[index + 1, 1]))
			axis_text[index].set_3d_properties(rotated_axes[index + 1, 2])

		if serial_error:
			status = f"SERIAL ERROR: {serial_error}"
		elif gyro_bias is None:
			elapsed = 0 if calibration_started is None else time.monotonic() - calibration_started
			status = f"CALIBRATING GYRO  {min(elapsed, args.calibration_seconds):.1f}/{args.calibration_seconds:.1f} s"
		elif time.monotonic() - last_update > 2:
			status = "Waiting for IMU stream..."
		else:
			status = "LIVE"
		readout.set_text(
			f"{status}\nROLL  {orientation[0]:8.2f} deg    "
			f"PITCH {orientation[1]:8.2f} deg    YAW {orientation[2]:8.2f} deg"
		)
		return [board, *axis_lines, *axis_text, readout]

	animation = FuncAnimation(figure, animate, interval=40, blit=False, cache_frame_data=False)
	try:
		plt.show()
	finally:
		if connection is not None and connection.is_open:
			connection.close()
	return 0


if __name__ == "__main__":
	try:
		raise SystemExit(main())
	except KeyboardInterrupt:
		raise SystemExit(0)