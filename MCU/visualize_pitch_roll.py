"""Live pitch-and-roll artificial horizon for the Arduino MPU-6500 stream."""

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
from matplotlib.patches import Circle, Polygon
from matplotlib.lines import Line2D


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


def initial_quaternion(accel: tuple[float, float, float]) -> np.ndarray:
    ax, ay, az = accel
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
    half_roll = roll / 2
    half_pitch = pitch / 2
    return np.array(
        [
            math.cos(half_roll) * math.cos(half_pitch),
            math.sin(half_roll) * math.cos(half_pitch),
            math.cos(half_roll) * math.sin(half_pitch),
            -math.sin(half_roll) * math.sin(half_pitch),
        ]
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Live pitch and roll view for an MPU-6500.")
    parser.add_argument("port", nargs="?", default="COM16", help="Arduino port (default: COM16)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--calibration-seconds", type=float, default=2.0)
    parser.add_argument("--display-smoothing", type=float, default=0.22)
    parser.add_argument("--refresh-rate", type=int, default=60, help="visualizer redraw rate in frames per second")
    args = parser.parse_args()
    if args.calibration_seconds < 0:
        parser.error("calibration seconds must be non-negative")
    if not 0.0 < args.display_smoothing <= 1.0:
        parser.error("display smoothing must be in (0, 1]")
    if not 1 <= args.refresh_rate <= 120:
        parser.error("refresh rate must be between 1 and 120 fps")

    figure = plt.figure("IMU Pitch and Roll", figsize=(10, 6), facecolor="#111b1d")
    panel = figure.add_axes([0.04, 0.07, 0.92, 0.86], facecolor="#111b1d")
    panel.set_xlim(0, 10)
    panel.set_ylim(0, 6)
    panel.set_aspect("equal")
    panel.axis("off")
    panel.text(0.25, 5.58, "ATTITUDE", color="#f1f4ee", fontsize=17, weight="bold")
    panel.text(0.27, 5.25, "MPU-6500  /  LIVE", color="#90a6a5", fontsize=9)

    center = np.array([3.25, 3.05])
    radius = 2.18
    clip_circle = Circle(center, radius, transform=panel.transData)
    sky = Circle(center, radius, facecolor="#51a9c0", edgecolor="none", zorder=1)
    panel.add_patch(sky)
    ground = Polygon(np.zeros((4, 2)), closed=True, facecolor="#bd7448", edgecolor="none", zorder=2)
    ground.set_clip_path(clip_circle)
    panel.add_patch(ground)
    panel.add_patch(Circle(center, radius, fill=False, edgecolor="#dce3dc", linewidth=2, zorder=6))

    roll_angle = 0.0
    for degrees in range(-60, 61, 10):
        angle = math.radians(degrees - 90)
        inner = radius - (0.24 if degrees % 30 == 0 else 0.12)
        outer_point = center + np.array([math.cos(angle), math.sin(angle)]) * (radius - 0.04)
        inner_point = center + np.array([math.cos(angle), math.sin(angle)]) * inner
        panel.add_line(Line2D(
            [inner_point[0], outer_point[0]], [inner_point[1], outer_point[1]],
            color="#f1f4ee", linewidth=1.4 if degrees % 30 == 0 else 0.8, zorder=7,
        ))
    panel.add_patch(Polygon(
        [center + [0, radius + 0.02], center + [-0.12, radius - 0.25], center + [0.12, radius - 0.25]],
        closed=True, facecolor="#e4bd61", edgecolor="none", zorder=8,
    ))

    ladder_lines: list[Line2D] = []
    ladder_labels = []
    for degrees in (-30, -20, -10, 10, 20, 30):
        line = Line2D([], [], color="#f3f3e9", linewidth=1.25, zorder=5)
        panel.add_line(line)
        ladder_lines.append(line)
        label = panel.text(0, 0, str(abs(degrees)), color="#f3f3e9", fontsize=8,
                           ha="center", va="center", zorder=5, clip_on=True)
        label.set_clip_path(clip_circle)
        ladder_labels.append(label)

    panel.add_patch(Circle(center, 0.055, facecolor="#e4bd61", edgecolor="none", zorder=9))
    panel.text(6.25, 4.78, "ROLL CHANGE / FROM START", color="#90a6a5", fontsize=9, weight="bold")
    roll_readout = panel.text(6.22, 4.0, "+00.0 deg", color="#f1f4ee", fontsize=25, family="monospace")
    panel.text(6.25, 3.60, "PITCH CHANGE / FROM START", color="#90a6a5", fontsize=9, weight="bold")
    pitch_readout = panel.text(6.22, 2.88, "+00.0 deg", color="#f1f4ee", fontsize=25, family="monospace")
    panel.text(6.25, 2.30, "YAW CHANGE / FROM START", color="#90a6a5", fontsize=9, weight="bold")
    yaw_readout = panel.text(6.22, 1.58, "+00.0 deg", color="#f1f4ee", fontsize=25, family="monospace")
    status_readout = panel.text(6.25, 0.72, "CONNECTING", color="#e4bd61", fontsize=10, weight="bold")

    quaternion = np.array([1.0, 0.0, 0.0, 0.0])
    imu_filter = EKF(frequency=100)
    gyro_samples: list[tuple[float, float, float]] = []
    gyro_bias: tuple[float, float, float] | None = None
    accel: tuple[float, float, float] | None = None
    calibration_started: float | None = None
    previous_time: float | None = None
    last_update = time.monotonic()
    serial_error: str | None = None
    target_roll = 0.0
    target_pitch = 0.0
    target_yaw_delta = 0.0
    roll_reference = 0.0
    pitch_reference = 0.0
    previous_yaw = 0.0
    shown_roll = 0.0
    shown_pitch = 0.0
    shown_yaw_delta = 0.0

    try:
        connection = serial.Serial(args.port, args.baud, timeout=0.001)
        connection.reset_input_buffer()
    except serial.SerialException as error:
        connection = None
        serial_error = str(error)

    def read_samples() -> None:
        nonlocal accel, gyro_bias, calibration_started, previous_time
        nonlocal last_update, serial_error, quaternion, target_roll, target_pitch
        nonlocal target_yaw_delta, roll_reference, pitch_reference, previous_yaw
        if connection is None or serial_error:
            return
        try:
            for _ in range(60):
                line = connection.readline().decode("ascii", errors="ignore").strip()
                if not line:
                    break
                data_record = parse_data_record(line)
                if data_record is not None:
                    accel, gyro = data_record
                else:
                    gyro = None
                parsed_accel = parse_vector(ACCEL_PATTERN, line)
                if data_record is None and parsed_accel is not None:
                    accel = parsed_accel
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
                        quaternion = initial_quaternion(accel)
                        initial_angles = np.degrees(q2euler(quaternion))
                        target_roll, target_pitch = float(initial_angles[0]), float(initial_angles[1])
                        roll_reference = target_roll
                        pitch_reference = target_pitch
                        previous_yaw = float(initial_angles[2])
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
                angles = np.degrees(q2euler(quaternion))
                target_roll, target_pitch = float(angles[0]), float(angles[1])
                yaw_step = (float(angles[2]) - previous_yaw + 180.0) % 360.0 - 180.0
                target_yaw_delta += yaw_step
                previous_yaw = float(angles[2])
        except serial.SerialException as error:
            serial_error = str(error)

    def animate(_frame: int):
        nonlocal shown_roll, shown_pitch, shown_yaw_delta
        read_samples()
        target_roll_delta = (target_roll - roll_reference + 180.0) % 360.0 - 180.0
        roll_delta = (target_roll_delta - shown_roll + 180.0) % 360.0 - 180.0
        shown_roll += args.display_smoothing * roll_delta
        target_pitch_delta = target_pitch - pitch_reference
        shown_pitch += args.display_smoothing * (target_pitch_delta - shown_pitch)
        shown_yaw_delta += args.display_smoothing * (target_yaw_delta - shown_yaw_delta)

        angle = math.radians(-shown_roll)
        direction = np.array([math.cos(angle), math.sin(angle)])
        normal = np.array([-direction[1], direction[0]])
        pitch_offset = -shown_pitch * 0.022
        span = 8.0
        horizon_center = center + normal * pitch_offset
        start = horizon_center - direction * span
        end = horizon_center + direction * span
        ground.set_xy([start, end, end - normal * span * 2, start - normal * span * 2])

        for index, degrees in enumerate((-30, -20, -10, 10, 20, 30)):
            ladder_offset = pitch_offset - degrees * 0.022
            ladder_center = center + normal * ladder_offset
            half_width = 0.55 if abs(degrees) == 10 else 0.42
            first = ladder_center - direction * half_width
            second = ladder_center + direction * half_width
            ladder_lines[index].set_data([first[0], second[0]], [first[1], second[1]])
            ladder_labels[index].set_position((ladder_center[0], ladder_center[1]))

        roll_readout.set_text(f"{shown_roll:+06.1f} deg")
        pitch_readout.set_text(f"{shown_pitch:+06.1f} deg")
        yaw_readout.set_text(f"{shown_yaw_delta:+06.1f} deg")
        if serial_error:
            status_readout.set_text("SERIAL ERROR")
            status_readout.set_color("#e27a67")
        elif gyro_bias is None:
            elapsed = 0.0 if calibration_started is None else time.monotonic() - calibration_started
            status_readout.set_text(f"CALIBRATING  {min(elapsed, args.calibration_seconds):.1f}s")
        elif time.monotonic() - last_update > 2:
            status_readout.set_text("WAITING FOR DATA")
            status_readout.set_color("#e4bd61")
        else:
            status_readout.set_text("LIVE")
            status_readout.set_color("#83c69a")
        return [ground, *ladder_lines, *ladder_labels, roll_readout, pitch_readout, yaw_readout, status_readout]

    refresh_interval_ms = max(1, round(1000 / args.refresh_rate))
    animation = FuncAnimation(figure, animate, interval=refresh_interval_ms, blit=False, cache_frame_data=False)
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
