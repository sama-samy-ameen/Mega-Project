"""Guided six-position MPU-6500 calibration for the Arduino sketch."""

import argparse
import json
import statistics
import time
from pathlib import Path

import serial


POSES = ("+X", "-X", "+Y", "-Y", "+Z", "-Z")


def write_header(path: Path, calibration: dict) -> None:
    accel_bias = calibration["accelerometer"]["bias_g"]
    accel_scale = calibration["accelerometer"]["scale"]
    gyro_bias = calibration["gyroscope"]["bias_dps"]
    temp_ref = calibration["gyroscope"]["reference_temperature_c"]
    content = f"""#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H

#define IMU_ACCEL_BIAS_X_G {accel_bias[0]:.8f}f
#define IMU_ACCEL_BIAS_Y_G {accel_bias[1]:.8f}f
#define IMU_ACCEL_BIAS_Z_G {accel_bias[2]:.8f}f
#define IMU_ACCEL_SCALE_X {accel_scale[0]:.8f}f
#define IMU_ACCEL_SCALE_Y {accel_scale[1]:.8f}f
#define IMU_ACCEL_SCALE_Z {accel_scale[2]:.8f}f
#define IMU_GYRO_BIAS_X_DPS {gyro_bias[0]:.8f}f
#define IMU_GYRO_BIAS_Y_DPS {gyro_bias[1]:.8f}f
#define IMU_GYRO_BIAS_Z_DPS {gyro_bias[2]:.8f}f
#define IMU_GYRO_TEMP_SLOPE_X_DPS_PER_C 0.0f
#define IMU_GYRO_TEMP_SLOPE_Y_DPS_PER_C 0.0f
#define IMU_GYRO_TEMP_SLOPE_Z_DPS_PER_C 0.0f
#define IMU_CALIBRATION_TEMP_C {temp_ref:.4f}f

#endif
"""
    path.write_text(content, encoding="ascii")


def collect_pose(
    connection: serial.Serial, pose: str, axis_index: int, count: int
) -> list[tuple[float, ...]]:
    expected_sign = 1.0 if pose[0] == "+" else -1.0
    expected = [0.0, 0.0, 0.0]
    expected[axis_index] = expected_sign

    while True:
        print(f"\n{pose} pose: expected accel near ({expected[0]:+.0f}, {expected[1]:+.0f}, {expected[2]:+.0f}) g.")
        input(f"Point the marked {pose} axis up, keep the IMU still, then press Enter...")
        connection.reset_input_buffer()
        print(f"Collecting {count} samples for {pose}...")
        samples: list[tuple[float, ...]] = []
        deadline = time.monotonic() + max(15.0, count * 1.5)
        while len(samples) < count and time.monotonic() < deadline:
            line = connection.readline().decode("ascii", errors="ignore").strip()
            if not line.startswith("CAL,"):
                continue
            try:
                values = tuple(float(value) for value in line.split(",")[1:])
            except ValueError:
                continue
            if len(values) == 7:
                samples.append(values)
        if len(samples) < count:
            raise RuntimeError(f"Only received {len(samples)} calibration samples for {pose}.")

        accel_mean = tuple(statistics.fmean(sample[i] for sample in samples) for i in range(3))
        others = [abs(accel_mean[i]) for i in range(3) if i != axis_index]
        valid = expected_sign * accel_mean[axis_index] >= 0.5 and abs(accel_mean[axis_index]) >= max(others)
        print(f"Measured accel: X={accel_mean[0]:+.3f} g, Y={accel_mean[1]:+.3f} g, Z={accel_mean[2]:+.3f} g")
        if valid:
            return samples

        print(f"WARNING: {pose} pose does not match the expected direction. Reposition the IMU before continuing.")
        retry = input("Press Enter to retry this pose, or type q to cancel calibration: ").strip().lower()
        if retry == "q":
            raise RuntimeError(f"Calibration cancelled at {pose} after an invalid orientation.")


def main() -> int:
    parser = argparse.ArgumentParser(description="Calibrate the IMU in six stationary orientations.")
    parser.add_argument("port", nargs="?", default="COM16")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--samples", type=int, default=40, help="samples to average per position")
    parser.add_argument("--json", type=Path, default=Path("imu_calibration.json"))
    parser.add_argument("--header", type=Path, default=Path("imu_calibration.h"))
    args = parser.parse_args()
    if args.samples < 10:
        parser.error("use at least 10 samples per position")

    print("Close Arduino Serial Monitor. Calibration takes about 25 seconds.")
    print("Do not move the sensor while each position is being sampled.")
    try:
        with serial.Serial(args.port, args.baud, timeout=0.5) as connection:
            try:
                time.sleep(2.5)
                connection.reset_input_buffer()
                connection.write(b"C")
                print("Waiting for calibration stream from Arduino...")
                deadline = time.monotonic() + 10
                while time.monotonic() < deadline:
                    if connection.readline().startswith(b"CAL,"):
                        break
                else:
                    raise RuntimeError("No CAL data received. Upload the updated imu.c sketch first.")

                pose_means: dict[str, tuple[float, ...]] = {}
                all_samples: list[tuple[float, ...]] = []
                for pose_index, pose in enumerate(POSES):
                    samples = collect_pose(connection, pose, pose_index // 2, args.samples)
                    pose_means[pose] = tuple(
                        statistics.fmean(sample[i] for sample in samples) for i in range(7)
                    )
                    all_samples.extend(samples)
                    print(f"{pose} captured.")
            finally:
                connection.write(b"N")
                connection.flush()
    except (serial.SerialException, RuntimeError) as error:
        print(f"Calibration failed: {error}")
        return 1

    for axis_index, axis in enumerate("XYZ"):
        positive = pose_means[f"+{axis}"][axis_index]
        negative = pose_means[f"-{axis}"][axis_index]
        if positive < 0.5 or negative > -0.5:
            print(f"Invalid {axis} poses: expected +{axis} and -{axis} to point up in turn.")
            return 1

    bias = []
    scale = []
    for axis_index, axis in enumerate("XYZ"):
        positive = pose_means[f"+{axis}"][axis_index]
        negative = pose_means[f"-{axis}"][axis_index]
        span = positive - negative
        if span < 1.0:
            print(f"Insufficient accel range on {axis}: {span:.3f} g.")
            return 1
        bias.append((positive + negative) / 2.0)
        scale.append(2.0 / span)

    calibration = {
        "accelerometer": {"bias_g": bias, "scale": scale},
        "gyroscope": {
            "bias_dps": [statistics.fmean(sample[3 + axis] for sample in all_samples) for axis in range(3)],
            "reference_temperature_c": statistics.fmean(sample[6] for sample in all_samples),
            "temperature_slope_dps_per_c": [0.0, 0.0, 0.0],
        },
        "pose_means": {pose: list(values) for pose, values in pose_means.items()},
    }
    args.json.write_text(json.dumps(calibration, indent=2) + "\n", encoding="utf-8")
    write_header(args.header, calibration)
    print(f"Saved calibration profile: {args.json}")
    print(f"Generated Arduino calibration header: {args.header}")
    print("Now upload imu.c again so the Nano applies the new calibration.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
