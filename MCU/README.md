# MCU Firmware and Tools

This folder contains the two Arduino sketches, two live visualizers, and the six-position MPU-6500 calibration utility.

## Python Setup

From this folder in PowerShell:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

## IMU Calibration

Close Serial Monitor, upload the calibration-capable sketch, then run:

```powershell
python .\calibrate_imu.py COM16
```

Follow the six-face prompts. The utility updates `imu_calibration.json` and `imu_calibration.h`. Upload the sketch again after calibration so the Nano applies the generated values.

## Standalone IMU

Compile and upload the standalone IMU sketch:

```powershell
.\compile_upload.ps1
```

Run either visualizer after upload:

```powershell
python .\visualize_imu.py COM16
python .\visualize_pitch_roll.py COM16
```

## Thruster Controller

Compile and upload the combined IMU/thruster controller:

```powershell
.\compile_upload_thrusters.ps1
```

Close Serial Monitor before upload. The controller starts disarmed, requires explicit arming, and returns ESC signals to neutral on host timeout. Test with thrusters safely secured and props clear. The Nano output pins are logic/signal outputs; external power drivers must supply actuator and thruster current.

Use `-CompileOnly` to validate either sketch without uploading. Use `-Processor atmega328old` if the Nano uses the old bootloader.

## ROS 2 Serial Bridge

The matching ROS 2 node is in `src/rov_serial/rov_serial/rov_serial_bridge.py` and is also copied here as `rov_serial_bridge.py`. It sends a 22-byte sync/sequence/arm/PWM/actuator/CRC packet every 50 ms and disarms if `/rov/motion` becomes stale. The Nano also has a 500 ms serial packet watchdog.

Install ROS package dependencies with `rosdep`, build the workspace, then run:

```bash
ros2 run rov_serial rov_serial_bridge --ros-args -p serial_port:=/dev/ttyACM0 -p baud_rate:=115200
```

Topics:

- `/rov/motion`: `Int32MultiArray` with eight ESC pulse widths in microseconds, each 1100-1900.
- `/rov/actuators`: `Int32MultiArray` with up to eight nonzero/on or zero/off states.
- `/rov/arm`: `Bool`; publish false to establish disarmed state, then true to arm after fresh motion commands are arriving.
- `/rov/armed`: `Bool` reporting the armed state acknowledged by the Nano.
- `/rov/imu`: `Float32MultiArray` ordered as accel X/Y/Z (g), gyro X/Y/Z (dps), temperature (C).

The current firmware has no pressure sensor, so it does not publish depth. The serial bridge uses `/dev/ttyACM0` by default; pass the actual device path as `serial_port` on the target computer.
