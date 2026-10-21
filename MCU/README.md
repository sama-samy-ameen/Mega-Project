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
