/************************************************************
MPU6500_Basic
 Basic example sketch for MPU-6500 DMP Arduino Library

 Renzo Mischianti @ mischianti.org
 https://github.com/xreef/MPU-6500-DMP_Library

This example sketch demonstrates how to initialize the
MPU-6500, and stream its sensor outputs to a serial monitor.

*************************************************************/
#include <MPU6500-DMP.h>
#include "imu_calibration.h"

#if defined(SAMD)
#define SerialPort SerialUSB
#else
#define SerialPort Serial
#endif

MPU6500_DMP imu;
bool calibrationMode = false;

void processSerialCommand(void);

void printIMUData(void);

void setup()
{
  SerialPort.begin(115200);
  delay(2000); // Wait a bit for the serial monitor to open
  SerialPort.println("Starting...");

  // Call imu.begin() to verify communication with and
  // initialize the MPU-6500 to it's default values.
  // Most functions return an error code - INV_SUCCESS (0)
  // indicates the IMU was present and successfully set up
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-6500");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }

  SerialPort.println("MPU-6500 initialized successfully!");

  // Use setSensors to turn on or off MPU-6500 sensors.
  // Any of the following defines can be combined:
  // INV_XYZ_GYRO, INV_XYZ_ACCEL,
  // INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
  // Enable all sensors:
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);

  // Use setGyroFSR() and setAccelFSR() to configure the
  // gyroscope and accelerometer full scale ranges.
  // Gyro options are +/- 250, 500, 1000, or 2000 dps
  imu.setGyroFSR(500); // Set gyro to 2000 dps
  // Accel options are +/- 2, 4, 8, or 16 g
  imu.setAccelFSR(4); // Set accel to +/-2g

  // 500 and 4 are typical rov valyes
  // can be changed later needs trial testign

  // setLPF() can be used to set the digital low-pass filter
  // of the accelerometer and gyroscope.
  // Can be any of the following: 188, 98, 42, 20, 10, 5
  // (values are in Hz).
  imu.setLPF(5); // Set LPF corner frequency to 5Hz

  // The sample rate of the accel/gyro can be set using
  // setSampleRate. Acceptable values range from 4Hz to 1kHz
  imu.setSampleRate(100); // Set sample rate to 100Hz
}

void loop()
{
  processSerialCommand();

  // dataReady() checks to see if new accel/gyro data
  // is available. It will return a boolean true or false.
  // NOTE: This is a software polling approach. For a hardware interrupt-driven
  // approach, see the MPU6500_Basic_Interrupt example.
  if ( imu.dataReady() )
  {
    // Call update() to update the imu objects sensor data.
    // You can specify which sensors to update by combining
    // UPDATE_ACCEL, UPDATE_GYRO, and/or UPDATE_TEMP.
    // (The update function defaults to accel and gyro,
    //  so you don't have to specify these values.)
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_TEMP);
    printIMUData();
  }
  delay(10); // Prevents I2C spam on fast microcontrollers like ESP32
}

void printIMUData(void)
{
  // After calling update() the ax, ay, az, gx, gy, gz,
  // time, and/or temerature class variables are all
  // updated. Access them by placing the object. in front:

  // Use the calcAccel and calcGyro functions to
  // convert the raw sensor readings (signed 16-bit values)
  // to their respective units.
  float rawAccelX = imu.calcAccel(imu.ax);
  float rawAccelY = imu.calcAccel(imu.ay);
  float rawAccelZ = imu.calcAccel(imu.az);
  float rawGyroX = imu.calcGyro(imu.gx);
  float rawGyroY = imu.calcGyro(imu.gy);
  float rawGyroZ = imu.calcGyro(imu.gz);

  // Calculate temperature in degrees Celsius
  float temp = imu.calcTempCelsius();

  if (calibrationMode)
  {
    SerialPort.print("CAL,");
    SerialPort.print(rawAccelX, 5); SerialPort.print(',');
    SerialPort.print(rawAccelY, 5); SerialPort.print(',');
    SerialPort.print(rawAccelZ, 5); SerialPort.print(',');
    SerialPort.print(rawGyroX, 5); SerialPort.print(',');
    SerialPort.print(rawGyroY, 5); SerialPort.print(',');
    SerialPort.print(rawGyroZ, 5); SerialPort.print(',');
    SerialPort.println(temp, 3);
    return;
  }

  float accelX = (rawAccelX - IMU_ACCEL_BIAS_X_G) * IMU_ACCEL_SCALE_X;
  float accelY = (rawAccelY - IMU_ACCEL_BIAS_Y_G) * IMU_ACCEL_SCALE_Y;
  float accelZ = (rawAccelZ - IMU_ACCEL_BIAS_Z_G) * IMU_ACCEL_SCALE_Z;
  float gyroX = rawGyroX - IMU_GYRO_BIAS_X_DPS -
                IMU_GYRO_TEMP_SLOPE_X_DPS_PER_C * (temp - IMU_CALIBRATION_TEMP_C);
  float gyroY = rawGyroY - IMU_GYRO_BIAS_Y_DPS -
                IMU_GYRO_TEMP_SLOPE_Y_DPS_PER_C * (temp - IMU_CALIBRATION_TEMP_C);
  float gyroZ = rawGyroZ - IMU_GYRO_BIAS_Z_DPS -
                IMU_GYRO_TEMP_SLOPE_Z_DPS_PER_C * (temp - IMU_CALIBRATION_TEMP_C);

  SerialPort.print("D,");
  SerialPort.print(accelX, 4); SerialPort.print(',');
  SerialPort.print(accelY, 4); SerialPort.print(',');
  SerialPort.print(accelZ, 4); SerialPort.print(',');
  SerialPort.print(gyroX, 3); SerialPort.print(',');
  SerialPort.print(gyroY, 3); SerialPort.print(',');
  SerialPort.print(gyroZ, 3); SerialPort.print(',');
  SerialPort.println(temp, 1);
}

void processSerialCommand(void)
{
  while (SerialPort.available() > 0)
  {
    char command = SerialPort.read();
    if (command == 'C') calibrationMode = true;
    if (command == 'N') calibrationMode = false;
  }
}
