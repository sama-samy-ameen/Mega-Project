#include <MPU6500-DMP.h>
#include <Servo.h>
#include <avr/wdt.h>   // AVR boards only (Nano/Uno/Mega)
#include <stdlib.h>
#include <string.h>
#include "imu_calibration.h"

#if defined(SAMD)
#define SerialPort SerialUSB
#else
#define SerialPort Serial
#endif

const uint8_t ESC_COUNT = 8;
const uint8_t ACTUATOR_COUNT = 8;
const uint8_t escPins[ESC_COUNT] = {2, 3, 4, 5, 6, 7, 8, 9};
const uint8_t actuatorPins[ACTUATOR_COUNT] = {10, 11, 12, 13, A0, A1, A2, A3};
const int ESC_MIN_US = 1100;
const int ESC_NEUTRAL_US = 1500;
const int ESC_MAX_US = 1900;
const unsigned long ESC_NEUTRAL_STARTUP_MS = 3000;
const unsigned long HOST_TIMEOUT_MS = 500;
const uint8_t WATCHDOG_TIMEOUT = WDTO_500MS;  // reboots the board if loop() hangs
const uint8_t PACKET_LENGTH = 22;
const uint8_t PACKET_SYNC_0 = 0xA5;
const uint8_t PACKET_SYNC_1 = 0x5A;

MPU6500_DMP imu;
Servo esc[ESC_COUNT];
int escCommandUs[ESC_COUNT];
bool actuatorState[ACTUATOR_COUNT];
bool armed = false;
bool needsDisarmFirst = true;   // after boot or any disarm, host must send a disarm before arming
bool imuReady = false;
bool calibrationMode = false;
unsigned long lastHostCommand = 0;
char commandBuffer[80];
uint8_t commandLength = 0;
uint8_t packetBuffer[PACKET_LENGTH];
uint8_t packetLength = 0;
bool receivingPacket = false;
bool haveSequence = false;
uint8_t lastSequence = 0;

void processSerialCommands();
void processCommand(char *line);
void processBinaryPacket(const uint8_t *packet);
uint8_t calculateCrc8(const uint8_t *data, uint8_t length);
void disarmAll(const char *reason);
void printStatus();
void readAndPrintImu();
int thrustToPulse(float thrust);

void setup()
{
  wdt_disable();  // keep it off during the 3 s ESC startup; enabled at the end

  // Put every output in its safe state before doing anything slow.
  for (uint8_t i = 0; i < ACTUATOR_COUNT; ++i)
  {
    pinMode(actuatorPins[i], OUTPUT);
    digitalWrite(actuatorPins[i], LOW);
    actuatorState[i] = false;
  }

  for (uint8_t i = 0; i < ESC_COUNT; ++i)
  {
    escCommandUs[i] = ESC_NEUTRAL_US;
    esc[i].attach(escPins[i], ESC_MIN_US, ESC_MAX_US);
    esc[i].writeMicroseconds(ESC_NEUTRAL_US);
  }

  SerialPort.begin(115200);
  SerialPort.println(F("BOOT,THRUSTER_IMU_CONTROLLER"));
  SerialPort.println(F("ESC_INIT,NEUTRAL,3000ms"));
  for (uint8_t i = 0; i < ESC_COUNT; ++i)
  {
    SerialPort.print(F("ESC_SIGNAL_READY,"));
    SerialPort.print(i + 1);
    SerialPort.print(',');
    SerialPort.println(escPins[i]);
  }

  delay(ESC_NEUTRAL_STARTUP_MS);

  if (imu.begin() == INV_SUCCESS)
  {
    imuReady = true;
    imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    imu.setGyroFSR(500);
    imu.setAccelFSR(4);
    imu.setLPF(5);
    imu.setSampleRate(100);
    SerialPort.println(F("IMU,READY"));
  }
  else
  {
    SerialPort.println(F("FAULT,IMU_NOT_RESPONDING,THRUSTERS_LOCKED"));
  }

  lastHostCommand = millis();
  SerialPort.println(F("STATE,DISARMED"));
  SerialPort.println(F("COMMANDS,ARM|DISARM|PING|STATUS|T channel -1..1|A channel 0|1|C|N"));

  wdt_enable(WATCHDOG_TIMEOUT);
}

void loop()
{
  wdt_reset();
  processSerialCommands();

  if (armed && millis() - lastHostCommand > HOST_TIMEOUT_MS)
  {
    disarmAll("HOST_TIMEOUT");
  }

  if (imuReady && imu.dataReady())
  {
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_TEMP);
    readAndPrintImu();   // the 100 Hz sample rate already paces the output
  }
}

void processSerialCommands()
{
  while (SerialPort.available() > 0)
  {
    uint8_t incoming = (uint8_t)SerialPort.read();

    if (receivingPacket)
    {
      if (packetLength == 1 && incoming != PACKET_SYNC_1)
      {
        receivingPacket = incoming == PACKET_SYNC_0;
        packetLength = receivingPacket ? 1 : 0;
        continue;
      }
      packetBuffer[packetLength++] = incoming;
      if (packetLength == PACKET_LENGTH)
      {
        processBinaryPacket(packetBuffer);
        receivingPacket = false;
        packetLength = 0;
      }
      continue;
    }

    if (incoming == PACKET_SYNC_0)
    {
      receivingPacket = true;
      packetLength = 1;
      packetBuffer[0] = incoming;
      continue;
    }

    char textByte = (char)incoming;
    if (textByte == '\r') continue;
    if (textByte == '\n')
    {
      commandBuffer[commandLength] = '\0';
      if (commandLength > 0) processCommand(commandBuffer);
      commandLength = 0;
    }
    else if (commandLength < sizeof(commandBuffer) - 1)
    {
      commandBuffer[commandLength++] = textByte;
    }
    else
    {
      commandLength = 0;
      SerialPort.println(F("ERROR,COMMAND_TOO_LONG"));
    }
  }
}

void processBinaryPacket(const uint8_t *packet)
{
  if (packet[0] != PACKET_SYNC_0 || packet[1] != PACKET_SYNC_1 ||
      calculateCrc8(packet + 2, PACKET_LENGTH - 3) != packet[PACKET_LENGTH - 1])
  {
    SerialPort.println(F("ERROR,BAD_PACKET_CRC"));
    return;
  }

  uint8_t sequence = packet[2];
  if (haveSequence && sequence == lastSequence)
  {
    SerialPort.println(F("ERROR,DUPLICATE_PACKET"));
    return;
  }
  haveSequence = true;
  lastSequence = sequence;
  lastHostCommand = millis();

  bool armRequested = (packet[3] & 0x01) != 0;
  uint8_t actuatorMask = packet[20];

  // Step 1: check every pulse before touching any output.
  int pulses[ESC_COUNT];
  for (uint8_t i = 0; i < ESC_COUNT; ++i)
  {
    uint8_t offset = 4 + (i * 2);
    pulses[i] = (int)packet[offset] | ((int)packet[offset + 1] << 8);
    if (pulses[i] < ESC_MIN_US || pulses[i] > ESC_MAX_US)
    {
      disarmAll("INVALID_ESC_PULSE");
      SerialPort.println(F("ERROR,INVALID_ESC_PULSE"));
      return;
    }
  }

  // Step 2: decide whether arming is allowed. On every refusal the outputs
  // are already neutral, so we just report and return.
  if (armRequested)
  {
    if (needsDisarmFirst)
    {
      SerialPort.println(F("ERROR,DISARM_FIRST"));
      return;
    }
    if (!imuReady)
    {
      disarmAll("IMU_NOT_READY");
      SerialPort.println(F("ERROR,IMU_NOT_READY"));
      return;
    }
    if (calibrationMode)
    {
      SerialPort.println(F("ERROR,CALIBRATION_MODE"));
      return;
    }
  }
  else
  {
    needsDisarmFirst = false;   // the host has really sent a disarm
  }
  armed = armRequested;

  // Step 3: apply.
  for (uint8_t i = 0; i < ESC_COUNT; ++i)
  {
    escCommandUs[i] = armed ? pulses[i] : ESC_NEUTRAL_US;
    esc[i].writeMicroseconds(escCommandUs[i]);
  }
  for (uint8_t i = 0; i < ACTUATOR_COUNT; ++i)
  {
    actuatorState[i] = armed && ((actuatorMask & (1U << i)) != 0);
    digitalWrite(actuatorPins[i], actuatorState[i] ? HIGH : LOW);
  }

  // ACK,<sequence>,<armed> so the host can see the real armed state.
  SerialPort.print(F("ACK,"));
  SerialPort.print(sequence);
  SerialPort.print(',');
  SerialPort.println(armed ? 1 : 0);
}

uint8_t calculateCrc8(const uint8_t *data, uint8_t length)
{
  uint8_t crc = 0;
  for (uint8_t i = 0; i < length; ++i)
  {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit)
    {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

void processCommand(char *line)
{
  char *command = strtok(line, " ,\t");
  if (command == NULL) return;

  if (strcmp(command, "PING") == 0)
  {
    lastHostCommand = millis();
    SerialPort.println(F("PONG"));
    return;
  }
  if (strcmp(command, "STATUS") == 0)
  {
    printStatus();
    return;
  }
  if (strcmp(command, "DISARM") == 0)
  {
    disarmAll("COMMAND");
    return;
  }
  if (strcmp(command, "ARM") == 0)
  {
    if (!imuReady)
    {
      SerialPort.println(F("ERROR,IMU_NOT_READY"));
      return;
    }
    for (uint8_t i = 0; i < ESC_COUNT; ++i)
    {
      escCommandUs[i] = ESC_NEUTRAL_US;
      esc[i].writeMicroseconds(ESC_NEUTRAL_US);
    }
    armed = true;
    lastHostCommand = millis();
    SerialPort.println(F("STATE,ARMED,THRUSTERS_NEUTRAL"));
    return;
  }
  if (strcmp(command, "C") == 0)
  {
    disarmAll("CALIBRATION_MODE");
    calibrationMode = true;
    SerialPort.println(F("MODE,CALIBRATION"));
    return;
  }
  if (strcmp(command, "N") == 0)
  {
    calibrationMode = false;
    SerialPort.println(F("MODE,NORMAL"));
    return;
  }
  if (strcmp(command, "T") == 0)
  {
    char *channelText = strtok(NULL, " ,\t");
    char *thrustText = strtok(NULL, " ,\t");
    if (channelText == NULL || thrustText == NULL)
    {
      SerialPort.println(F("ERROR,USE_T_channel_thrust"));
      return;
    }
    int channel = atoi(channelText);
    float thrust = atof(thrustText);
    if (channel < 1 || channel > ESC_COUNT || thrust < -1.0f || thrust > 1.0f)
    {
      SerialPort.println(F("ERROR,THRUSTER_RANGE_CHANNEL_1_TO_8_THRUST_MINUS1_TO_1"));
      return;
    }
    if (!armed)
    {
      SerialPort.println(F("ERROR,DISARMED"));
      return;
    }
    uint8_t index = (uint8_t)(channel - 1);
    escCommandUs[index] = thrustToPulse(thrust);
    esc[index].writeMicroseconds(escCommandUs[index]);
    lastHostCommand = millis();
    SerialPort.print(F("ESC_COMMAND,"));
    SerialPort.print(channel);
    SerialPort.print(',');
    SerialPort.println(escCommandUs[index]);
    return;
  }
  if (strcmp(command, "A") == 0)
  {
    char *channelText = strtok(NULL, " ,\t");
    char *stateText = strtok(NULL, " ,\t");
    if (channelText == NULL || stateText == NULL)
    {
      SerialPort.println(F("ERROR,USE_A_channel_0_OR_1"));
      return;
    }
    int channel = atoi(channelText);
    int state = atoi(stateText);
    if (channel < 1 || channel > ACTUATOR_COUNT || (state != 0 && state != 1))
    {
      SerialPort.println(F("ERROR,ACTUATOR_RANGE_CHANNEL_1_TO_8_STATE_0_OR_1"));
      return;
    }
    if (!armed && state == 1)
    {
      SerialPort.println(F("ERROR,DISARMED"));
      return;
    }
    uint8_t index = (uint8_t)(channel - 1);
    actuatorState[index] = state == 1;
    digitalWrite(actuatorPins[index], actuatorState[index] ? HIGH : LOW);
    lastHostCommand = millis();
    SerialPort.print(F("ACTUATOR,"));
    SerialPort.print(channel);
    SerialPort.print(',');
    SerialPort.println(actuatorState[index] ? "ON" : "OFF");
    return;
  }

  SerialPort.println(F("ERROR,UNKNOWN_COMMAND"));
}

void disarmAll(const char *reason)
{
  needsDisarmFirst = true;   // require an explicit host disarm before the next arm
  bool wasArmed = armed;
  armed = false;
  for (uint8_t i = 0; i < ESC_COUNT; ++i)
  {
    escCommandUs[i] = ESC_NEUTRAL_US;
    esc[i].writeMicroseconds(ESC_NEUTRAL_US);
  }
  for (uint8_t i = 0; i < ACTUATOR_COUNT; ++i)
  {
    actuatorState[i] = false;
    digitalWrite(actuatorPins[i], LOW);
  }
  if (wasArmed || strcmp(reason, "COMMAND") == 0)
  {
    SerialPort.print(F("STATE,DISARMED,"));
    SerialPort.println(reason);
  }
}

void printStatus()
{
  SerialPort.print(F("STATUS,STATE,"));
  SerialPort.print(armed ? F("ARMED") : F("DISARMED"));
  SerialPort.print(F(",IMU,"));
  SerialPort.print(imuReady ? F("READY") : F("FAULT"));
  SerialPort.print(F(",MODE,"));
  SerialPort.println(calibrationMode ? F("CALIBRATION") : F("NORMAL"));

  for (uint8_t i = 0; i < ESC_COUNT; ++i)
  {
    SerialPort.print(F("ESC_SIGNAL,"));
    SerialPort.print(i + 1);
    SerialPort.print(',');
    SerialPort.println(escCommandUs[i]);
  }
  for (uint8_t i = 0; i < ACTUATOR_COUNT; ++i)
  {
    SerialPort.print(F("ACTUATOR_STATE,"));
    SerialPort.print(i + 1);
    SerialPort.print(',');
    SerialPort.println(actuatorState[i] ? F("ON") : F("OFF"));
  }
  SerialPort.println(F("NOTE,ESC_SIGNAL_IS_NOT_MOTOR_FEEDBACK"));
}

int thrustToPulse(float thrust)
{
  if (thrust < -1.0f) thrust = -1.0f;
  if (thrust > 1.0f) thrust = 1.0f;
  return ESC_NEUTRAL_US + (int)(thrust * 400.0f);
}

void readAndPrintImu()
{
  float rawAccelX = imu.calcAccel(imu.ax);
  float rawAccelY = imu.calcAccel(imu.ay);
  float rawAccelZ = imu.calcAccel(imu.az);
  float rawGyroX = imu.calcGyro(imu.gx);
  float rawGyroY = imu.calcGyro(imu.gy);
  float rawGyroZ = imu.calcGyro(imu.gz);
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

  SerialPort.print(F("D,"));
  SerialPort.print(accelX, 4); SerialPort.print(',');
  SerialPort.print(accelY, 4); SerialPort.print(',');
  SerialPort.print(accelZ, 4); SerialPort.print(',');
  SerialPort.print(gyroX, 3); SerialPort.print(',');
  SerialPort.print(gyroY, 3); SerialPort.print(',');
  SerialPort.print(gyroZ, 3); SerialPort.print(',');
  SerialPort.println(temp, 1);
}
