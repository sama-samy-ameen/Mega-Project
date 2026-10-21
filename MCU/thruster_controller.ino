#include <MPU6500-DMP.h>
#include <Servo.h>
#include <Wire.h>
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
const uint8_t ACTUATOR_COUNT = 6;
const uint8_t escPins[ESC_COUNT] = {2, 3, 4, 5, 6, 7, 8, 9};
const uint8_t actuatorPins[ACTUATOR_COUNT] = {10, 12, 13, A0, A1, A2};
const uint8_t CYTRON_PWM_PIN = 11;
const uint8_t CYTRON_DIRECTION_PIN = A3;
const uint8_t CYTRON_CW_DIRECTION = HIGH;
const uint8_t CYTRON_SPEED_DUTY = 64;  // fixed 25% PWM duty
const uint8_t CYTRON_ACTUATOR_FLAG = 6;  // seventh on/off signal in actuatorMask
const int ESC_MIN_US = 1262;
const int ESC_NEUTRAL_US = 1500;
const int ESC_MAX_US = 1736;
const unsigned long ESC_NEUTRAL_STARTUP_MS = 3000;
const unsigned long HOST_TIMEOUT_MS = 500;
const uint8_t WATCHDOG_TIMEOUT = WDTO_500MS;  // reboots the board if loop() hangs
const unsigned long PRESSURE_PERIOD_MS = 100;
const unsigned long PRESSURE_CONVERSION_MS = 20;
const uint8_t PRESSURE_ZERO_SAMPLES = 20;
const float FLUID_DENSITY_KG_M3 = 997.0f;
const uint8_t PRESSURE_I2C_ADDRESS = 0x76;
const uint8_t PRESSURE_RESET_COMMAND = 0x1E;
const uint8_t PRESSURE_ADC_READ_COMMAND = 0x00;
const uint8_t PRESSURE_PROM_READ_COMMAND = 0xA0;
const uint8_t PRESSURE_CONVERT_D1_COMMAND = 0x4A;
const uint8_t PRESSURE_CONVERT_D2_COMMAND = 0x5A;
const uint8_t PACKET_LENGTH = 22;
const uint8_t PACKET_SYNC_0 = 0xA5;
const uint8_t PACKET_SYNC_1 = 0x5A;

MPU6500_DMP imu;
Servo esc[ESC_COUNT];
int escCommandUs[ESC_COUNT];
bool actuatorState[ACTUATOR_COUNT];
bool cytronEnabled = false;
bool armed = false;
bool needsDisarmFirst = true;   // after boot or any disarm, host must send a disarm before arming
bool imuReady = false;
bool pressureReady = false;
bool calibrationMode = false;
unsigned long lastHostCommand = 0;
unsigned long lastPressureRead = 0;
float depthOffset = 0.0f;
float filteredDepth = 0.0f;
uint16_t pressureProm[8] = {0};
uint32_t pressureD1 = 0;
uint32_t pressureD2 = 0;
int32_t pressureTemperatureCentiC = 0;
int32_t pressureRaw = 0;
enum PressureState { PRESSURE_IDLE, PRESSURE_WAIT_D1, PRESSURE_WAIT_D2 };
PressureState pressureState = PRESSURE_IDLE;
unsigned long pressureConversionStarted = 0;
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
void initializePressureSensor();
void servicePressureSensor();
bool startPressureConversion(uint8_t command);
bool readPressureAdc(uint32_t &value);
uint8_t pressurePromCrc4(uint16_t *prom);
void calculatePressure();
int thrustToPulse(float thrust);
void setCytronMotor(bool enabled);

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
  pinMode(CYTRON_PWM_PIN, OUTPUT);
  pinMode(CYTRON_DIRECTION_PIN, OUTPUT);
  digitalWrite(CYTRON_DIRECTION_PIN, CYTRON_CW_DIRECTION);
  analogWrite(CYTRON_PWM_PIN, 0);

  for (uint8_t i = 0; i < ESC_COUNT; ++i)
  {
    escCommandUs[i] = ESC_NEUTRAL_US;
    esc[i].attach(escPins[i], ESC_MIN_US, ESC_MAX_US);
    esc[i].writeMicroseconds(ESC_NEUTRAL_US);
  }

  SerialPort.begin(115200);
  Wire.begin();
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

  initializePressureSensor();

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

  servicePressureSensor();
}

void initializePressureSensor()
{
  Wire.beginTransmission(PRESSURE_I2C_ADDRESS);
  Wire.write(PRESSURE_RESET_COMMAND);
  if (Wire.endTransmission() != 0)
  {
    SerialPort.println(F("SENSOR_FAULT,MS5837_NOT_FOUND,DEPTH_UNAVAILABLE"));
    return;
  }
  delay(10);

  for (uint8_t index = 0; index < 7; ++index)
  {
    Wire.beginTransmission(PRESSURE_I2C_ADDRESS);
    Wire.write(PRESSURE_PROM_READ_COMMAND + index * 2);
    if (Wire.endTransmission() != 0 || Wire.requestFrom(PRESSURE_I2C_ADDRESS, (uint8_t)2) != 2)
    {
      SerialPort.println(F("SENSOR_FAULT,MS5837_PROM_READ,DEPTH_UNAVAILABLE"));
      return;
    }
    pressureProm[index] = ((uint16_t)Wire.read() << 8) | Wire.read();
  }
  pressureProm[7] = 0;
  uint8_t storedCrc = pressureProm[0] >> 12;
  if (pressurePromCrc4(pressureProm) != storedCrc)
  {
    SerialPort.println(F("SENSOR_FAULT,MS5837_PROM_CRC,DEPTH_UNAVAILABLE"));
    return;
  }

  SerialPort.println(F("PRESSURE_ZERO,KEEP_SENSOR_AT_SURFACE"));
  float depthSum = 0.0f;
  for (uint8_t sample = 0; sample < PRESSURE_ZERO_SAMPLES; ++sample)
  {
    if (!startPressureConversion(PRESSURE_CONVERT_D1_COMMAND)) return;
    delay(PRESSURE_CONVERSION_MS);
    if (!readPressureAdc(pressureD1) || !startPressureConversion(PRESSURE_CONVERT_D2_COMMAND)) return;
    delay(PRESSURE_CONVERSION_MS);
    if (!readPressureAdc(pressureD2)) return;
    calculatePressure();
    depthSum += ((pressureRaw * 10.0f) - 101300.0f) /
                (FLUID_DENSITY_KG_M3 * 9.80665f);
  }
  depthOffset = depthSum / PRESSURE_ZERO_SAMPLES;
  filteredDepth = 0.0f;
  pressureReady = true;
  lastPressureRead = millis();
  SerialPort.println(F("SENSOR,MS5837,READY"));
}

void servicePressureSensor()
{
  if (!pressureReady) return;
  unsigned long now = millis();

  if (pressureState == PRESSURE_IDLE)
  {
    if (now - lastPressureRead < PRESSURE_PERIOD_MS) return;
    if (!startPressureConversion(PRESSURE_CONVERT_D1_COMMAND)) return;
    pressureState = PRESSURE_WAIT_D1;
    pressureConversionStarted = now;
    return;
  }

  if (now - pressureConversionStarted < PRESSURE_CONVERSION_MS) return;

  if (pressureState == PRESSURE_WAIT_D1)
  {
    if (!readPressureAdc(pressureD1) || !startPressureConversion(PRESSURE_CONVERT_D2_COMMAND))
    {
      pressureReady = false;
      SerialPort.println(F("SENSOR_FAULT,MS5837_READ,DEPTH_UNAVAILABLE"));
      return;
    }
    pressureState = PRESSURE_WAIT_D2;
    pressureConversionStarted = now;
    return;
  }

  if (!readPressureAdc(pressureD2))
  {
    pressureReady = false;
    SerialPort.println(F("SENSOR_FAULT,MS5837_READ,DEPTH_UNAVAILABLE"));
    return;
  }

  calculatePressure();
  float depthMeters = ((pressureRaw * 10.0f) - 101300.0f) /
                      (FLUID_DENSITY_KG_M3 * 9.80665f) - depthOffset;
  filteredDepth = 0.7f * filteredDepth + 0.3f * depthMeters;

  SerialPort.print(F("P,"));
  SerialPort.println(filteredDepth, 3);
  pressureState = PRESSURE_IDLE;
  lastPressureRead = now;
}

bool startPressureConversion(uint8_t command)
{
  Wire.beginTransmission(PRESSURE_I2C_ADDRESS);
  Wire.write(command);
  return Wire.endTransmission() == 0;
}

bool readPressureAdc(uint32_t &value)
{
  Wire.beginTransmission(PRESSURE_I2C_ADDRESS);
  Wire.write(PRESSURE_ADC_READ_COMMAND);
  if (Wire.endTransmission() != 0 || Wire.requestFrom(PRESSURE_I2C_ADDRESS, (uint8_t)3) != 3)
  {
    return false;
  }
  value = ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | Wire.read();
  return true;
}

uint8_t pressurePromCrc4(uint16_t *prom)
{
  uint16_t remainder = 0;
  prom[0] &= 0x0FFF;
  prom[7] = 0;
  for (uint8_t index = 0; index < 16; ++index)
  {
    remainder ^= (index & 1) ? (prom[index >> 1] & 0x00FF) : (prom[index >> 1] >> 8);
    for (uint8_t bit = 0; bit < 8; ++bit)
    {
      remainder = (remainder & 0x8000) ? (remainder << 1) ^ 0x3000 : remainder << 1;
    }
  }
  return (remainder >> 12) & 0x0F;
}

void calculatePressure()
{
  int32_t deltaTemperature = (int32_t)(pressureD2 - (uint32_t)pressureProm[5] * 256UL);
  int64_t sensitivity = (int64_t)pressureProm[1] * 32768L +
                        ((int64_t)pressureProm[3] * deltaTemperature) / 256L;
  int64_t offset = (int64_t)pressureProm[2] * 65536L +
                   ((int64_t)pressureProm[4] * deltaTemperature) / 128L;
  int32_t temperature = 2000L +
                        (int32_t)(((int64_t)deltaTemperature * pressureProm[6]) / 8388608LL);
  int32_t temperatureCorrection = 0;
  int64_t offsetCorrection = 0;
  int64_t sensitivityCorrection = 0;

  if (temperature < 2000)
  {
    temperatureCorrection = (int32_t)(((int64_t)3 * deltaTemperature * deltaTemperature) / 8589934592LL);
    offsetCorrection = (int64_t)3 * (temperature - 2000L) * (temperature - 2000L) / 2;
    sensitivityCorrection = (int64_t)5 * (temperature - 2000L) * (temperature - 2000L) / 8;
    if (temperature < -1500)
    {
      offsetCorrection += (int64_t)7 * (temperature + 1500L) * (temperature + 1500L);
      sensitivityCorrection += (int64_t)4 * (temperature + 1500L) * (temperature + 1500L);
    }
  }
  else
  {
    temperatureCorrection = (int32_t)(((int64_t)2 * deltaTemperature * deltaTemperature) / 137438953472LL);
    offsetCorrection = (int64_t)(temperature - 2000L) * (temperature - 2000L) / 16;
  }

  temperature -= temperatureCorrection;
  pressureTemperatureCentiC = temperature;
  sensitivity -= sensitivityCorrection;
  offset -= offsetCorrection;
  pressureRaw = (int32_t)((((int64_t)pressureD1 * sensitivity) / 2097152LL - offset) / 8192LL);
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
  setCytronMotor(armed && ((actuatorMask & (1U << CYTRON_ACTUATOR_FLAG)) != 0));

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
    if (channel < 1 || channel > ACTUATOR_COUNT + 1 || (state != 0 && state != 1))
    {
      SerialPort.println(F("ERROR,ACTUATOR_RANGE_CHANNEL_1_TO_7_STATE_0_OR_1"));
      return;
    }
    if (!armed && state == 1)
    {
      SerialPort.println(F("ERROR,DISARMED"));
      return;
    }
    if (channel == ACTUATOR_COUNT + 1)
    {
      setCytronMotor(armed && state == 1);
      lastHostCommand = millis();
      SerialPort.print(F("CYTRON_CLOCKWISE,"));
      SerialPort.println(cytronEnabled ? F("ON") : F("OFF"));
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
  setCytronMotor(false);
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
  SerialPort.print(F("CYTRON_CLOCKWISE,"));
  SerialPort.print(cytronEnabled ? F("ON") : F("OFF"));
  SerialPort.print(F(",DUTY,"));
  SerialPort.println(CYTRON_SPEED_DUTY);
  SerialPort.println(F("NOTE,ESC_SIGNAL_IS_NOT_MOTOR_FEEDBACK"));
}

void setCytronMotor(bool enabled)
{
  cytronEnabled = enabled;
  digitalWrite(CYTRON_DIRECTION_PIN, CYTRON_CW_DIRECTION);
  analogWrite(CYTRON_PWM_PIN, enabled ? CYTRON_SPEED_DUTY : 0);
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
