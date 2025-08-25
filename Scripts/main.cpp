#include <Arduino.h>
#include <imxrt.h>
#include <IntervalTimer.h>
#include <IntervalTimerEx.h>
#include <TimeLib.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_INA219.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MCP9808.h>
#include "TeenGen.h"
#include "MQ8.h"
#include "WireIMXRT.h"

// Pin and address configuration
#define ONE_WIRE_BUS 0
#define SEALEVELPRESSURE_HPA (1013.25)
#define pinACS1 20
#define pinACS2 21
#define pinACS3 22
#define pinACS4 23
#define ina1Address 0b1000000
#define ina2Address 0b1000001
#define ina3Address 0b1000100
#define ina4Address 0b1000101
#define mcp1Address 0x18
#define mcp2Address 0x19
#define mcp3Address 0x1A
#define mcp4Address 0x1C

// Instantiate sensors and helpers
TeenGen generalTeensy = TeenGen();
MQ8 hSensor;
Adafruit_LSM6DSOX sox;
Adafruit_MCP9808 mcp1;
Adafruit_MCP9808 mcp2;
Adafruit_MCP9808 mcp3;
Adafruit_MCP9808 mcp4;
Adafruit_BME280 bme;
Adafruit_INA219 ina1 = Adafruit_INA219(ina1Address);
Adafruit_INA219 ina2 = Adafruit_INA219(ina2Address);
Adafruit_INA219 ina3 = Adafruit_INA219(ina3Address);
Adafruit_INA219 ina4 = Adafruit_INA219(ina4Address);

// Timers
IntervalTimerEx mhdTimer;
IntervalTimerEx sensorTimer;
IntervalTimerEx currentSensorTimer;
IntervalTimer sendTimer;

// Intervals in microseconds
int sensorReadInterval = 1'000'000;   // 100 Hz
int currentReadInterval = 10'000;     // 10 KHz

// ACS current sensor parameters
int ACS_VpA = 185;
int ACS_zero = 2500;
int ACS_value1, ACS_value2, ACS_value3, ACS_value4 = 0;
double ACS_voltage1, ACS_voltage2, ACS_voltage3, ACS_voltage4 = 0;

// Shared sensor variables
volatile int timestamp_general;
volatile int timestamp_current;
volatile int hydrogenVoltage;
volatile float accelX, accelY, accelZ;
volatile float gyroX, gyroY, gyroZ;
volatile float currentE1, currentE2, currentE3, currentE4;
volatile float voltageE1, voltageE2, voltageE3, voltageE4;
volatile float shuntvoltage1, shuntvoltage2, shuntvoltage3, shuntvoltage4;
volatile float busvoltage1, busvoltage2, busvoltage3, busvoltage4;
double currentM1, currentM2, currentM3, currentM4;
volatile float coilTemp[4];
volatile float boxTemp;
volatile float boxPres;
volatile float boxHum;

// Read general sensors (IMU, gas, temps)
void readSensors() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  timestamp_general = now();

  sox.getEvent(&accel, &gyro, &temp);
  accelX = accel.acceleration.x;
  accelY = accel.acceleration.y;
  accelZ = accel.acceleration.z;
  gyroX = gyro.gyro.x;
  gyroY = gyro.gyro.y;
  gyroZ = gyro.gyro.z;

  hydrogenVoltage = hSensor.readSensor();

  boxTemp = bme.readTemperature();
  boxPres = bme.readPressure() / 100.0F;
  boxHum = bme.readHumidity();

  coilTemp[0] = mcp1.readTempC();
  coilTemp[1] = mcp2.readTempC();
  coilTemp[2] = mcp3.readTempC();
  coilTemp[3] = mcp4.readTempC();
}

// Read current sensors
void readCurrent() {
  timestamp_current = now();

  currentE1 = ina1.getCurrent_mA();
  busvoltage1 = ina1.getBusVoltage_V();
  shuntvoltage1 = ina1.getShuntVoltage_mV();
  voltageE1 = busvoltage1 + (shuntvoltage1 / 1000);

  currentE2 = ina2.getCurrent_mA();
  busvoltage2 = ina2.getBusVoltage_V();
  shuntvoltage2 = ina2.getShuntVoltage_mV();
  voltageE2 = busvoltage2 + (shuntvoltage2 / 1000);

  currentE3 = ina3.getCurrent_mA();
  busvoltage3 = ina3.getBusVoltage_V();
  shuntvoltage3 = ina3.getShuntVoltage_mV();
  voltageE3 = busvoltage3 + (shuntvoltage3 / 1000);

  currentE4 = ina4.getCurrent_mA();
  busvoltage4 = ina4.getBusVoltage_V();
  shuntvoltage4 = ina4.getShuntVoltage_mV();
  voltageE4 = busvoltage4 + (shuntvoltage4 / 1000);

  ACS_value1 = analogRead(pinACS1);
  ACS_voltage1 = (ACS_value1 / 1024.0) * 5000;
  currentM1 = ((ACS_voltage1 - ACS_zero) / ACS_VpA);

  ACS_value2 = analogRead(pinACS2);
  ACS_voltage2 = (ACS_value2 / 1024.0) * 5000;
  currentM2 = ((ACS_voltage2 - ACS_zero) / ACS_VpA);

  ACS_value3 = analogRead(pinACS3);
  ACS_voltage3 = (ACS_value3 / 1024.0) * 5000;
  currentM3 = ((ACS_voltage3 - ACS_zero) / ACS_VpA);

  ACS_value4 = analogRead(pinACS4);
  ACS_voltage4 = (ACS_value4 / 1024.0) * 5000;
  currentM4 = ((ACS_voltage4 - ACS_zero) / ACS_VpA);
}

// Send all collected data at 10 kHz
void sendData() {
  Serial.print(timestamp_general); Serial.print(',');
  Serial.print(hydrogenVoltage); Serial.print(',');
  Serial.print(accelX); Serial.print(',');
  Serial.print(accelY); Serial.print(',');
  Serial.print(accelZ); Serial.print(',');
  Serial.print(gyroX); Serial.print(',');
  Serial.print(gyroY); Serial.print(',');
  Serial.print(gyroZ); Serial.print(',');
  Serial.print(boxTemp); Serial.print(',');
  Serial.print(boxPres); Serial.print(',');
  Serial.print(boxHum); Serial.print(',');
  Serial.print(coilTemp[0]); Serial.print(',');
  Serial.print(coilTemp[1]); Serial.print(',');
  Serial.print(coilTemp[2]); Serial.print(',');
  Serial.print(coilTemp[3]); Serial.print(',');
  Serial.print(currentE1); Serial.print(',');
  Serial.print(voltageE1); Serial.print(',');
  Serial.print(currentE2); Serial.print(',');
  Serial.print(voltageE2); Serial.print(',');
  Serial.print(currentE3); Serial.print(',');
  Serial.print(voltageE3); Serial.print(',');
  Serial.print(currentE4); Serial.print(',');
  Serial.print(voltageE4); Serial.print(',');
  Serial.print(currentM1); Serial.print(',');
  Serial.print(currentM2); Serial.print(',');
  Serial.print(currentM3); Serial.print(',');
  Serial.print(currentM4); Serial.print(',');
  Serial.print(timestamp_current); Serial.print('\n');
}

void setup() {
  Serial.begin(5'000'000);  // High baud rate for 10 kHz transfers
  while (!Serial) {
    // Wait for the serial port to open
  }

  // Configure sensors
  ina1.begin();
  ina2.begin();
  ina3.begin();
  ina4.begin();
  mcp1.begin(mcp1Address);
  mcp2.begin(mcp2Address);
  mcp3.begin(mcp3Address);
  mcp4.begin(mcp4Address);
  mcp1.setResolution(0);
  mcp2.setResolution(0);
  mcp3.setResolution(0);
  mcp4.setResolution(0);
  mcp1.wake();
  mcp2.wake();
  mcp3.wake();
  mcp4.wake();
  sox.begin_I2C();
  bme.begin();

  // Start timers that continuously update sensor values
  mhdTimer.begin([] { generalTeensy.setMHDmode(); }, generalTeensy.intervalEnableMHD);
  sensorTimer.begin([] { readSensors(); }, sensorReadInterval);
  currentSensorTimer.begin([] { readCurrent(); }, currentReadInterval);

  // Start serial transmission timer at 10 kHz (every 100 microseconds)
  sendTimer.begin(sendData, 100);
}

void loop() {
  // Intentionally empty. All work is done in interrupts.
}

