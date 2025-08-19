#include <Arduino.h>
#include <imxrt.h>
#include <IntervalTimerEx.h>
#include <IntervalTimer.h>
#include <TimeLib.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_INA219.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MCP9808.h>
#include "TeenGen.h"
#include "MQ8.h"
//#include <OneWire.h>
//#include <DallasTemperature.h>
#include "WireIMXRT.h"


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
#define mcp2Address
#define mcp3Address
#define mcp4Address


MQ8 hSensor;
TeenGen generalTeensy = TeenGen();
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
//OneWire oneWire(ONE_WIRE_BUS);
//DallasTemperature tempSensors(&oneWire);
int sensorReadInterval = 1000000; // 100 Hz
int currentReadInterval = 10000; //10 KHz
int ACS_VpA = 185;
int ACS_zero = 2500;
int ACS_value1, ACS_value2, ACS_value3, ACS_value4 = 0;
double ACS_voltage1, ACS_voltage2, ACS_voltage3, ACS_voltage4 = 0;

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

IntervalTimerEx mhdTimer;
IntervalTimerEx sensorTimer;
IntervalTimerEx currentSensorTimer;

void readSensors()
{
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  
  timestamp_general = now();

  //Read IMU
  sox.getEvent(&accel, &gyro, &temp);
  accelX       = accel.acceleration.x;
  accelY       = accel.acceleration.y;
  accelZ       = accel.acceleration.z;
  gyroX        = gyro.gyro.x;
  gyroY        = gyro.gyro.y;
  gyroZ        = gyro.gyro.z;
  

  //Read hydrogen sensor
  hydrogenVoltage = hSensor.readSensor();

  //Read Zarges box temperature & humidity
  boxTemp = bme.readTemperature();
  boxPres = bme.readPressure() / 100.0F;
  boxHum = bme.readHumidity();

  //Read coil temperature with I2C Sensor
  coilTemp[0] = mcp1.readTempC();
  coilTemp[1] = mcp2.readTempC();
  coilTemp[2] = mcp3.readTempC();
  coilTemp[3] = mcp4.readTempC();

  /*
  //Read coil temperature with One Wire Sensor
  tempSensors.requestTemperatures(); 
  for(int i=0; i<4; i++)
  {
    coilTemp[i] = tempSensors.getTempCByIndex(i);
  }
    */
}

void readCurrent()
{
  timestamp_current = now();

  //INAS
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

  //ACS
  ACS_value1 = analogRead(pinACS1);
  ACS_voltage1 = (ACS_value1 /1024.0) * 5000;
  currentM1 = ((ACS_voltage1 - ACS_zero) / ACS_VpA);

  ACS_value2 = analogRead(pinACS2);
  ACS_voltage2 = (ACS_value2 /1024.0) * 5000;
  currentM2 = ((ACS_voltage2 - ACS_zero) / ACS_VpA);

  ACS_value3 = analogRead(pinACS3);
  ACS_voltage3 = (ACS_value3 /1024.0) * 5000;
  currentM3 = ((ACS_voltage3 - ACS_zero) / ACS_VpA);

  ACS_value4 = analogRead(pinACS4);
  ACS_voltage4 = (ACS_value4 /1024.0) * 5000;
  currentM4 = ((ACS_voltage4 - ACS_zero) / ACS_VpA);
}

void sendSensorData()
{
  int cTimestamp;
  float cAccelX, cAccelY, cAccelZ;
  float cGyroX, cGyroY, cGyroZ;
  float cBoxTemp, cBoxPres, cBoxHum;
  float cCurrentE1, cVoltageE1;
  float cHydrogen;

  noInterrupts();
  cTimestamp = timestamp_general;
  cAccelX = accelX;
  cAccelY = accelY;
  cAccelZ = accelZ;
  cGyroX = gyroX;
  cGyroY = gyroY;
  cGyroZ = gyroZ;
  cBoxTemp = boxTemp;
  cBoxPres = boxPres;
  cBoxHum = boxHum;
  cCurrentE1 = currentE1;
  cVoltageE1 = voltageE1;
  cHydrogen = hydrogenVoltage;
  interrupts();

  Serial.print(cTimestamp); Serial.print(",");
  Serial.print(cAccelX);  Serial.print(",");
  Serial.print(cAccelY);  Serial.print(",");
  Serial.print(cAccelZ);  Serial.print(",");
  Serial.print(cGyroX);   Serial.print(",");
  Serial.print(cGyroY);   Serial.print(",");
  Serial.print(cGyroZ);   Serial.print(",");
  Serial.print(cBoxTemp); Serial.print(",");
  Serial.print(cBoxPres); Serial.print(",");
  Serial.print(cBoxHum);  Serial.print(",");
  Serial.print(cCurrentE1); Serial.print(",");
  Serial.print(cVoltageE1); Serial.print(",");
  Serial.print(cHydrogen);
  Serial.println();
}

void setup()
{
  Serial.begin(9600);
  while(!Serial){}
  delay(200);

  ina1.begin();
  ina2.begin();
  ina3.begin();
  ina4.begin();
  mcp1.begin(mcp1Address);
  mcp2.begin(mcp2Address);
  mcp3.begin(mcp3Address);
  mcp4.begin(mcp4Address);
  mcp1.setResolution(0); //0.5°C Resolution
  mcp2.setResolution(0); //0.5°C Resolution
  mcp3.setResolution(0); //0.5°C Resolution
  mcp4.setResolution(0); //0.5°C Resolution
  mcp1.wake();
  mcp2.wake();
  mcp3.wake();
  mcp4.wake();
  //tempSensors.begin();
  sox.begin_I2C();
  bme.begin();

  mhdTimer.begin([] {generalTeensy.setMHDmode(); }, generalTeensy.intervalEnableMHD);
  sensorTimer.begin([] {readSensors(); }, sensorReadInterval);
  currentSensorTimer.begin([] {readCurrent(); }, currentReadInterval);

}

void loop()
{
  sendSensorData();
  delay(1000);
}

