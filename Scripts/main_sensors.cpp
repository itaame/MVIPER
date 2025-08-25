//Three timers are running on the general teensy
//1. mhdTimer: Reading and setting mode and bubble injection
//  - executes: generalTeensy.setMHDmode()
//  - interval: generalTeensy.intervalEnableMHD (default 10 KHz)
//2. sensorTimer: Reading general sensors
//  - executes: readSensors()
//  - interval: sensorReadInterval (default 100 Hz)
//3. currentSensorTimer: Reading and setting mode and bubble injection
//  - executes: readCurrent()
//  - interval: currentReadInterval (default 10 KHz)

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
#include "WireIMXRT.h"


#define ONE_WIRE_BUS 0
#define SEALEVELPRESSURE_HPA (1013.25) //Sea level pressure value for pressure sensor
#define pinACS1 20 //Pins for ACS sensors
#define pinACS2 21
#define pinACS3 22
#define pinACS4 23
#define ina1Address 0b1000000 //I2C addresses for the INA sensors
#define ina2Address 0b1000001
#define ina3Address 0b1000100
#define ina4Address 0b1000101
#define mcp1Address 0x18 //I2C addresses for temperature sensors
#define mcp2Address 0x19
#define mcp3Address 0x1A
#define mcp4Address 0x1C

//Initalize the teensy, sensors and timers
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
IntervalTimerEx mhdTimer;
IntervalTimerEx sensorTimer;
IntervalTimerEx currentSensorTimer;


int sensorReadInterval = 1000000; // 100 Hz frequency for reading general sensors
int currentReadInterval = 10000; //10 KHz frequency for reading current sensors

//Variables for ACS sensors
int ACS_VpA = 185;
int ACS_zero = 2500;
int ACS_value1, ACS_value2, ACS_value3, ACS_value4 = 0;
double ACS_voltage1, ACS_voltage2, ACS_voltage3, ACS_voltage4 = 0;

//Sensor variables
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

void readSensors()
{
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  
  //Save timestamp
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

}

void readCurrent()
{
  //Save timestamp
  timestamp_current = now();

  //Read INAs
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

  //Read ACS
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

void setup()
{
  //Uncomment the next lines if you need to print something over serial
  //WARNING: The software might not work with these lines if the teensy is connected to the USB hub
  //Serial.begin(9600);
  //delay(500);

  //Configure sensors
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
  sox.begin_I2C();
  bme.begin();

  //Start timers
  mhdTimer.begin([] {generalTeensy.setMHDmode(); }, generalTeensy.intervalEnableMHD); //Reading and setting mode and bubble injection
  sensorTimer.begin([] {readSensors(); }, sensorReadInterval); //Reading general sensors
  currentSensorTimer.begin([] {readCurrent(); }, currentReadInterval); //Reading current sensors
  
}
  //Uncomment the following lines if you want to print sensor values using serial
  /*
  float caccelX = 0;
  float caccelY = 0;
  float caccelZ = 0;
  float cboxTemp = 0;
  float cboxPres = 0;
  float cboxHum = 0;
  float ccurrentE1 = 0;
  float cvoltageE1 = 0;
  float ccurrentE2 = 0;
  float cvoltageE2 = 0;
  float ccurrentE3 = 0;
  float cvoltageE3 = 0;
  float ccurrentE4 = 0;
  float cvoltageE4 = 0;
  float ccurrentM1 = 0;
  float ccurrentM2 = 0;
  float ccurrentM3 = 0;
  float ccurrentM4 = 0;
  float cHyd = 0;
  float cTemp1 = 0;
  float cTemp2 = 0;
  float cTemp3 = 0;
  float cTemp4 = 0;
  */

void loop()
{
  //Uncomment the following lines to print sensor values via serial
  //WARNING: The software might not work with these lines if the teensy is connected to the USB hub
  /*
  Serial.print("MHD Mode = ");
  Serial.print(generalTeensy.signalOut);
  Serial.print("\n");  
  Serial.print("BI Mode = ");
  Serial.print(generalTeensy.stateBI);
  Serial.print("\n");  

  caccelX = 0;
  caccelY = 0;
  caccelZ = 0;
  cboxTemp = 0;
  cboxPres = 0;
  cboxHum = 0;
  ccurrentE1 = 0;
  cvoltageE1 = 0;
  ccurrentE2 = 0;
  cvoltageE2 = 0;
  ccurrentE3 = 0;
  cvoltageE3 = 0;
  ccurrentE4 = 0;
  cvoltageE4 = 0;
  cHyd = 0;
  cTemp1 = 0;
  cTemp2 = 0;
  cTemp3 = 0;
  cTemp4 = 0;
  ccurrentM1 =  0;
  ccurrentM2 =  0;
  ccurrentM3 =  0;
  ccurrentM4 =  0;
  delay(200);
  
  noInterrupts();
  caccelX = accelX;
  caccelY = accelY;
  caccelZ = accelZ;
  cboxTemp = boxTemp;
  cboxPres = boxPres;
  cboxHum = boxHum;
  ccurrentE1 = currentE1;
  cvoltageE1 = voltageE1;
  ccurrentE2 = currentE2;
  cvoltageE2 = voltageE2;
  ccurrentE3 = currentE3;
  cvoltageE3 = voltageE3;
  ccurrentE4 = currentE3;
  cvoltageE4 = voltageE3;
  ccurrentM1 =  currentM1;
  ccurrentM2 =  currentM2;
  ccurrentM3 =  currentM3;
  ccurrentM4 =  currentM4;
  cHyd = hydrogenVoltage;
  cTemp1 = coilTemp[0];
  interrupts();

  Serial.print("Current E1= ");
  Serial.print(ccurrentE1);
  Serial.print(" Voltage E1= ");
  Serial.print(cvoltageE1);
  Serial.print("\n");
  Serial.print("Current E2= ");
  Serial.print(ccurrentE2);
  Serial.print(" Voltage E2= ");
  Serial.print(cvoltageE2);
  Serial.print("\n");
  Serial.print("Current E3= ");
  Serial.print(ccurrentE3);
  Serial.print(" Voltage E3= ");
  Serial.print(cvoltageE3);
  Serial.print("\n");
  Serial.print("Current E4= ");
  Serial.print(ccurrentE4);
  Serial.print(" Voltage E4= ");
  Serial.print(cvoltageE4);
  Serial.print("\n");

  Serial.print("Current M1 = ");
  Serial.print(ccurrentE1);
  Serial.print(" Current M2 = ");
  Serial.print(ccurrentE2);
  Serial.print(" Current M3 = ");
  Serial.print(ccurrentE3);
  Serial.print(" Current M4 = ");
  Serial.print(ccurrentE4);
  Serial.print("\n");
  

  
  Serial.print("acelX = ");
  Serial.print(caccelX);
  Serial.print(" acelY = ");
  Serial.print(caccelY);
  Serial.print(" acelZ = ");
  Serial.print(caccelZ);
  Serial.print("\n");

  Serial.print("Temp = ");
  Serial.print(cboxTemp);
  Serial.print(" Pressure = ");
  Serial.print(cboxPres);
  Serial.print(" Humidity = ");
  Serial.print(cboxHum);
  Serial.print("\n");
  
  Serial.print("CoilTemp1 = ");
  Serial.print(cTemp1);
  Serial.print("\n");
  
  Serial.print("Hydrogen voltage = ");
  Serial.print(cHyd);
  Serial.print("\n");
  Serial.print("\n");
  delay(2000);
  
  */
}

