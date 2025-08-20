#include <Arduino.h>
#include <imxrt.h>
#include "core_pins.h"
#include "debug/printf.h"
#include "math.h"
#include "TeenPWM.h"
#include "Wire.h"
#include <IntervalTimerEx.h>

TeenPWM TeensyPWM = TeenPWM();
IntervalTimerEx readingTimer;

void setup() 
{
  pinMode(TeensyPWM.s1_pin, INPUT_PULLDOWN);
  pinMode(TeensyPWM.s2_pin, INPUT_PULLDOWN);
  pinMode(TeensyPWM.s3_pin, INPUT_PULLDOWN);
  pinMode(TeensyPWM.s4_pin, INPUT_PULLDOWN);
  

  //Serial.begin(9600);
  //while(!Serial){}

  //Comment out the next line for manual operation:
  readingTimer.begin([] {TeensyPWM.set_Mode(); }, TeensyPWM.intervalRead);


  //Uncomment the lines below for manual operation. Set the values accordingly
  /*float frequency = 100; //set the frequency [Hz]
  float deadtime_electrodes = 10*powf(10, -6); //set the deadtime [s] for the electrodes (pins 2 & 3)
  float deadtime_magnets    = 10*powf(10, -6); //set the deadtime [s] for the magnets (pins 22 & 23)
  
  TeensyPWM.setOutputPWM_freq(frequency, deadtime_electrodes, deadtime_magnets);
  //TeensyPWM.pwmDC();*/
  
}

void loop() 
{
  /*
  Serial.print("Mode = ");
  Serial.print(TeensyPWM.s);
  Serial.print("\n");
  Serial.print("Frequency = ");
  Serial.print(TeensyPWM.PWMfreq);
  Serial.print("\n");

  delay(2000);
  */
}
