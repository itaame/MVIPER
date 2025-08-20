#include <Arduino.h>
#include <imxrt.h>
#include "core_pins.h"
#include "debug/printf.h"
#include "math.h"
#include "TeenPWM.h"

TeenPWM::TeenPWM()
{

  s1 = 0;
  s2 = 0;
  s3 = 0;
  s4 = 0;

  s = 0;

  intervalRead = 10000;
  deadtime_electrodes = 10*powf(10, -6); //set the deadtime [s] for the electrodes (pins 2 & 3)
  deadtime_magnets    = 10*powf(10, -6); //set the deadtime [s] for the magnets (pins 22 & 23)

  In1 = 6;  // Complementary PWM signals for the electrodes are output on pins 6 & 9
  In2 = 9;
  In3 = 36;  // Complementary PWM signals for the magnets are output on pins 36 & 37
  In4 = 37;
  EnA = 4;  // Enable Signal for Module A (In1 & In2) is output on pin 4
  EnB = 5;   // Enable Signal for Module B (In3 & In4) is output on pin 5
  MaskA = 4; //Masks are used for addressing the correct FlexPWM submodules
  MaskB = 8;
  MaskEnA = 1;
  MaskEnB = 2;
  ResolutionPWM = 12; //12-Bit resolution for PWM
  PWMfreq = 100;
}


void TeenPWM::setOutputFrequency(float frequency) //This function automatically adjusts the prescaler to set the correct frequency
{
  uint32_t CurrentCycles = (uint32_t)((float)F_BUS_ACTUAL / frequency + 0.5);
  uint32_t Prescaler = 0;

  //The prescaler is adjusted automatically if the freqency requires it
  while (CurrentCycles > 65535 && Prescaler < 7) {
  CurrentCycles = CurrentCycles >> 1;
  Prescaler = Prescaler + 1;
  }
  if (CurrentCycles > 65535) {
  CurrentCycles = 65535;
  } else if (CurrentCycles < 2) {
  CurrentCycles = 2; //minimal cycle duration --> around 10nS = 100 MHz
  }

  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_CLDOK(MaskA);
  FLEXPWM2_SM2CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_PRSC(Prescaler); //Erst nach vollem Cycle updaten und Prescaler setzen!
  FLEXPWM2_SM2VAL1 = CurrentCycles - 1; //Cycles für Periodendauer setzen!
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK(MaskA);

  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_CLDOK(MaskB);
  FLEXPWM2_SM3CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_PRSC(Prescaler); //Erst nach vollem Cycle updaten und Prescaler setzen!
  FLEXPWM2_SM3VAL1 = CurrentCycles - 1; //Cycles für Periodendauer setzen!
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK(MaskB);

  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_CLDOK(MaskEnA);
  FLEXPWM2_SM0CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_PRSC(Prescaler); //Erst nach vollem Cycle updaten und Prescaler setzen!
  FLEXPWM2_SM0VAL1 = CurrentCycles - 1; //Cycles für Periodendauer setzen!
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK(MaskEnA);

  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_CLDOK(MaskEnB);
  FLEXPWM2_SM1CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_PRSC(Prescaler); //Erst nach vollem Cycle updaten und Prescaler setzen!
  FLEXPWM2_SM1VAL1 = CurrentCycles - 1; //Cycles für Periodendauer setzen!
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK(MaskEnB);

}


void TeenPWM::setOutputPWM_freq(float freq, float deadtime_e_s, float deadtime_m_s) //This function first calls setOutputFrequency() then sets up the pwm signals including the deadtime
{
  //Setting the frequency
  setOutputFrequency(freq);

  //Calculating the deadtime for the timer
  float deadtime_e = (float)FLEXPWM2_SM2VAL1*deadtime_e_s*freq;
  float deadtime_m = (float)FLEXPWM2_SM2VAL1*deadtime_m_s*freq;

  //Setting up Pins 6 & 9 -> Magnets
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_CLDOK(MaskA);
  FLEXPWM2_SM2VAL2 = FLEXPWM2_SM2INIT;
  FLEXPWM2_SM2VAL3 = (uint32_t)((FLEXPWM2_SM2VAL1-1))/2;
  FLEXPWM2_SM2VAL4 = FLEXPWM2_SM2VAL3;
  FLEXPWM2_SM2VAL5 = FLEXPWM2_SM2VAL1 - 1;
  FLEXPWM2_OUTEN |= FLEXPWM_OUTEN_PWMA_EN(MaskA); //In1 Output enable
  FLEXPWM2_OUTEN |= FLEXPWM_OUTEN_PWMB_EN(MaskA); //In2 Output enable
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK(MaskA);
  *(portConfigRegister(In1)) = 2; //Set pin 6 to output the PWM signal
  *(portConfigRegister(In2)) = 2; //Set pin 9 to output the PWM signal
  

  //Setting up Pins 36 & 37 -> Electrodes
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_CLDOK(MaskB);
  FLEXPWM2_SM3VAL2 =  (uint32_t)((FLEXPWM2_SM3VAL1-1)/8);
  FLEXPWM2_SM3VAL3 = (uint32_t)(5*(FLEXPWM2_SM3VAL1-1)/8);
  FLEXPWM2_SM3VAL4 = FLEXPWM2_SM3VAL3;
  FLEXPWM2_SM3VAL5 = FLEXPWM2_SM3VAL2;
  FLEXPWM2_OUTEN |= FLEXPWM_OUTEN_PWMA_EN(MaskB); //In3 Output enable
  FLEXPWM2_OUTEN |= FLEXPWM_OUTEN_PWMB_EN(MaskB); //In4 Output enable
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK(MaskB);
  *(portConfigRegister(In3)) = 6; //Set pin 36 to output the PWM signal
  *(portConfigRegister(In4)) = 6; //Set pin 37 to output the PWM signal

  //Setting up Pins 4 -> Magnets
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_CLDOK(MaskEnA);
  FLEXPWM2_SM0CTRL |= FLEXPWM_SMCTRL_DBLEN;
  FLEXPWM2_SM0VAL2 = FLEXPWM2_SM2VAL2 + (uint32_t)(deadtime_e / 2);
  FLEXPWM2_SM0VAL4 = FLEXPWM2_SM2VAL3 - (uint32_t)(deadtime_e / 2);
  FLEXPWM2_SM0VAL5 = FLEXPWM2_SM0VAL4 + (uint32_t)(deadtime_e / 2);
  FLEXPWM2_SM0VAL3 = FLEXPWM2_SM2VAL5 - (uint32_t)(deadtime_e / 2);
  FLEXPWM2_OUTEN |= FLEXPWM_OUTEN_PWMA_EN(MaskEnA); //EnA Output enable
  FLEXPWM2_OUTEN |= FLEXPWM_OUTEN_PWMB_EN(MaskEnA); 
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK(MaskEnA);
  *(portConfigRegister(EnA)) = 1; //Set pin 4 to output the PWM signal

  //Setting up Pin 5 -> Electrodes
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_CLDOK(MaskEnB);
  FLEXPWM2_SM1CTRL |= FLEXPWM_SMCTRL_DBLEN;
  FLEXPWM2_SM1VAL2 = FLEXPWM2_SM3VAL2 + (uint32_t)(deadtime_m / 2);
  FLEXPWM2_SM1VAL4 = FLEXPWM2_SM3VAL3 - (uint32_t)(deadtime_m / 2);
  FLEXPWM2_SM1VAL5 = FLEXPWM2_SM3VAL4 + (uint32_t)(deadtime_m / 2);
  FLEXPWM2_SM1VAL3 = FLEXPWM2_SM3VAL5 - (uint32_t)(deadtime_m / 2);
  FLEXPWM2_OUTEN |= FLEXPWM_OUTEN_PWMA_EN(MaskEnB); //EnB Output enable
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK(MaskEnB);
  *(portConfigRegister(EnB)) = 1; //Set pin 5 to output the PWM signal
  

  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_RUN(MaskA);
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_RUN(MaskB);
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_RUN(MaskEnA);
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_RUN(MaskEnB);
  }

void TeenPWM::pwmOn() //This function turns the pwm signals on (Only use after settOutputPWM_freq)
{
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK(MaskA);
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_RUN(MaskA);
  *(portConfigRegister(In1)) = 2;
  *(portConfigRegister(In2)) = 2;

  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK(MaskB);
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_RUN(MaskB);
  *(portConfigRegister(In3)) = 6;
  *(portConfigRegister(In4)) = 6;

  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK(MaskEnA);
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_RUN(MaskEnA);
  *(portConfigRegister(EnA)) = 1;

  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK(MaskEnB);
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_RUN(MaskEnB);
  *(portConfigRegister(EnB)) = 1;
}

void TeenPWM::pwmOff() //This function turns the pwm signals off (Only use after settOutputPWM_freq)
{
  FLEXPWM2_MCTRL &= ~ FLEXPWM_MCTRL_RUN(MaskA);
  FLEXPWM2_MCTRL &= ~ FLEXPWM_MCTRL_RUN(MaskB);
  FLEXPWM2_MCTRL &= ~ FLEXPWM_MCTRL_RUN(MaskEnA);
  FLEXPWM2_MCTRL &= ~ FLEXPWM_MCTRL_RUN(MaskEnB);
  
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);
  pinMode(EnA, OUTPUT);
  pinMode(EnB, OUTPUT);

  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);
  digitalWrite(In3, LOW);
  digitalWrite(In4, LOW);
  digitalWrite(EnA, LOW);
  digitalWrite(EnB, LOW);
  
}

void TeenPWM::pwmDC() //For DC with electromagnets: Sends power to electrodes & magnets
{
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);
  pinMode(EnA, OUTPUT);
  pinMode(EnB, OUTPUT);

  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
  digitalWrite(EnA, HIGH);
  digitalWrite(EnB, HIGH);
}

void TeenPWM::standardDC() //For DC with permanent magnets: Sends power to the electrodes, does not power magnets
{
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);
  pinMode(EnA, OUTPUT);
  pinMode(EnB, OUTPUT);

  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
  digitalWrite(EnA, LOW);
  digitalWrite(EnB, HIGH);
}

void TeenPWM::set_Mode() //Reads inputs from general teensy and sets correct mode
{
  s1 = digitalReadFast(s1_pin);
  s2 = digitalReadFast(s2_pin);
  s3 = digitalReadFast(s3_pin);
  s4 = digitalReadFast(s4_pin);

  s = (s1<<3) | (s2<<2) | (s3<<1) | s4;

  if(s == 1)
  {
    // DC case using the permanent magnets (only powering electrodes)
    standardDC();
  }
  else if (s == 2)
  {
    // DC case using the electromagnets
    pwmDC();
  }
  else if ((s >= 3) && (s <= 15))
  {
    //AC case
    PWMfreq = freq_array[s-3];
    setOutputPWM_freq(PWMfreq, deadtime_electrodes, deadtime_magnets);
    
  }
  else
  {
    // Off
    pwmOff();
  }

}


