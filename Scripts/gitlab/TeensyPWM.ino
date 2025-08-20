#include "imxrt.h"
#include "core_pins.h"
#include "debug/printf.h"
#include "math.h"

#define ChannelA 2 // Complementary PWM signals for the electrodes are output on pins 2 & 3
#define ChannelB 3
#define ChannelC 22 // Complementary PWM signals for the magnets are output on pins 22 & 23
#define ChannelD 23
#define Maske1 4
#define Maske2 1
#define Maske3 2
#define ResolutionPWM 12 //12-Bit resolution for PWM

void setOutputFrequency(float frequency)
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
  CurrentCycles = 2; //minimale Cycles --> around 10nS 
  }

  FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_CLDOK(Maske1);
  FLEXPWM4_SM2CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_PRSC(Prescaler); //Erst nach vollem Cycle updaten und Prescaler setzen!
  FLEXPWM4_SM2VAL1 = CurrentCycles - 1; //Cycles für Periodendauer setzen!
  FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_LDOK(Maske1);

  FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_CLDOK(Maske2);
  FLEXPWM4_SM0CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_PRSC(Prescaler); //Erst nach vollem Cycle updaten und Prescaler setzen!
  FLEXPWM4_SM0VAL1 = CurrentCycles - 1; //Cycles für Periodendauer setzen!
  FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_LDOK(Maske2);

  FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_CLDOK(Maske3);
  FLEXPWM4_SM1CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_PRSC(Prescaler); //Erst nach vollem Cycle updaten und Prescaler setzen!
  FLEXPWM4_SM1VAL1 = CurrentCycles - 1; //Cycles für Periodendauer setzen!
  FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_LDOK(Maske3);
}


void setOutputPWM_freq(float freq, float deadtime_e_s, float deadtime_m_s) {

  //Setting the frequency
  setOutputFrequency(freq);

  //Calculating the deadtime for the timer
  float deadtime_e = (float)FLEXPWM4_SM2VAL1*deadtime_e_s*freq;
  float deadtime_m = (float)FLEXPWM4_SM2VAL1*deadtime_m_s*freq;

  //Setting up channels A und B (Pins 2 & 3)
  FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_CLDOK(Maske1);
  FLEXPWM4_SM2VAL2 = FLEXPWM4_SM2INIT + (uint32_t)deadtime_e;
  FLEXPWM4_SM2VAL3 = (uint32_t)((FLEXPWM4_SM2VAL1-1))/2;
  FLEXPWM4_SM2VAL4 = FLEXPWM4_SM2VAL3 + (uint32_t)deadtime_e; //PauseOffset anpassen!
  FLEXPWM4_SM2VAL5 = FLEXPWM4_SM2VAL1 - 1; //tOn für Channel B anpassen
  FLEXPWM4_OUTEN |= FLEXPWM_OUTEN_PWMA_EN(Maske1); //ChannelA aktivieren
  FLEXPWM4_OUTEN |= FLEXPWM_OUTEN_PWMB_EN(Maske1); //ChannelB aktivieren
  FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_LDOK(Maske1);
  *(portConfigRegister(ChannelA)) = 1;//Maske;
  *(portConfigRegister(ChannelB)) = 1;//Maske;

  //Setting up channel C (Pin 22)
  FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_CLDOK(Maske2);
  FLEXPWM4_SM0VAL2 = (uint32_t)deadtime_m + (uint32_t)((FLEXPWM4_SM0VAL1-1)/4);
  FLEXPWM4_SM0VAL3 = (uint32_t)(3*(FLEXPWM4_SM0VAL1-1)/4);
  FLEXPWM4_OUTEN |= FLEXPWM_OUTEN_PWMA_EN(Maske2); //ChannelC aktivieren
  FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_LDOK(Maske2);
  *(portConfigRegister(ChannelC)) = 1;//Maske;

  //Setting up channel D (Pin 23)
  FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_CLDOK(Maske3);
  FLEXPWM4_SM1VAL2 = FLEXPWM4_SM0VAL3 + (uint32_t)deadtime_m;
  FLEXPWM4_SM1VAL3 = (uint32_t)((FLEXPWM4_SM0VAL1-1)/4);
  FLEXPWM4_OUTEN |= FLEXPWM_OUTEN_PWMA_EN(Maske3); //ChannelD aktivieren
  FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_LDOK(Maske3);
  *(portConfigRegister(ChannelD)) = 1;//Maske;
  }



void setup() 
{
  float frequency = 100; //set the frequency [Hz]
  float deadtime_electrodes = 10*powf(10, -6); //set the deadtime [s] for the electrodes (pins 2 & 3)
  float deadtime_magnets    = 10*powf(10, -6); //set the deadtime [s] for the magnets (pins 22 & 23)

  setOutputPWM_freq(frequency, deadtime_electrodes, deadtime_magnets);
}

void loop() {
}
