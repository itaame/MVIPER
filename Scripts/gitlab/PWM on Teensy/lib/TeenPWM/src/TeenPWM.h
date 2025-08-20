//pwm.h
#include <Arduino.h>
#include <imxrt.h>
#include <IntervalTimerEx.h>


class TeenPWM
{
    private:     
        

        float freq_array[13] = {10, 20, 40, 50, 100, 200, 300, 400, 500, 600, 700, 900, 1000};
        
        float deadtime_electrodes; //set the deadtime [s] for the electrodes (pins 2 & 3)
        float deadtime_magnets; //set the deadtime [s] for the magnets (pins 22 & 23)

        int In1;  // Complementary PWM signals for the electrodes are output on pins 6 & 9
        int In2;
        int In3;  // Complementary PWM signals for the magnets are output on pins 36 & 37
        int In4;
        int EnA;  // Enable Signal for Module A (In1 & In2) is output on pin 4
        int EnB;   // Enable Signal for Module B (In3 & In4) is output on pin 5
        int MaskA; //Masks are used for addressing the correct FlexPWM submodules
        int MaskB;
        int MaskEnA;
        int MaskEnB;
        int ResolutionPWM; //12-Bit resolution for PWM
        
    public:
        TeenPWM();
        const int s1_pin = 29;
        const int s2_pin = 30;
        const int s3_pin = 31;
        const int s4_pin = 32;
        int intervalRead;
        float PWMfreq;
        volatile int s;
        volatile int s1;
        volatile int s2;
        volatile int s3;
        volatile int s4;
        void setOutputFrequency(float frequency);
        void setOutputPWM_freq(float freq, float deadtime_e_s, float deadtime_m_s);
        void pwmOn();
        void pwmOff();
        void pwmDC();
        void standardDC();
        void set_Mode();
};
