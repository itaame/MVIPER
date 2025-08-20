//TeenGen.h
#include <imxrt.h>
#include <Arduino.h>

class TeenGen
{
    private:
        const int pinS1 = 29;
        const int pinS2 = 30;
        const int pinS3 = 31;
        const int pinS4 = 32;
        const int pinBI = 27;
        const int pinBD = 26;
        const int pinInputEN = 37;
        const int pinInputBI = 38;
        const int pinRot1 = 36;
        const int pinRot2 = 35;
        const int pinRot3 = 34;
        const int pinRot4 = 33;

        elapsedMillis ms_since_BI_start;
        elapsedMillis ms_since_BI_stop;
        int trackBIstart;
        int trackBIstop;
 
        int s1;
        int s2;
        int s3;
        int s4;
        
        int time;
        

    public:
        TeenGen();
        int intervalEnableMHD;
        volatile int enableBI;
        volatile int enableMHD;
        int stateBI;
        int signalOut;
        unsigned int rot1;
        unsigned int rot2;
        unsigned int rot3;
        unsigned int rot4;
        void setMHDmode();
        void setBImode();
        void readEnableMHD();
};