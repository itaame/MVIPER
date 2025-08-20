#include "TeenGen.h"
#include <imxrt.h>
#include <Arduino.h>
#include <IntervalTimerEx.h>
#include <TimeLib.h>


TeenGen::TeenGen()
{
    signalOut = 0;
    s1 = 0;
    s2 = 0;
    s3 = 0;
    s4 = 0;
    enableMHD = 0;
    enableBI = 0;
    stateBI = 0;
    rot1 = 0;
    rot2 = 0;
    rot3 = 0;
    rot4 = 0;
    ms_since_BI_start = 0;
    ms_since_BI_stop = 0;
    trackBIstart = 0;
    trackBIstop = 0;

    intervalEnableMHD = 10000; //10 KHz

    pinMode(pinS1, OUTPUT);
    pinMode(pinS2, OUTPUT);
    pinMode(pinS3, OUTPUT);
    pinMode(pinS4, OUTPUT);
    pinMode(pinBI, OUTPUT);
    pinMode(pinBD, OUTPUT);
    pinMode(pinInputEN, INPUT_PULLUP);
    pinMode(pinInputBI, INPUT_PULLUP);
    pinMode(pinRot1, INPUT_PULLUP);
    pinMode(pinRot2, INPUT_PULLUP);
    pinMode(pinRot3, INPUT_PULLUP);
    pinMode(pinRot4, INPUT_PULLUP);

    digitalWrite(pinBI, HIGH);
    digitalWrite(pinBD, HIGH);
}
        
void TeenGen::setMHDmode()
{
    enableMHD = ~(digitalReadFast(pinInputEN)) & 0b1;
    if(enableMHD != 1)
    {
        rot1 = 0;
        rot2 = 0;
        rot3 = 0;
        rot4 = 0;
        signalOut = 0;
    }
    else
    {
        rot1 = ~(digitalReadFast(pinRot1)) & 0b1;
        rot2 = ~(digitalReadFast(pinRot2)) & 0b1;
        rot3 = ~(digitalReadFast(pinRot3)) & 0b1;
        rot4 = ~(digitalReadFast(pinRot4)) & 0b1;
        signalOut = (rot1<<3) | (rot2<<2) | (rot3<<1) | rot4;
        
        /*
        s1 = signalOut & (1<<3);
        s2 = signalOut & (1<<2);
        s3 = signalOut & (1<<1);
        s4 = signalOut & (1<<0);
        */
    }
    digitalWriteFast(pinS1, rot1);
    digitalWriteFast(pinS2, rot2);
    digitalWriteFast(pinS3, rot3);
    digitalWriteFast(pinS4, rot4);
    setBImode();
}

void TeenGen::setBImode()
{
    enableBI = ~(digitalReadFast(pinInputBI)) & 0b1; 
    enableBI = enableBI & enableMHD;
    if((enableBI == 1)&&(stateBI == 0))
    {
        if(trackBIstart == 0)
        {
            digitalWriteFast(pinBI, LOW);
            ms_since_BI_start = 0;
            trackBIstart = 1;
        }
        else if (ms_since_BI_start >= 200)
        {
            digitalWriteFast(pinBI, HIGH);
            stateBI = 1;
            trackBIstart = 0;
        }
    }
    else if((enableBI == 0)&&(stateBI == 1))
    {
        if(trackBIstop == 0)
        {
            digitalWriteFast(pinBD, LOW);
            ms_since_BI_stop = 0;
            trackBIstop = 1;
        }
        else if(ms_since_BI_stop >= 200)
        {
            digitalWriteFast(pinBD, HIGH);
            stateBI = 0;
            trackBIstop = 0;
        }
        
        
    }
    

}

void TeenGen::readEnableMHD()
{
    enableMHD = digitalReadFast(pinInputEN);
}