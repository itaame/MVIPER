//MQ8.cpp
#include "MQ8.h"
#include <imxrt.h>
#include <Arduino.h>

MQ8::MQ8()
{
    int analogPin = 13;
    int voltage;
    int offset = 0;
    analogReadResolution(16);
}

int MQ8::readSensor()
{
    voltage = analogRead(analogPin) - offset;
    if (voltage < 0)
    {
        voltage = 0;
    }
    return voltage;
    
}

void MQ8::setZero()
{
    int temp = 0;
    offset = 0;
    for(int i = 0; i < 10; i++)
    {
        temp = temp + readSensor();
    }
    offset = temp / 10;
}