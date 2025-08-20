#include <Arduino.h>
#include <imxrt.h>

///Arduino Sample Code
void setup()
{
  Serial.begin(9600); //Set serial baud rate to 9600 bps
  Serial.println(10,DEC);
}
void loop()
{
int val = 100;
//val=analogRead(13); //Read Gas value from analog 0
Serial.println(val,DEC);//Print the value to serial port
delay(100);
}
