#include <Arduino.h>
#include <IntervalTimer.h>

// Counter used as example payload
volatile uint32_t counter = 0;
IntervalTimer timer;

// Interrupt routine that fires at 10 kHz (every 100 microseconds)
void sendData() {
  Serial.println(counter++);
}

void setup() {
  Serial.begin(5'000'000);  // High baud rate for reliable 10 kHz transfer
  while (!Serial) {
    // Wait for the serial port to open
  }

  // Start the periodic interrupt at 100 microseconds
  timer.begin(sendData, 100);
}

void loop() {
  // Main loop intentionally left empty.
  // Data transmission is handled in the sendData interrupt.
}

