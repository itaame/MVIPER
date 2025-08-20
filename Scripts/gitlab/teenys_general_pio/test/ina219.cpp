#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

void setup() {
  // Starte die serielle Kommunikation für die Ausgabe
  Serial.begin(115200);
  // Warte bis zu 4 Sekunden auf eine Verbindung
  while (!Serial && millis() < 4000) {}

  Serial.println("--- INA219 Sensor Test ---");

  // Initialisiere den Sensor
  if (!ina219.begin()) {
    Serial.println("Fehler: INA219 nicht gefunden. Verkabelung pruefen!");
    while (1) { delay(10); } // Stoppe das Programm
  }

  Serial.println("INA219 gefunden. Messung startet...");
  Serial.println("------------------------------------");
}

void loop() {
  float shuntVoltage = 0;
  float busVoltage = 0;
  float current_mA = 0;
  float power_mW = 0;

  // Lese die Werte vom Sensor
  shuntVoltage = ina219.getShuntVoltage_mV();
  busVoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();

  // Gib die Werte formatiert im Seriellen Monitor aus
  Serial.print("Bus-Spannung:  ");
  Serial.print(busVoltage);
  Serial.println(" V");

  Serial.print("Shunt-Spannung: ");
  Serial.print(shuntVoltage);
  Serial.println(" mV");

  Serial.print("Strom:         ");
  Serial.print(current_mA);
  Serial.println(" mA");

  Serial.print("Leistung:      ");
  Serial.print(power_mW);
  Serial.println(" mW");

  Serial.println("------------------------------------");

  // Warte eine Sekunde bis zur nächsten Messung
  delay(1000);
}