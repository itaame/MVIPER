#include <Wire.h>
#include <SD.h>
#include <Adafruit_INA219.h>

// --- KONFIGURATION ---
// Ändere diesen Wert, um die Schreibfrequenz zu testen.
// 1000 = 1 kHz (1000 mal pro Sekunde)
// 5000 = 5 kHz
// 10000 = 10 kHz
const int TEST_FREQUENZ_HZ = 1000; 

// --- GLOBALE OBJEKTE ---
Adafruit_INA219 ina219;
File dataFile;
IntervalTimer stressTestTimer;

// Berechne das Intervall in Mikrosekunden für den Timer
const int INTERVALL_US = 1000000 / TEST_FREQUENZ_HZ;

// Zähler für die geschriebenen Zeilen
volatile unsigned long lineCounter = 0;


// =========================================================================
//  Interrupt Service Routine (ISR)
//  ACHTUNG: Führt langsame SD-Karten-Operationen aus! Nur für Testzwecke!
// =========================================================================
void writeDataISR() {
  // 1. Daten vom Sensor lesen (sehr schnell)
  float spannung = ina219.getBusVoltage_V();
  float strom = ina219.getCurrent_mA();

  // 2. Daten direkt auf die SD-Karte schreiben (langsam und riskant in einer ISR)
  if (dataFile) {
    dataFile.print(micros()); // Zeitstempel mit Mikrosekunden für genaue Analyse
    dataFile.print(",");
    dataFile.print(spannung, 4); // Mit 4 Nachkommastellen
    dataFile.print(",");
    dataFile.println(strom, 4);
    lineCounter++;
  }
}


// =========================================================================
//  SETUP - Wird einmal beim Start ausgeführt
// =========================================================================
void setup() {
  Serial.begin(115200);
  // Warte bis zu 4 Sekunden auf eine serielle Verbindung
  while (!Serial && millis() < 4000) {}

  Serial.println("--- Teensy SD-Card Write Speed Test ---");
  Serial.print("Test-Frequenz: ");
  Serial.print(TEST_FREQUENZ_HZ);
  Serial.println(" Hz");
  Serial.print("Timer-Intervall: ");
  Serial.print(INTERVALL_US);
  Serial.println(" Mikrosekunden");
  
  // I2C-Bus starten
  Wire.begin();

  // INA219-Sensor initialisieren
  if (!ina219.begin()) {
    Serial.println("Fehler: INA219 nicht gefunden!");
    while (1); // Anhalten
  }
  Serial.println("INA219 gefunden.");

  // SD-Karte initialisieren
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("Fehler: SD-Karte nicht gefunden!");
    while (1); // Anhalten
  }
  Serial.println("SD-Karte initialisiert.");

  // CSV-Datei öffnen und Header schreiben
  String filename = "freq_test_" + String(TEST_FREQUENZ_HZ) + "hz.csv";
  dataFile = SD.open(filename.c_str(), FILE_WRITE);
  
  if (dataFile) {
    Serial.print("Schreibe in Datei: ");
    Serial.println(filename);
    dataFile.println("Timestamp_us,Spannung_V,Strom_mA");
    dataFile.flush(); // Header sofort schreiben
  } else {
    Serial.println("Fehler beim Oeffnen der Datei!");
    while (1); // Anhalten
  }

  // Starte den Interrupt-Timer
  stressTestTimer.begin(writeDataISR, INTERVALL_US);
}


// =========================================================================
//  LOOP - Läuft kontinuierlich nach dem Setup
// =========================================================================
void loop() {
  // Gib alle 2 Sekunden einen Statusbericht aus
  static unsigned long lastReportTime = 0;
  if (millis() - lastReportTime > 2000) {
    lastReportTime = millis();
    
    unsigned long count = lineCounter; // Kopiere den Wert, da er sich ändern kann
    Serial.print("Status: ");
    Serial.print(count);
    Serial.println(" Zeilen geschrieben. System laeuft.");
    
    // WICHTIG: Regelmäßiges Flushen der Daten aus dem Puffer auf die Karte
    // Dies ist entscheidend, damit bei einem Absturz nicht zu viele Daten verloren gehen.
    if(dataFile) {
      dataFile.flush();
    }
  }
}