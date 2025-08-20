// Benötigte Bibliotheken einbinden
#include <Wire.h>
#include <SD.h>
#include <TimeLib.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>

// --- Globale Objekte und Variablen ---

// Sensor-Objekt
Adafruit_LSM6DSOX sox;

// SD-Karten-Datei
File dataFile;

// Hardware-Timer für die Mess-Interrupts
IntervalTimer measurementTimer;

// "volatile" ist entscheidend für Variablen, die in einer ISR und im loop() verwendet werden.
// Diese Variablen speichern die zuletzt gemessenen Werte.
volatile bool newDataAvailable = false;
volatile float accelX, accelY, accelZ;
volatile float gyroX, gyroY, gyroZ;
volatile float temperatureC;
volatile time_t timestamp;


// =========================================================================
//  Interrupt Service Routine (ISR) für die Messungen
//  Wird vom Timer automatisch aufgerufen. Hält sich kurz und schnell.
// =========================================================================
void measureAndLog() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  // Daten vom Sensor abrufen
  sox.getEvent(&accel, &gyro, &temp);

  // Zeitstempel und Messwerte in globalen Variablen ablegen
  timestamp    = now();
  accelX       = accel.acceleration.x;
  accelY       = accel.acceleration.y;
  accelZ       = accel.acceleration.z;
  gyroX        = gyro.gyro.x;
  gyroY        = gyro.gyro.y;
  gyroZ        = gyro.gyro.z;
  temperatureC = temp.temperature;

  // Flag setzen: Dem Haupt-Loop signalisieren, dass neue Daten bereitstehen
  newDataAvailable = true;
}


// =========================================================================
//  SETUP - Wird einmal beim Start ausgeführt
// =========================================================================
void setup() {
  Serial.begin(115200);
  // Optional: Warten, bis eine serielle Verbindung besteht, um keine Startmeldungen zu verpassen
  while (!Serial && millis() < 4000) {}

  Serial.println("--- Teensy IMU Test Start ---");

  // I2C-Bus starten
  Wire.begin();

  // IMU-Sensor initialisieren
  if (!sox.begin_I2C()) {
    Serial.println("Fehler: LSM6DSOX nicht gefunden!");
    while (1) { delay(10); } // Programm anhalten
  }
  Serial.println("LSM6DSOX gefunden und initialisiert.");

  // Optional: Messbereiche einstellen (je nach Bedarf)
  // sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  // sox.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);

  // SD-Karte initialisieren
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("Fehler: SD-Karte nicht gefunden oder initialisierbar!");
    while (1) { delay(10); } // Programm anhalten
  }
  Serial.println("SD-Karte initialisiert.");

  // RTC initialisieren und Zeit setzen
  // Pass die Zeit hier einmalig an oder kommentiere es aus, wenn die Batterie die Zeit hält.
  
  // Teensy3Clock.set(1751383200); // Beispielzeit: 01.07.2025 15:20:00 UTC
  // setSyncProvider(Teensy3Clock.get);

  // CSV-Datei öffnen und den Header (Spaltenüberschriften) schreiben
  dataFile = SD.open("imu_test.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.println("Timestamp,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,TempC");
    dataFile.flush(); // Header sofort auf die Karte schreiben
    Serial.println("CSV-Datei geoeffnet und Header geschrieben.");
  } else {
    Serial.println("Fehler beim Oeffnen der CSV-Datei!");
    while (1) { delay(10); } // Programm anhalten
  }

  // Starte den Interrupt-Timer.
  // Ruft 'measureAndLog' alle 100.000 Mikrosekunden (100ms -> 10Hz) auf.
  // Für schnellere Messungen (z.B. 50Hz), den Wert auf 20000 reduzieren.
  measurementTimer.begin(measureAndLog, 100000);
  Serial.println("Interrupt-Timer gestartet. Datenerfassung laeuft...");
}


// =========================================================================
//  LOOP - Läuft kontinuierlich nach dem Setup
// =========================================================================
void loop() {
  // Prüfen, ob die ISR neue Daten bereitgestellt hat
  if (newDataAvailable) {
    // Flag sofort zurücksetzen, um keine Daten doppelt zu verarbeiten
    newDataAvailable = false;

    // --- Jetzt die langsamen Aufgaben ausführen ---

    // 1. Daten in einen CSV-String formatieren
    String dataString = String(timestamp) + "," +
                        String(accelX) + "," + String(accelY) + "," + String(accelZ) + "," +
                        String(gyroX) + "," + String(gyroY) + "," + String(gyroZ) + "," +
                        String(temperatureC);

    // 2. String auf die SD-Karte schreiben
    if (dataFile) {
      dataFile.println(dataString);
    }

    // 3. String über die serielle USB-Schnittstelle senden
    Serial.println(dataString);
  }
  
  // Die Loop hat kein delay() und kann bei Bedarf andere, nicht-blockierende
  // Aufgaben ausführen (z.B. eine Status-LED blinken lassen).
}