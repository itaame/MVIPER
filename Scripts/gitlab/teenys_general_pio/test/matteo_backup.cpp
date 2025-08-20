#include <Wire.h>
#include <SD.h>
#include <Adafruit_INA219.h>

// --- KONFIGURATION ---
const uint32_t SAMPLE_FREQUENCY = 500; // Ziel-Abtastrate in Hz
const uint32_t SAMPLE_INTERVAL_US = 1000000 / SAMPLE_FREQUENCY;

// Größe der Puffer. Ein größerer Puffer bedeutet seltenere, aber größere Schreibvorgänge auf die SD-Karte (effizienter).
// Muss ein Kompromiss zwischen RAM-Verbrauch und Schreibeffizienz sein. 512 Einträge sind ein guter Start.
const size_t BUFFER_SIZE = 512;

// Definiert die Struktur für einen einzelnen Messpunkt.
// Wichtig: 'pack(1)' verhindert, dass der Compiler leere Füll-Bytes einfügt, um die Datenstruktur auszurichten.
// Dies garantiert, dass die Größe in C++ und im späteren Python-Skript identisch ist.
#pragma pack(push, 1)
struct LogEntry {
  uint32_t timestamp_us;
  float voltage;
  float current_ma;
};
#pragma pack(pop)

// --- GLOBALE OBJEKTE UND VARIABLEN ---
Adafruit_INA219 ina219;
File dataFile;
IntervalTimer samplingTimer;

// Die beiden Puffer für das Ping-Pong-Verfahren
LogEntry bufferA[BUFFER_SIZE];
LogEntry bufferB[BUFFER_SIZE];

// Zeiger und Flags für die Pufferverwaltung. 'volatile' ist entscheidend!
volatile LogEntry* volatile buffer_to_fill = bufferA;
volatile LogEntry* volatile buffer_to_write = bufferB;
volatile size_t fill_index = 0;
volatile bool write_request = false;
volatile uint32_t overrun_counter = 0;


// =========================================================================
//  Interrupt Service Routine (ISR) - Extrem schnell und schlank!
// =========================================================================
void sampleISR() {
  // Nur Daten sammeln, wenn der Puffer nicht gerade von einem langsamen
  // Schreibvorgang blockiert wird. Verhindert Systemabsturz.
  if (fill_index < BUFFER_SIZE) {
    buffer_to_fill[fill_index].timestamp_us = micros();
    buffer_to_fill[fill_index].voltage = ina219.getBusVoltage_V();
    buffer_to_fill[fill_index].current_ma = ina219.getCurrent_mA();
    fill_index++;
  }

  // Wenn der Puffer voll ist
  if (fill_index == BUFFER_SIZE) {
    // Prüfen, ob der letzte Schreibvorgang noch aktiv ist (Buffer Overrun)
    if (write_request) {
      overrun_counter++;
    }

    // Puffer tauschen (Ping-Pong)
    volatile LogEntry* temp = buffer_to_fill;
    buffer_to_fill = buffer_to_write;
    buffer_to_write = temp;
    
    fill_index = 0;      // Index für den neuen Puffer zurücksetzen
    write_request = true; // Signal an die loop() senden: "Ein Puffer ist bereit!"
  }
}

// =========================================================================
//  SETUP
// =========================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {}
  
  Serial.println("--- High Frequency Datalogger ---");
  Serial.printf("Frequenz: %u Hz, Intervall: %u us\n", SAMPLE_FREQUENCY, SAMPLE_INTERVAL_US);
  Serial.printf("Puffergroesse: %u Eintraege (%u Bytes)\n", BUFFER_SIZE, sizeof(LogEntry) * BUFFER_SIZE);

  Wire.begin();
  if (!ina219.begin()) {
    Serial.println("Fehler: INA219 nicht gefunden!");
    while(1);
  }
  Serial.println("INA219 initialisiert.");

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("Fehler: SD-Karte nicht gefunden!");
    while(1);
  }
  dataFile = SD.open("datalog.bin", FILE_WRITE | O_TRUNC);
  if (!dataFile) {
    Serial.println("Fehler beim Oeffnen der Datei!");
    while(1);
  }
  Serial.println("Schreibe Binärdaten in datalog.bin...");
  
  // Starte den Interrupt-Timer
  samplingTimer.begin(sampleISR, SAMPLE_INTERVAL_US);
}

// =========================================================================
//  LOOP - Kümmert sich nur um das Schreiben der vollen Puffer
// =========================================================================
void loop() {
  // Prüfen, ob die ISR einen Schreibauftrag erteilt hat
  if (write_request) {
    write_request = false; // Flag sofort zurücksetzen

    // Schreibe den gesamten Puffer in einem einzigen, schnellen Befehl
    size_t bytes_to_write = sizeof(LogEntry) * BUFFER_SIZE;
    size_t bytes_written = dataFile.write((const uint8_t*)buffer_to_write, bytes_to_write);

    if (bytes_written != bytes_to_write) {
      Serial.println("!!! Schreibfehler auf SD-Karte !!!");
    }
  }

  // Statusbericht ausgeben, um zu sehen, ob das System noch lebt und ob Overruns passieren
  static uint32_t last_report = 0;
  if (millis() - last_report > 2000) {
    last_report = millis();
    if (overrun_counter > 0) {
      Serial.printf("WARNUNG: %u Puffer-Ueberlaeufe seit dem letzten Bericht!\n", overrun_counter);
      overrun_counter = 0; // Zähler zurücksetzen
    } else {
      Serial.println("System laeuft stabil, keine Ueberlaeufe.");
    }
    dataFile.flush(); // Regelmäßig Puffer auf die Karte zwingen
  }
}