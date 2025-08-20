import numpy as np
import pandas as pd

# Der Dateiname deiner Binärdatei
binary_file = '/Users/matteo/Git/code-repository/teenys_general_pio/test/datalog.bin'
# Der Name der Zieldatei im CSV-Format
output_csv_file = '/Users/matteo/Git/code-repository/teenys_general_pio/test/datalog_converted.csv'

# Definiere die Datenstruktur. Muss exakt der 'struct' in C++ entsprechen!
# 'u4' -> uint32_t (4 bytes, unsigned)
# 'f4' -> float (4 bytes)
dt = np.dtype([
    ('timestamp_us', np.uint32),
    ('voltage', np.float32),
    ('current_ma', np.float32)
])

try:
    # Lese die Binärdatei in ein NumPy-Array
    data = np.fromfile(binary_file, dtype=dt)

    # Konvertiere das NumPy-Array in einen Pandas DataFrame
    df = pd.DataFrame(data)

    # Berechne die Zeit in Sekunden für eine schönere Darstellung
    df['time_s'] = df['timestamp_us'] / 1_000_000.0

    print(f"Erfolgreich {len(df)} Messpunkte aus '{binary_file}' gelesen.")

    # Speichere den DataFrame als CSV-Datei
    df.to_csv(output_csv_file, index=False, float_format='%.6f')

    print(f"Daten erfolgreich in '{output_csv_file}' konvertiert.")

except Exception as e:
    print(f"Ein Fehler ist aufgetreten: {e}")