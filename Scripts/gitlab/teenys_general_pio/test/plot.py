import pandas as pd
import matplotlib.pyplot as plt

# CSV-Datei einlesen
df = pd.read_csv("/Users/matteo/Git/code-repository/teenys_general_pio/test/datalog_converted.csv")

# Plot erstellen
plt.figure(figsize=(12, 6))

# Spannung (Voltage) plotten
plt.plot(df["time_s"], df["voltage"], label="Voltage (V)", color="blue")

# Strom (Current) plotten
plt.plot(df["time_s"], df["current_ma"], label="Current (mA)", color="red")

# Achsen beschriften und Titel
plt.xlabel("Time (s)")
plt.ylabel("Value")
plt.title("Voltage and Current over Time")
plt.legend()
plt.grid(True)
plt.tight_layout()

# Plot anzeigen
plt.show()
