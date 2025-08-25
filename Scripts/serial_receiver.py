"""Simple serial receiver that prints incoming lines and their frequency.

This version is tailored for the Teensy firmware that streams comma-separated
sensor values at 10 kHz.  Each line contains the following fields:

```
timestamp_general, hydrogenVoltage, accelX, accelY, accelZ, gyroX, gyroY,
gyroZ, boxTemp, boxPres, boxHum, coilTemp0, coilTemp1, coilTemp2, coilTemp3,
currentE1, voltageE1, currentE2, voltageE2, currentE3, voltageE3, currentE4,
voltageE4, currentM1, currentM2, currentM3, currentM4, timestamp_current
```

The script decodes the line into a dictionary and prints it alongside the
observed line frequency.
"""

import argparse
import time
from typing import Dict, List, Optional


# Order of fields in the comma-separated payload emitted by the firmware
FIELDS: List[str] = [
    "timestamp_general",
    "hydrogenVoltage",
    "accelX",
    "accelY",
    "accelZ",
    "gyroX",
    "gyroY",
    "gyroZ",
    "boxTemp",
    "boxPres",
    "boxHum",
    "coilTemp0",
    "coilTemp1",
    "coilTemp2",
    "coilTemp3",
    "currentE1",
    "voltageE1",
    "currentE2",
    "voltageE2",
    "currentE3",
    "voltageE3",
    "currentE4",
    "voltageE4",
    "currentM1",
    "currentM2",
    "currentM3",
    "currentM4",
    "timestamp_current",
]


def main(port: str, baudrate: int) -> None:
    try:
        import serial
    except ModuleNotFoundError as exc:  # pragma: no cover - import is small
        raise SystemExit(
            "PySerial is required to run this script. Install it with 'pip install pyserial'."
        ) from exc

    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            last_time: Optional[float] = None
            try:
                while True:
                    line = ser.readline()
                    if not line:
                        continue
                    now = time.perf_counter()
                    if last_time is None:
                        freq = float("inf")
                    else:
                        delta = now - last_time
                        freq = float("inf") if delta == 0 else 1.0 / delta
                    last_time = now

                    try:
                        payload = line.decode().strip()
                    except UnicodeDecodeError:
                        # Skip frames with invalid encoding
                        continue

                    parts = payload.split(",")
                    if len(parts) != len(FIELDS):
                        # Ignore incomplete frames
                        continue

                    try:
                        numbers = [float(p) for p in parts]
                    except ValueError:
                        # Skip frames with invalid numeric data
                        continue

                    data: Dict[str, float] = dict(zip(FIELDS, numbers))
                    print(f"{data} @ {freq:.1f} Hz")
            except KeyboardInterrupt:
                pass
    except serial.SerialException as exc:
        raise SystemExit(f"Failed to open serial port {port}: {exc}") from exc


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Receive lines from a serial port")
    parser.add_argument("--port", default="COM3", help="Serial port to open (default: COM3)")
    parser.add_argument(
        "--baudrate", type=int, default=5_000_000, help="Baud rate for the serial connection"
    )
    args = parser.parse_args()
    main(args.port, args.baudrate)

