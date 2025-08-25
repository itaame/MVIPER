"""Simple serial receiver that forwards incoming lines as UDP packets.

This version is tailored for the Teensy firmware that streams comma-separated
sensor values at 10 kHz.  Each line contains the following fields:

```
timestamp_general, hydrogenVoltage, accelX, accelY, accelZ, gyroX, gyroY,
gyroZ, boxTemp, boxPres, boxHum, coilTemp0, coilTemp1, coilTemp2, coilTemp3,
currentE1, voltageE1, currentE2, voltageE2, currentE3, voltageE3, currentE4,
voltageE4, currentM1, currentM2, currentM3, currentM4, timestamp_current
```

The script decodes each line into floats, packages them with a CCSDS-like
header, and sends the result via UDP.
"""

import argparse
import socket
import struct
import time
from typing import List, Optional


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


TM_SEND_ADDRESS = "127.0.0.1"
TM_SEND_PORT = 10015


def header(seq_count: int, apid: int, data_len: int) -> bytes:
    if seq_count >= 16382:
        seq_count = 0
    ccsds_len = data_len + 5
    return (
        apid.to_bytes(2, "big")
        + (49152 + seq_count).to_bytes(2, "big")
        + ccsds_len.to_bytes(2, "big")
    )


def floats_to_be(*values: float) -> bytes:
    return b"".join(struct.pack(">f", v) for v in values)


def main(port: str, baudrate: int) -> None:
    try:
        import serial
    except ModuleNotFoundError as exc:  # pragma: no cover - import is small
        raise SystemExit(
            "PySerial is required to run this script. Install it with 'pip install pyserial'."
        ) from exc

    tm_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    seq = 0
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
                        continue

                    parts = payload.split(",")
                    if len(parts) != len(FIELDS):
                        continue

                    try:
                        numbers = [float(p) for p in parts]
                    except ValueError:
                        continue

                    padded = numbers + [0.0] * (30 - len(numbers))
                    data_bytes = floats_to_be(*padded)
                    pkt = header(seq, apid=0x64, data_len=len(data_bytes)) + data_bytes
                    tm_socket.sendto(pkt, (TM_SEND_ADDRESS, TM_SEND_PORT))
                    seq += 1
            except KeyboardInterrupt:
                pass
    except serial.SerialException as exc:
        raise SystemExit(f"Failed to open serial port {port}: {exc}") from exc
    finally:
        tm_socket.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Receive lines from a serial port")
    parser.add_argument("--port", default="COM3", help="Serial port to open (default: COM3)")
    parser.add_argument(
        "--baudrate", type=int, default=5_000_000, help="Baud rate for the serial connection"
    )
    args = parser.parse_args()
    main(args.port, args.baudrate)

