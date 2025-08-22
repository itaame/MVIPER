"""Simple serial receiver that prints incoming lines and their frequency."""

import argparse
import time

import serial


def main(port: str, baudrate: int) -> None:
    with serial.Serial(port, baudrate, timeout=1) as ser:
        last_time = None
        try:
            while True:
                line = ser.readline()
                if not line:
                    continue
                now = time.perf_counter()
                if last_time is None:
                    freq = float("inf")
                else:
                    freq = 1.0 / (now - last_time)
                last_time = now

                try:
                    payload = line.decode().strip()
                except UnicodeDecodeError:
                    continue

                print(f"{payload} @ {freq:.1f} Hz")
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Receive lines from a serial port")
    parser.add_argument("--port", default="COM3", help="Serial port to open (default: COM3)")
    parser.add_argument(
        "--baudrate", type=int, default=5_000_000, help="Baud rate for the serial connection"
    )
    args = parser.parse_args()
    main(args.port, args.baudrate)

