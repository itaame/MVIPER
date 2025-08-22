import argparse
import struct
import time

import serial


PACKET = struct.Struct("<i9f")


def main(port: str, baudrate: int) -> None:
    """Decode SensorPacket frames from the serial port."""
    with serial.Serial(port, baudrate, timeout=1) as ser:
        last_print = time.perf_counter()
        frames = 0
        try:
            while True:
                raw = ser.read(PACKET.size)
                if len(raw) != PACKET.size:
                    continue
                frames += 1
                now = time.perf_counter()
                if now - last_print >= 1.0:
                    freq = frames / (now - last_print)
                    print(f"RX freq: {freq:.2f} Hz")
                    frames = 0
                    last_print = now

                (
                    timestamp,
                    accel_x,
                    accel_y,
                    accel_z,
                    gyro_x,
                    gyro_y,
                    gyro_z,
                    box_temp,
                    current_e1,
                    hydrogen,
                ) = PACKET.unpack(raw)
                print(
                    f"ts={timestamp} accel=({accel_x:.2f},{accel_y:.2f},{accel_z:.2f}) "
                    f"gyro=({gyro_x:.2f},{gyro_y:.2f},{gyro_z:.2f}) "
                    f"boxTemp={box_temp:.2f} currentE1={current_e1:.2f} hydrogen={hydrogen:.2f}"
                )
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Receive and decode sensor data from serial port",
    )
    parser.add_argument(
        "--port", default="COM3", help="Serial port to open (default: COM3)",
    )
    parser.add_argument(
        "--baudrate", type=int, default=5_000_000,
        help="Baud rate for the serial connection",
    )
    args = parser.parse_args()
    main(args.port, args.baudrate)

