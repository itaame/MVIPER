import argparse
import serial


def main(port: str, baudrate: int, frame_size: int) -> None:
    """Read fixed-size frames from the serial port and print them."""
    with serial.Serial(port, baudrate) as ser:
        buffer = bytearray(frame_size)
        try:
            while True:
                bytes_read = ser.readinto(buffer)
                if bytes_read:
                    print(buffer[:bytes_read].decode("utf-8", errors="ignore"))
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Receive and print sensor data from serial port"
    )
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port to open")
    parser.add_argument(
        "--baudrate", type=int, default=5000000, help="Baud rate for the serial connection"
    )
    parser.add_argument(
        "--frame-size", type=int, default=64, help="Number of bytes to read per frame"
    )
    args = parser.parse_args()
    main(args.port, args.baudrate, args.frame_size)
