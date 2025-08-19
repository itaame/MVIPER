import argparse
import serial


def main(port: str, baudrate: int) -> None:
    """Read lines from the serial port and print them."""
    with serial.Serial(port, baudrate, timeout=1) as ser:
        try:
            while True:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    print(line)
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Receive and print sensor data from serial port")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port to open")
    parser.add_argument("--baudrate", type=int, default=115200, help="Baud rate for the serial connection")
    args = parser.parse_args()
    main(args.port, args.baudrate)
