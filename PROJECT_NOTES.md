# Project Notes

## Serial Baud Rate

- The telemetry link now uses a baud rate of **5,000,000 bps** for higher throughput.
- Ensure the microcontroller and host machine both use this baud rate (see `Scripts/main.cpp` and `Scripts/serial_receiver.py`).
- Teensy 4.x hardware UART derives from a 24 MHz clock with a minimum oversampling ratio of 4, allowing baud rates up to 6 Mbit/s, so 5 Mbit/s is within spec.
- Use short, well‑shielded wires and verify that USB‑serial adapters and peripherals can reliably handle 5 Mbit/s.
- If errors occur, lower the baud rate or improve wiring quality.
