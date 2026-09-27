#!/usr/bin/env python3
"""Opens the OAPA serial port like the plugin does and prints the boot banner
plus the reply to "?". Used to prove new firmware changes nothing on the wire.

    python tools/serial_probe.py COM5
"""
import sys
import time

import serial


def main():
    port = serial.Serial(sys.argv[1], 115200, timeout=0.5)
    time.sleep(2.5)  # CH340 resets the ESP32 when the port opens
    banner = port.read(4096).decode(errors="replace")
    port.write(b"?\n")
    reply = [port.readline().decode(errors="replace") for _ in range(2)]
    port.close()
    print("--- banner ---")
    print(banner, end="")
    print("--- ? reply ---")
    print("".join(reply), end="")


if __name__ == "__main__":
    main()
