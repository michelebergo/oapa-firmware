#!/usr/bin/env python3
"""Opens the OAPA serial port WITHOUT resetting the board and sends "?" like
N.I.N.A. polls, so the firmware sees an active N.I.N.A. session.

    python tools/serial_poke.py /dev/ttyUSB0 --seconds 3
"""
import argparse
import time

import serial


def main():
    p = argparse.ArgumentParser()
    p.add_argument("port")
    p.add_argument("--seconds", type=float, default=3)
    args = p.parse_args()
    port = serial.Serial()
    port.port, port.baudrate, port.timeout = args.port, 115200, 1
    port.dtr = False
    port.rts = False  # CH340 auto-reset lines stay released
    port.open()
    end = time.time() + args.seconds
    while time.time() < end:
        port.write(b"?\n")
        port.readline()
        port.readline()
        time.sleep(0.3)
    port.close()


if __name__ == "__main__":
    main()
