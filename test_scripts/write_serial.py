#!/usr/bin/env python3
"""
Constantly sends 0x41 (ASCII 'A') out on /dev/serial0.
Intended for testing the serial ports on the RPi.
"""
import serial
import time

with serial.Serial('/dev/serial0', 9600) as ser:
	while True:
		ser.write([0x41])
		time.sleep(0.5)
