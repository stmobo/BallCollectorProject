#!/usr/bin/env python3
"""
Implements basic remote motor control.
Intended to be run from the robot RPi.
"""
import serial
import time
import re

ptn = re.compile(r'\s*(\-?\d+)\s*[/|,]\s*(\-?\d+)')

with serial.Serial('/dev/serial0', 9600) as ser:
	while True:
		i = input("m> ")
		if i == 'e' or i == 'exit':
			break
		elif i == '':
			ser.write([0xAA, 0x01, 0, 0])
		elif i == 'f' or i == 'w':
			ser.write([0xAA, 0x01, 50, 50])
		elif i == 'b' or i == 's':
			ser.write([0xAA, 0x01, 256-50, 256-50])
		elif i == 'l' or i == 'a':
			ser.write([0xAA, 0x01, 50, 256-50])
		elif i == 'r' or i == 'd':
			ser.write([0xAA, 0x01, 256-50, 50])
		else:
			m = ptn.match(i)

			m1 = int(m.group(1))
			m2 = int(m.group(2))

			if m1 < 0:
				m1 = 256 + m1

			if m2 < 0:
				m2 = 256 + m2

			print("{:x} / {:x}".format(m1, m2))

			ser.write([0xAA, 0x01, m1, m2])
			ser.flush()
