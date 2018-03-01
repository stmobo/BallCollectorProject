#!/usr/bin/env python3
"""
Prints all data coming in on /dev/serial0.
Intended for serial port testing on the RPi.
"""
import serial
import time

with serial.Serial('/dev/serial0', 9600) as ser:
	print("Waiting for serial data...")
	while True:
		data = ser.read()
		#print("{:.3f}: Got data!".format(time.clock()))
		print("{:.3f}: {}".format(time.clock(), str(data)))
