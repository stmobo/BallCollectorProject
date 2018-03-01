#!/usr/bin/env python3
"""
Constantly requests and displays sensor data frames from the Cortex.
Intended to be run from the robot RPi.
"""
import serial
import time

def wait_for_start_byte(ser):
	t = time.time()
	while True:
		start_byte = ser.read(1)
		if start_byte == b'\x55':
			return True

		#print("[{:.3f}] Ignoring byte: {}".format(time.clock(), start_byte))
		time.sleep(0.02)

		if time.time() - t >= 1.5:
			return False


with serial.Serial('/dev/serial0', 9600, timeout=1.5) as ser:
	while True:
		ser.write([0xAA, 0x02])
		ser.flush()
		print("[{:.3f}] Sent command...".format(time.time()))
		time.sleep(0.05)

		if wait_for_start_byte(ser):
			data = bytearray(ser.read(4))

			if len(data) < 4:
				print("[{:.3f}] Command timed out".format(time.time()))
				continue

			e1 = data[0] | (data[1] << 8)
			e2 = data[2] | (data[3] << 8)

			if e1 > 0x7FFF:
				e1 = -(0x10000 - e1)

			if e2 > 0x7FFF:
				e2 = -(0x10000 - e2)

			print("[{:.3f}] Got data -- e1={}, e2={}".format(time.time(), e1, e2))
			time.sleep(0.5)
		else:
			print("[{:.3f}] Command timed out".format(time.time()))
