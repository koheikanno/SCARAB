#!/usr/bin/env python

# Stream Chasers Aircraft Research and Build
# Copyright TBD

# CSV Logger

import csv
from datetime import datetime

class CSVWriter:
	def write_row(self, telemetry):
		row = [str(datetime.time(datetime.now())), telemetry.airspeed, \
		telemetry.groundspeed, telemetry.heading, telemetry.throttle, \
		telemetry.altitude, telemetry.climb]
		self.writer.writerow(row)
	def payload_drop(self, payload, time, altitude):
		row = [payload, time, altitude]
		self.writer.writerow(row)

	def btn_clicked(self):
		if self.writer_on == False:
			# file path should not be a constant
			self.file_name = "/home/kohei/scarablogs/%s.csv" %datetime.now().strftime("%y%m%d_%H%M%S")
			self.f = open(self.file_name, 'w')
			self.writer = csv.writer(self.f)
			self.writer_on = True
			return True
		else:
			self.f.close()
			self.writer_on = False
			return False

	def __init__(self):
		self.writer_on = False
