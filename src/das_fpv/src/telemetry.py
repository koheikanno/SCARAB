#!/usr/bin/env python

# Stream Chasers Aircraft Research and Build
# Copyright TBD

# TODO
# ^C doesn't kill process. ^D only kills the process but the window remains. ^\ finally kills it.

import rospy, threading
from mavros_msgs.msg import VFR_HUD, RadioStatus
from sensor_msgs.msg import BatteryState
from mavros_msgs.srv import CommandLong

from ui import MainWindow

class FCU:
	def callback_vfr_hud(self, data):
		telemetry = VFR_HUD()
		if self.init_set == False:
			self.initial_altitude = data.altitude
			self.init_set = True
		# Convert to freedom units
		feet = 3.28084 # ft/m
		g = 32.174 # ft/s2
		try:
			telemetry.airspeed = data.airspeed * feet # ft/s
			telemetry.groundspeed = data.groundspeed * feet # ft/s
			telemetry.heading = data.heading # deg
			telemetry.throttle = data.throttle * 100 # percent
			telemetry.altitude = (data.altitude - self.initial_altitude) * feet # ft
			telemetry.climb = data.climb * feet # ft/s
			if telemetry.altitude > 0:
				landing_time = (2 * telemetry.altitude / g)**0.5 # s
				landing_length = telemetry.groundspeed * landing_time # ft
			else:
				landing_length = 0
			self.main_window.set_telemetry(telemetry, landing_length)
			try:
				if self.main_window.logger.writer_on == True:
					self.main_window.logger.write_row(telemetry)
			except rospy.ROSInterruptException as e:
				rospy.loggerr(e)
				pass
			try:
				main_window.video_window.update_loc(landing_length, 0, telemetry.altitude)
			except rospy.ROSInterruptException as e:
				rospy.loggerr(e)
				pass
		except rospy.ROSInterruptException as e:
			rospy.loggerr(e)
			pass

	def callback_radiostatus(self, data):
		try:
			self.main_window.set_radio_status(data.rssi_dbm)
		except rospy.ROSInterruptException as e:
			rospy.loggerr(e)
			pass

	def callback_batterystate(self, data):
		try:
			self.main_window.set_battery_state(data.percentage * 100)
		except rospy.ROSInterruptException as e:
			rospy.logerr(e)
			pass

	def listener(self):
		rospy.init_node('listener', anonymous=True, log_level=rospy.DEBUG)
		rospy.Subscriber("/mavros/vfr_hud", VFR_HUD, self.callback_vfr_hud)
		rospy.Subscriber("/mavros/radio_status", RadioStatus, self.callback_radiostatus)
		rospy.Subscriber("/mavros/battery", BatteryState, self.callback_batterystate)
		for i in range(0, 10):
			self.main_window.pub.publish(self.main_window.msg)
		rospy.spin()

	def __init__(self, main_window):
		self.initial_altitude = 0
		self.init_set = False
		self.main_window = main_window
		self.listener()

if __name__ == "__main__":
	gui_ready = threading.Event()
	main_window = MainWindow(gui_ready)
	gui_thread = threading.Thread(target=main_window.run_gui_thread)
	gui_thread.start()
	gui_ready.wait()
	fcu = FCU(main_window)
