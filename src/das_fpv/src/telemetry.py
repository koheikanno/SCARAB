#!/usr/bin/env python

# Copyright - Stream Chasers Aircraft Research and Build
# Idk what this will become v0.3
# 1/23/2018
# Kohei Kanno, Andy Lee, Alex Lui, Neboneed Farhadi, Linwei Zhuo

# Prerequisites
# ROS, MAVROS, GTK, OpenCV2. UI designed with Glade. Tested on Ubuntu 17.10.

# TODO
# A lot of stuff here and there...
# ^C doesn't kill process. ^D only kills the process but the window remains. ^\ finally
# Implement air resistance to the calculation, lateral length
# Maybe more telemetry info to show?
# Brush up UI
# Put all MAVROS stuff into a class

import rospy, threading, csv
from datetime import datetime
from mavros_msgs.msg import VFR_HUD, ActuatorControl, RadioStatus
from mavros_msgs.srv import CommandLong
import cv2

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import GObject, Gtk

# Apparently this thread for GTK must run first, and the Gtk.main() should be run within the same thread
gui_ready = threading.Event()
def run_gui_thread():
	GObject.threads_init()
	gui_ready.set()
	Gtk.main()
	print(1)

class Video:
	def update_loc(self, x, y, alt):
		# ft -> pixel to be implemented here
		# Assumptions: 4:3 640x480, 1 ft altitude -> 1 ft/480 px
		alt_pix = 1 / 480 # ft/px per foot of altitude
		self.x = self.cam_height / 2 + x / alt_pix / alt
		self.y = self.cam_width / 2 + y / alt_pix / alt
	def overlay(self):
		while True:
			ret, frame = self.video_capture.read()
			roi = frame[(y - self.crosshair_height/2):(y + self.crosshair_height/2),(x - self.crosshair_width/2):(x + self.crosshair_width/2)]
			roi_bg = cv2.bitwise_and(roi, roi, mask = self.mask_inv)
			roi_fg = cv2.bitwise_and(self.crosshair, self.crosshair, mask = self.mask)
			dst = cv2.add(roi_bg, roi_fg)
			frame[(y - self.crosshair_height/2):(y + self.crosshair_height/2),(x - self.crosshair_width/2):(x + self.crosshair_width/2)] = dst
			cv2.imshow('Video', frame)
			k = cv2.waitKey(1) & 0xFF
			if k == 27:
				cv2.destroyAllWindows()

	def __init__(self):
		self.img_crosshair = cv2.imread('/home/kohei/crosshair.png', -1)
		self.orig_mask = self.img_crosshair[:,:,3]
		self.orig_mask_inv = cv2.bitwise_not(self.orig_mask)
		self.img_crosshair = self.img_crosshair[:,:,0:3]
		self.video_capture = cv2.VideoCapture(0)
		ret, frame = self.video_capture.read()
		self.cam_height, self.cam_width = frame.shape[:2]
		self.crosshair_width = 128
		self.crosshair_height = 128
		self.crosshair = cv2.resize(self.img_crosshair, (self.crosshair_width, self.crosshair_height), interpolation = cv2.INTER_AREA)
		self.mask = cv2.resize(self.orig_mask, (self.crosshair_width, self.crosshair_height), interpolation = cv2.INTER_AREA)
		self.mask_inv = cv2.resize(self.orig_mask_inv, (self.crosshair_width, self.crosshair_height), interpolation = cv2.INTER_AREA)
		cv2.namedWindow('Video')
		self.x = 0
		self.y = 0

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

# Main window class initializes and the class methods that control the features
class MainWindow:
	def on_main_window_destroy(self, object, data=None):
		Gtk.main_quit()
		print(2)
	def on_main_window_delete_event(self):
		Gtk.main_quit()
		print(3)
	def on_reset_dialog_delete_event(self):
		self.reset_dialog.hide()
	def on_reset_dialog_destroy(self, object, data=None):
		self.reset_dialog.hide()
	def on_payload_dialog_delete_event(self):
		self.payload_dialog.hide()
	def on_payload_dialog_destroy(self, object, data=None):
		self.payload_dialog.hide()
	def on_btn_reset_dialog_cncl_clicked(self, btn_reset_dialog_cncl):
		self.reset_dialog.hide()
	def on_btn_reset_dialog_ok_clicked(self, btn_reset_dialog_ok):
		rospy.wait_for_service('/mavros/cmd/command')
		reset_srv = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
		reset_srv(0, 246, 1, 1, 1, 0, 0, 0, 0, 0)
		self.reset_dialog.hide()
	def on_btn_payload_dialog_ok_clicked(self, btn_payload_dialog_ok):
		global logger
		try:
			pub = rospy.Publisher('/mavros/actuator_control', ActuatorControl, queue_size=3)
			msg = ActuatorControl()
			msg.header.frame_id = "payload_servo"
			if self.actuator_payload == 0:
				msg.controls = [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0]
				self.btn_drop_payload0.set_sensitive(False)
				self.payload0_drop_time = str(datetime.time(datetime.now()))
				self.payload0_altitude = self.current_altitude
				logger.payload_drop(0, self.payload0_drop_time, self.payload0_altitude)
				self.lbl_payload0_info.set_text("Drop Time: %s\n Altitude: %s ft" %(self.payload0_drop_time, self.payload0_altitude))
			else:
				msg.controls = [0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0]
				self.btn_drop_payload1.set_sensitive(False)
				self.payload1_drop_time = str(datetime.time(datetime.now()))
				self.payload1_altitude = self.current_altitude
				logger.payload_drop(1, self.payload1_drop_time, self.payload1_altitude)
				self.lbl_payload1_info.set_text("Drop Time: %s\n Altitude: %s ft" %(self.payload1_drop_time, self.payload1_altitude))
			msg.group_mix = 3
			pub.publish()
			self.payload_dialog.hide()
		except rospy.ROSInterruptException as e:
			rospy.loggerr(e)

	def on_btn_record_telemetry_clicked(self, btn_record_telemetry):
		if self.btn_record_telemetry.get_label() == "Record":
			global logger
			logger = CSVWriter()
			if logger.btn_clicked() == True:
				self.btn_record_telemetry.set_label("Stop")
		else:
			if logger.btn_clicked() == False:
				self.btn_record_telemetry.set_label("Record")
	def on_btn_reset_das_clicked(self, btn_reset_das_clicked):
		self.reset_dialog.show()
	def on_btn_drop_payload0_clicked(self, btn_drop_payload0):
		self.lbl_payload_dialog.set_text("Are you sure you want to release Payload #1?")
		self.actuator_payload = 0
		self.payload_dialog.show()
	def on_btn_payload_dialog_cncl_clicked(self, btn_payload_dialog_cncl):
		self.payload_dialog.hide()
	def on_btn_drop_payload1_clicked(self, btn_drop_payload0):
		self.lbl_payload_dialog.set_text("Are you sure you want to release Payload #2?")
		self.actuator_payload = 1
		self.payload_dialog.show()

	def set_telemetry(self, data, landing_length):
		try:
			self.lbl_airspeed.set_text("%d ft/s" %data.airspeed)
			self.lbl_groundspeed.set_text("%d ft/s" %data.groundspeed)
			self.lbl_heading.set_text("%d deg" %data.heading)
			self.lbl_throttle.set_text("%d %%" %data.throttle)
			self.lbl_altitude.set_text("%d ft" %data.altitude)
			self.lbl_climb.set_text("%d ft/s" %data.climb)
			self.lbl_landing_length.set_text("%d ft" %landing_length)
			self.current_altitude = data.altitude
			print(1234)
		except:
			print(34567324567345763457)
			pass
	def set_radio_status(self, rssi):
		try:
			self.lbl_rssi.set_text("%d dbm" %rssi)
		except Exception as e:
			print(e)
			pass
	def __init__(self):
		self.gladefile = "gui.glade"
		self.builder = Gtk.Builder()
		self.builder.add_from_file(self.gladefile)
		self.builder.connect_signals(self)
		self.window = self.builder.get_object("main_window")
		self.window.show()
		self.lbl_airspeed = self.builder.get_object("lbl_airspeed")
		self.lbl_groundspeed = self.builder.get_object("lbl_groundspeed")
		self.lbl_heading = self.builder.get_object("lbl_heading")
		self.lbl_throttle = self.builder.get_object("lbl_throttle")
		self.lbl_altitude = self.builder.get_object("lbl_altitude")
		self.lbl_climb = self.builder.get_object("lbl_climb")
		self.lbl_landing_length = self.builder.get_object("lbl_landing_length")
		self.lbl_rssi = self.builder.get_object("lbl_rssi")
		self.lbl_payload0_info = self.builder.get_object("lbl_payload0_info")
		self.lbl_payload1_info = self.builder.get_object("lbl_payload1_info")
		self.btn_record_telemetry = self.builder.get_object("btn_record_telemetry")
		self.btn_drop_payload0 = self.builder.get_object("btn_drop_payload0")
		self.btn_drop_payload1 = self.builder.get_object("btn_drop_payload1")
		self.btn_reset_das = self.builder.get_object("btn_reset_das")
		self.btn_record_telemetry.set_label("Record")
		
		self.reset_dialog = self.builder.get_object("reset_dialog")
		self.reset_dialog.set_transient_for(self.window)
		self.btn_reset_dialog_cncl = self.builder.get_object("btn_reset_dialog_cncl")
		self.btn_reset_dialog_ok = self.builder.get_object("btn_reset_dialog_ok")
		
		self.payload_dialog = self.builder.get_object("payload_dialog")
		self.payload_dialog.set_transient_for(self.window)
		self.lbl_payload_dialog = self.builder.get_object("lbl_payload_dialog")
		self.btn_payload_dialog_cncl = self.builder.get_object("btn_payload_dialog_cncl")
		self.btn_payload_dialog_ok = self.builder.get_object("lbl_payload_dialog_ok")

# What should I do with this....
global initial_altitude, init_set
initial_altitude = 0
init_set = False

def callback_vfr_hud(data):
	telemetry = VFR_HUD()
	global init_set, main_window, initial_altitude, logger, video_window
	if init_set == False:
		initial_altitude = data.altitude
		init_set = True
	# Convert to freedom units
	feet = 3.28084 # ft/m
	g = 32.174 # ft/s2
	try:
		telemetry.airspeed = data.airspeed * feet # ft/s
		telemetry.groundspeed = data.groundspeed * feet # ft/s
		telemetry.heading = data.heading # deg
		telemetry.throttle = data.throttle * 100 # percent
		telemetry.altitude = (data.altitude - initial_altitude) * feet # ft
		telemetry.climb = data.climb * feet # ft/s
		if telemetry.altitude > 0:
			landing_time = (2 * telemetry.altitude / g)**0.5 # s
			landing_length = telemetry.groundspeed * landing_time # ft
		else:
			landing_length = 0
		main_window.set_telemetry(telemetry, landing_length)
		try:
			if logger.writer_on == True:
				logger.write_row(telemetry)
		except Exception as e:
			print(e)
			pass
		#video_window.update_loc(landing_length, 0, telemetry.altitude)
		print(4321)
	except Exception as e:
		rospy.logerr(e)
		pass

def callback_radiostatus(data):
	try:
		global main_window
		main_window.set_radio_status(data.rssi_dbm)
	except Exception as e:
		print(e)
		pass
	
def listener():
	rospy.init_node('listener', anonymous=True, log_level=rospy.DEBUG)
	rospy.Subscriber("/mavros/vfr_hud", VFR_HUD, callback_vfr_hud)
	rospy.Subscriber("/mavros/radio_status", RadioStatus, callback_radiostatus)
	rospy.spin()
	print(5423)

if __name__ == "__main__":
	main_window = MainWindow()
	gui_thread = threading.Thread(target=run_gui_thread)
	gui_thread.start()
	gui_ready.wait()
	#video_window = Video()
	#video_thread = threading.Thread(target=video_window.overlay())
	#video_thread.start()
	listener()
