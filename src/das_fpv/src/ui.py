#!/usr/bin/env python

# Stream Chasers Aircraft Research and Build
# Copyright TBD

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import GObject, Gtk

# Main window class initializes and the class methods that control the features
class MainWindow:
	def run_gui_thread(self):
		GObject.threads_init()
		self.gui_ready.set()
		Gtk.main()
	
	def on_main_window_destroy(self, object, data=None):
		Gtk.main_quit()
		print(2)
	def on_main_window_delete_event(self, object, data=None):
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
	def on_btn_video_window_clicked(self, btn_video_window_clicked):
		if self.btn_video_window.get_label() == "Open FPV Window":
			global video_window
			video_window = Video()
			video_thread = threading.Thread(target=video_window.overlay)
			video_thread.start()

			self.btn_video_window.set_label("Close FPV Window")
		else:
			cv2.destroyAllWindows()
			video_window.video_capture.release()
			self.btn_video_window.set_label("Open FPV Window")

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
		except rospy.ROSInterruptException as e:
			rospy.loggerr(e)
			pass
	def set_radio_status(self, rssi):
		try:
			self.lbl_rssi.set_text("%d dBm" %rssi)
		except rospy.ROSInterruptException as e:
			rospy.loggerr(e)
			pass
	def __init__(self, gui_ready):
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
		self.btn_video_window = self.builder.get_object("btn_video_window")
		self.btn_video_window.set_label("Open FPV Window")
		
		self.reset_dialog = self.builder.get_object("reset_dialog")
		self.reset_dialog.set_transient_for(self.window)
		self.btn_reset_dialog_cncl = self.builder.get_object("btn_reset_dialog_cncl")
		self.btn_reset_dialog_ok = self.builder.get_object("btn_reset_dialog_ok")
		
		self.payload_dialog = self.builder.get_object("payload_dialog")
		self.payload_dialog.set_transient_for(self.window)
		self.lbl_payload_dialog = self.builder.get_object("lbl_payload_dialog")
		self.btn_payload_dialog_cncl = self.builder.get_object("btn_payload_dialog_cncl")
		self.btn_payload_dialog_ok = self.builder.get_object("lbl_payload_dialog_ok")
		self.gui_ready = gui_ready