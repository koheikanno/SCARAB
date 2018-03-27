#!/usr/bin/env python

# Stream Chasers Aircraft Research and Build
# Copyright TBD

# FPV/Video Overlay Integration

import cv2, math, time
from datetime import datetime

class Video:
	def update_loc(self, dist, div, alt, speed):
		# Called from callback_vfr_hud. Updates the overlaid landing location.
		# self.x and self.y are location in pixel on the video feed. The arguments x and y are distance in ft 
		# ft -> pixel to be implemented here.
		# Current assumptions: Camera aligned horizontally
		# alt_pix = 1. / 480 # ft/px per foot of altitude
		if alt > 0 and dist > 0:
			phi = math.atan(alt/dist)
			psi = math.atan(div/dist)
		elif alt > 0 and dist < 0:
			phi = math.radians(90)
			psi = 0
		else:
			# if the reported altitude is less than 0, leave the initial location (center of the screen)
			phi = 0
			psi = 0
		phi = phi - math.radians(self.tilt_angle)
		psi = psi - math.radians(self.pan_angle)
		self.x = int(self.resized_width / 2 + psi * self.px_horiz_angle)
		self.y = int(self.resized_height / 2 + phi * self.px_vert_angle)
		if self.x < 0:
			self.x = 0
		if self.x > self.resized_width:
			self.x = self.resized_width
		if self.y < 0:
			self.y = 0
		if self.y > self.resized_height:
			self.y = self.resized_height
		
		self.speed_text = "%d FT/S" % speed
		self.altitude_text  = "%d FT" % alt

	def overlay(self):
		# This while loop runs continuously in a new thread while the video window is open
		while True:
			ret, frame = self.video_capture.read()
			frame = cv2.resize(frame, (self.resized_width, self.resized_height), interpolation = cv2.INTER_AREA)
			frame = cv2.flip(frame, 0)
			# Define the range of interest, if it's outside the screen, crop the crosshair and overlay.
			y1 = self.y - self.crosshair_height / 2
			y2 = self.y + self.crosshair_height / 2
			cropped = False
			if y1 < 0:
				crosshair = self.crosshair[-y1:self.crosshair_height, :]
				mask = self.mask[-y1:self.crosshair_height, :]
				mask_inv = self.mask_inv[-y1:self.crosshair_height, :]
				y1 = 0
				cropped = True
			if y2 > self.resized_height:
				crosshair = self.crosshair[0:self.crosshair_height - (y2 - self.resized_height), :]
				mask = self.mask[0:self.crosshair_height - (y2 - self.resized_height), :]
				mask_inv = self.mask_inv[0:self.crosshair_height - (y2 - self.resized_height), :]
				y2 = self.resized_height
				cropped = True
			x1 = self.x - self.crosshair_width / 2
			x2 = self.x + self.crosshair_width / 2
			if x1 < 0:
				crosshair = self.crosshair[:, -x1:self.crosshair_width]
				mask = self.mask[:, -x1:self.crosshair_width]
				mask_inv = self.mask_inv[:, -x1:self.crosshair_width]
				x1 = 0
				cropped = True
			if x2 > self.resized_width:
				crosshair = self.crosshair[:, 0:self.crosshair_width - (x2 - self.resized_width)]
				mask = self.mask[:, 0:self.crosshair_width - (x2 - self.resized_width)]
				mask_inv = self.mask_inv[:, 0:self.crosshair_width - (x2 - self.resized_width)]
				x2 = self.resized_width
				cropped = True
			roi = frame[y1:y2, x1:x2]
			if cropped:
				roi_bg = cv2.bitwise_and(roi, roi, mask = mask_inv)
				roi_fg = cv2.bitwise_and(crosshair, crosshair, mask = mask)
			else:
				roi_bg = cv2.bitwise_and(roi, roi, mask = self.mask_inv)
				roi_fg = cv2.bitwise_and(self.crosshair, self.crosshair, mask = self.mask)				
			dst = cv2.add(roi_bg, roi_fg)
			frame[y1:y2, x1:x2] = dst
			cv2.putText(frame, datetime.now().strftime("%m/%d/%y %H:%M:%S"), (512, self.resized_height - 32), self.font, 0.7, (0, 255, 0), 1, cv2.LINE_AA)
			cv2.putText(frame, self.speed_text, (128, self.resized_height - 32), self.font, 0.7, (0, 255, 0), 1, cv2.LINE_AA)
			cv2.putText(frame, self.altitude_text, (256, self.resized_height - 32), self.font, 0.7, (0, 255, 0), 1, cv2.LINE_AA)
			if self.show_drop_info == True:
				cv2.putText(frame, self.payload_drop_text, (64, 64), self.font, 1, (0, 255, 0), 2, cv2.LINE_AA)
				if time.time() - self.show_time > 5:
					self.show_drop_info = False
			
			if self.video_writer_on == True:
				self.out.write(frame)
			cv2.imshow('Video', frame)
			k = cv2.waitKey(1) & 0xFF
			if k == 27:
				cv2.destroyAllWindows()

	def set_tilt_angle(self, deg):
		self.tilt_angle = deg
		
	def set_pan_angle(self, deg):
		self.pan_angle = deg
		
	def payload_drop(self, payload, drop_time, alt):
		self.payload_drop_text = "Payload %i dropped at %s from %s ft" % (payload, drop_time, alt)
		self.show_time = time.time()
		self.show_drop_info = True
		
	def start_video_record(self):
		video_file_name = "/home/kohei/scarablogs/%s.avi" %datetime.now().strftime("%y%m%d_%H%M%S")
		self.out = cv2.VideoWriter(video_file_name, self.fourcc, 30., (self.resized_width, self.resized_height)) 
		self.video_writer_on = True
		
	def stop_video_record(self):
		self.video_writer_on = False
		
	def __init__(self, tilt, pan):
		self.img_crosshair = cv2.imread('/home/kohei/scarab/crosshair.png', -1)
		self.orig_mask = self.img_crosshair[:,:,3]
		self.orig_mask_inv = cv2.bitwise_not(self.orig_mask)
		self.img_crosshair = self.img_crosshair[:,:,0:3]
		self.video_capture = cv2.VideoCapture(0)
		ret, frame = self.video_capture.read()
		self.resized_height = int(frame.shape[:2][0] * 1.5)
		self.resized_width = int(frame.shape[:2][1] * 1.5)
		frame = cv2.resize(frame, (self.resized_width, self.resized_height), interpolation = cv2.INTER_AREA)
		frame = cv2.flip(frame, 0)
		# Assumed 120 deg horizontal angle of view, 90 deg vertical angle of view
		# no distortion
		self.cam_vert_angle = math.radians(109.69) # rad
		self.cam_horiz_angle = math.radians(124.32) # rad 
		self.px_vert_angle = self.resized_height / self.cam_vert_angle # px / rad
		self.px_horiz_angle = self.resized_width / self.cam_horiz_angle # px / rad
		self.crosshair_width = 64
		self.crosshair_height = 64
		self.crosshair = cv2.resize(self.img_crosshair, (self.crosshair_width, self.crosshair_height), interpolation = cv2.INTER_AREA)
		self.mask = cv2.resize(self.orig_mask, (self.crosshair_width, self.crosshair_height), interpolation = cv2.INTER_AREA)
		self.mask_inv = cv2.resize(self.orig_mask_inv, (self.crosshair_width, self.crosshair_height), interpolation = cv2.INTER_AREA)
		cv2.namedWindow('Video')
		self.x = int(self.resized_width / 2)
		self.y = int(self.resized_height / 2)
		self.tilt_angle = math.radians(tilt)
		self.pan_angle = math.radians(pan)
		self.video_writer_on = False
		self.fourcc = cv2.VideoWriter_fourcc(*'H264')
		self.font = cv2.FONT_HERSHEY_SIMPLEX
		self.speed_text = "0 FT/S"
		self.altitude_text = "0 FT"
		self.show_drop_info = False
