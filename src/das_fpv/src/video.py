#!/usr/bin/env python

# Stream Chasers Aircraft Research and Build
# Copyright TBD

# FPV/Video Overlay Integration

import cv2

class Video:
	def update_loc(self, x, y, alt):
		# Called from callback_vfr_hud. Updates the overlaid landing location.
		# ft -> pixel to be implemented here.
		# Assumptions: 4:3 640x480, 1 ft altitude -> 1 ft/480 px. Camera pointed directly down.
		alt_pix = 1 / 480 # ft/px per foot of altitude
		if alt > 0:
			self.x = self.cam_height / 2 + x / alt_pix / alt
			self.y = self.cam_width / 2 + y / alt_pix / alt
		else:
			# if the reported altitude is less than 0, leave the initial location (center of the screen)
			self.x = self.cam_height / 2
			self.y = self.cam_width / 2
			
	def overlay(self):
		# This while loop runs continuously in a new thread while the video window is open
		while True:
			ret, frame = self.video_capture.read()
			# Define the range of interest, if it's outside the screen, crop the crosshair and overlay.
			y1 = self.y - self.crosshair_height / 2
			y2 = self.y + self.crosshair_height / 2
			if y1 < 0:
				
				y2 = y2 - y1
				y1 = 0
			if y2 > self.cam_width:
				y1 = y1 - (y2 - self.cam_width)
				y2 = self.cam_width
			x1 = self.x - self.crosshair_height / 2
			x2 = self.x + self.crosshair_height / 2
			if x1 < 0:
				x2 = x2 - x1
				x1 = 0
			if x2 > self.cam_height:
				x1 = x1 - (x2 - self.cam_height)
				x2 = self.cam_height
			roi = frame[y1:y2, x1:x2]
			roi_bg = cv2.bitwise_and(roi, roi, mask = self.mask_inv)
			roi_fg = cv2.bitwise_and(self.crosshair, self.crosshair, mask = self.mask)
			dst = cv2.add(roi_bg, roi_fg)
			frame[y1:y2, x1:x2] = dst
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
		self.crosshair_width = 64
		self.crosshair_height = 64
		self.crosshair = cv2.resize(self.img_crosshair, (self.crosshair_width, self.crosshair_height), interpolation = cv2.INTER_AREA)
		self.mask = cv2.resize(self.orig_mask, (self.crosshair_width, self.crosshair_height), interpolation = cv2.INTER_AREA)
		self.mask_inv = cv2.resize(self.orig_mask_inv, (self.crosshair_width, self.crosshair_height), interpolation = cv2.INTER_AREA)
		cv2.namedWindow('Video')
		self.x = self.cam_width / 2
		self.y = self.cam_height / 2
