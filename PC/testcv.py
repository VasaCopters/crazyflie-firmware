#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2, math
import numpy as np
import math
import time
from threading import Thread
import cflib
from cflib.crazyflie import Crazyflie
import logging
logging.basicConfig(level=logging.ERROR)

def nothing(x):
    pass

class CrazyTracker:
	def __init__(self):
		# Init cameras
		self._camera_devices = [0, 1]
		self._w = 640
		self._h = 480
		self._camera = {}
		self._pos = {}

		# Camera properties
		self._clip = 620

		for i in self._camera_devices:
			self._camera[i] = cv2.VideoCapture(i)
			self._camera[i].set(cv2.CAP_PROP_FRAME_WIDTH, self._w)
			self._camera[i].set(cv2.CAP_PROP_FRAME_HEIGHT, self._h)
			self._pos[i] = ((0,0),0)
			self._camera[i].set(cv2.CAP_PROP_EXPOSURE, -6)

		# Overlay to see thresholds
		self._overlay = np.zeros((self._h,self._w, 3), np.uint8)
		self._overlay[:,:,:] = (255, 221, 201)
		
		# Create controlpanel
		cv2.namedWindow('Controls')
		cv2.createTrackbar('H-Value','Controls',251,255,nothing)
		cv2.createTrackbar('H-Range','Controls',15,128,nothing)
		cv2.createTrackbar('S-Low','Controls',102,255,nothing)
		cv2.createTrackbar('S-High','Controls',255,255,nothing)
		cv2.createTrackbar('V-Low','Controls',95,255,nothing)
		cv2.createTrackbar('V-High','Controls',245,255,nothing)
		cv2.createTrackbar('Gauss','Controls',10,50,nothing)
		cv2.createTrackbar('MinSize','Controls',4,96,nothing)
		cv2.createTrackbar('Overlay', 'Controls',0,1,nothing)
		cv2.createTrackbar('Thrust', 'Controls',0,60000,nothing)
		cv2.resizeWindow('Controls', 350, 300)
		self._font = cv2.FONT_HERSHEY_SIMPLEX

	def run(self):

		# Init Crazyflier
		cflib.crtp.init_drivers(enable_debug_driver=False)
		print "Scanning interfaces for Crazyflies..."
		available = cflib.crtp.scan_interfaces()
		time.sleep(2)
		print "Crazyflies found:"
		for i in available:
			print i[0]
		uri = available[-1][0]
		cf = Crazyflie()
		cf.open_link(uri)
		time.sleep(0.5)
		cf.commander.send_setpoint(0, 0, 0, 0)

		# Init image dictionaries
		img = {}
		img_orig = {}
		img_blur = {}
		img_hsv = {}
		mask_hsv = {}
		mask_hsv_inv = {}
		img_masked = {}
		img_tinted = {}

		while True:

			# Read controlpanel
			h_value 	= cv2.getTrackbarPos('H-Value', 'Controls')
			h_range		= cv2.getTrackbarPos('H-Range', 'Controls')
			s_low 		= cv2.getTrackbarPos('S-Low', 'Controls')
			s_high 		= cv2.getTrackbarPos('S-High', 'Controls')
			v_low 		= cv2.getTrackbarPos('V-Low', 'Controls')
			v_high 		= cv2.getTrackbarPos('V-High', 'Controls')
			gauss		= cv2.getTrackbarPos('Gauss','Controls') * 2 + 1
			min_size	= cv2.getTrackbarPos('MinSize','Controls') + 1
			overlay 	= cv2.getTrackbarPos('Overlay','Controls')
			thrust 		= cv2.getTrackbarPos('Thrust','Controls')
			show_overlay= cv2.getTrackbarPos('Overlay','Controls') == 1

			h_min = h_value - h_range
			h_max = h_value + h_range

			for i in self._camera_devices:
				self._camera[i].grab()

			for i in self._camera_devices:
				_, img_orig[i] = self._camera[i].retrieve()
				img_blur[i] = cv2.GaussianBlur(img_orig[i], (gauss,gauss), 0)
				img_hsv[i] = cv2.cvtColor(img_blur[i], cv2.COLOR_BGR2HSV_FULL)

				# Take care of region split for hue (red warps around 255)
				if h_min < 0 or h_max > 255:
					if h_min < 0:
						h_upper = h_min + 255
						h_lower = h_max

					elif h_max > 255:
						h_upper = h_min
						h_lower = h_max - 255

					mask_lower1 = np.array([h_upper, s_low, v_low],np.uint8)
					mask_upper1 = np.array([255, s_high, v_high],np.uint8)
					mask_hsv_lower = cv2.inRange(img_hsv[i], mask_lower1, mask_upper1)

					mask_lower2 = np.array([0, s_low, v_low],np.uint8)
					mask_upper2 = np.array([h_lower, s_high, v_high],np.uint8)
					mask_hsv_upper = cv2.inRange(img_hsv[i], mask_lower2, mask_upper2)

					mask_hsv[i] = cv2.bitwise_or(mask_hsv_lower, mask_hsv_upper)
				else:
					mask_lower = np.array([h_min, s_low, v_low],np.uint8)
					mask_upper = np.array([h_max, s_high, v_high],np.uint8)
					mask_hsv[i] = cv2.inRange(img_hsv[i], mask_lower, mask_upper)
				
				# close and open mask (remove small objects)
				kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(min_size,min_size))
				mask_hsv[i] = cv2.morphologyEx(mask_hsv[i], cv2.MORPH_CLOSE, kernel)
				mask_hsv[i] = cv2.morphologyEx(mask_hsv[i], cv2.MORPH_OPEN, kernel)

				img_masked[i] = cv2.bitwise_and(img_blur[i], img_blur[i], mask = mask_hsv[i])

				#IMPORTANT: This row will change the mask to 0-1 instead of 0-255 and also there will be some countour lines if you invert the mask
				_, contours, hierarchy = cv2.findContours(mask_hsv[i], cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
				
				# Find largest countur (we might need to change this to a range)
				max_area = 0
				largest_contour = None
				for idx, contour in enumerate(contours):
					area = cv2.contourArea(contour)
					if area > max_area:
						max_area = area
						largest_contour = contour

				if not largest_contour == None:
					moment = cv2.moments(largest_contour)
					self._pos[i]  = cv2.minEnclosingCircle(largest_contour)
					found = True
				else:
					found = False


				#NOT NICE CODE!! DON'T LOOK HERE!!!
				if show_overlay:
					mask_hsv_inv[i] = 1 - mask_hsv[i] #Invert now for nice effect
					img_tinted[i] = cv2.addWeighted(img_orig[i], 0.2, self._overlay, 0.8, 0)
					cv2.bitwise_xor(img_tinted[i], img_tinted[i], img_tinted[i], mask = mask_hsv[i])
					cv2.bitwise_xor(img_orig[i], img_orig[i], img_orig[i], mask = mask_hsv_inv[i])
					img[i] = cv2.add(img_tinted[i], img_orig[i])
				else:
					img[i] = img_orig[i]
					x = int(self._pos[i][0][0])
					y = int(self._pos[i][0][1])
					r = int(self._pos[i][1])
					if found:
						cv2.circle(img[i], (x, y), r, (255, 0, 255), 2)
					


			if found:
				x1 = self._pos[0][0][0]
				x2 = self._pos[1][0][0]
				y1 = self._pos[0][0][1]
				y2 = self._pos[1][0][1]

				B = 280
				D = x2 - x1
				f = self._clip
				z = f*B/D
				x = {}
				y = {}
				x[0] = (x1-self._w/2)*z/f
				y[0] = (self._h-y1)*z/f
				x[1] = (x2-self._w/2)*z/f
				y[1] = (self._h-y2)*z/f

				sendx = int(x[0])
				sendy = int(z)
				sendz = int(y[0])
				#SEND THE COMMAND!
				cf.commander.send_setpoint(sendx, sendy, sendz, thrust)

			for i in self._camera_devices:
				if not show_overlay and found:
					cv2.putText(img[i], 'XYZ:('+`sendx` + ';' + `sendy`+';'+`sendz`+')' , (10, self._h-15), self._font, 1, (0, 255, 255), 1, cv2.LINE_AA)
				cv2.imshow("Video" + `i`, img[i])

			if cv2.waitKey(1) == 27:
				cf.commander.send_setpoint(0, 0, 0, 0)
				time.sleep(0.1)
				cf.close_link()
				cv2.destroyAllWindows()
				for i in self._camera_devices:
					self._camera[i].release()
				break



if __name__ == '__main__':
	program = CrazyTracker()
	program.run()