#!/usr/bin/env python

################################################################################
## {Description}: Code (Bar/QR) Recognition using USB type camera
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

from __future__ import print_function
from __future__ import division

import sys
import rospy
import cv2
import imutils

from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

import numpy as np

from pyzbar import pyzbar
import datetime
import time

class BarcodeRecognition:
	def __init__(self):

		self.bridge = CvBridge()
		self.scanCode = String()
		self.image_received = False
		self.code_received = False
		self.status_received = False

		# Subscribe Image msg
		img_topic = "/cv_camera/image_raw"
		self.image_sub = rospy.Subscriber(img_topic, Image, self.cbImage)

		# Subscribe CameraInfo msg
		imgInfo_topic = "/cv_camera/camera_info"
		self.imageInfo_sub = rospy.Subscriber(imgInfo_topic, CameraInfo, self.cbImageInfo)

		# Subscribe String msg
		mode_topic = "/scan_mode"
		self.mode_sub = rospy.Subscriber(mode_topic, String, self.cbQRmode)

		# Subscribe String msg
		status_topic = "/scan_status"
		self.status_sub = rospy.Subscriber(status_topic, String, self.cbStatus)

		# Subscribe String msg
		code_topic = "/scanned_barcode"
		self.code_sub = rospy.Subscriber(code_topic, String, self.cbCode)

		# Allow up to one second to connection
		rospy.sleep(1)

	def cbImage(self, msg):

		# Convert image to OpenCV format
		try:
			cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		except CvBridgeError as e:
			print(e)

		self.image_received = True
		self.image = cv_image

	def cbImageInfo(self, msg):

		self.image_width = msg.width
		self.image_height = msg.height

	def cbQRmode(self, msg):

		try:
			mode = msg.data
		except KeyboardInterrupt as e:
			print(e)

		self.code_received = True
		self.typeQR = mode

	def cbStatus(self, msg):

		try:
			scan = msg.data
		except KeyboardInterrupt as e:
			print(e)

		self.status_received = True
		self.status = scan

	def preview(self):

		# Overlay some text onto the image display
		timestr = time.strftime("%Y%m%d-%H%M%S")
		cv2.putText(self.image, timestr, (10, 20), 1, 1, 
			(255, 255, 255), 1, cv2.LINE_AA, False)

		# show the output frame
		cv2.imshow("Frame", self.image)
		cv2.waitKey(1)

	# Publishing
	def pubScanCode(self):

		self.scanCode.data = self.barcodeData
		self.code_pub.publish(self.scanCode)

		# Sleep to give the last log messages time to be sent
#		rospy.sleep(0.5)

	# Get the Scanned Barcode
	def getBarcode(self):
#		try:
		if self.image_received:

			# find the barcodes in the frame and decode each of the barcodes
			self.barcodes = pyzbar.decode(self.image)

			# loop over the detected barcodes
			for self.barcode in self.barcodes:

				# extract the bounding box location of the barcode and 
				# draw the bounding box surrounding the barcode on the 
				# image
				(self.x, self.y, self.w, self.h) = self.barcode.rect
				cv2.rectangle(self.image, (self.x, self.y), 
					(self.x + self.w, self.y + self.h), (0, 0, 255), 2)

				# the barcode data is a bytes object so if we want to 
				# draw it on our output image we need to convert it to 
				# a string first
				self.barcodeData = self.barcode.data.decode("utf-8")
				self.barcodeType = self.barcode.type

				# Publishing
				self.pubScanCode()

			# Refresh the image on the screen
			self.preview()

			if self.code_received:
				cv2.putText(self.image, self.typeQR, (10, 60), 2, 
					1, (0, 0, 255), 1, cv2.LINE_AA, False)

				self.code_received = False

			if self.status_received:
				cv2.putText(self.image, self.status, (10, 80), 2, 
					1, (255, 255, 255), 1, cv2.LINE_AA, False)

				self.status_received = False

		else:
			rospy.logerr("No images recieved")

if __name__ == '__main__':

	rospy.loginfo("Barcode Recognition node [ONLINE]...")

	# Initialize
	rospy.init_node("barcode_recognition", anonymous=False)
	barcode = BarcodeRecognition()

	# Camera preview
	while not rospy.is_shutdown():
		barcode.getBarcode()
