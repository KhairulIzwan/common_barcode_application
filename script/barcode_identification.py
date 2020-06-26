#!/usr/bin/env python

################################################################################
## {Description}: Identify the QR/Bar Code -- CustomerQR or StoreQR
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

from pyzbar import pyzbar
import datetime
import time
import os
import rospkg

import re

class BarcodeIdentity_node:
	def __init__(self):

		self.typeQR = String()
		self.code_received = False

		# Subscribe String msg
		code_topic = "/scanned_barcode"
		self.code_sub = rospy.Subscriber(code_topic, String, self.cbCode)

		# Publish String msg
		mode_topic = "/scan_mode"
		self.mode_pub = rospy.Publisher(mode_topic, String, queue_size=10)

	def cbCode(self, msg):

		try:
			qrcode = msg.data
		except KeyboardInterrupt as e:
			print(e)

		self.code_received = True
		self.qr = qrcode

	def pubMode(self):

		self.mode_pub.publish(self.typeQR)

		# Sleep to give the last log messages time to be sent
#		rospy.sleep(0.1)

	def getMode(self):

		if self.code_received:
			# Identify the barcode data
			# Current approach:
			# StoreQR: OrderID & CustomerEmail
			# CustomerQR: OrderID
			if len(self.qr.split(",")) == 2:
				self.typeQR.data = "store"
			elif len(self.qr.split(",")) == 1:
				self.typeQR.data = "customer"

#			lst = re.findall('\S+@\S+', self.qr)
#
#			if len(lst) == 1:
#				self.typeQR.data = "store"
#
#			elif len(lst) == 0:
#				self.typeQR.data = "customer"

			self.pubMode()

			self.code_received = False

if __name__ == '__main__':

	rospy.loginfo("Barcode Identification node [ONLINE]...")

	# Initialize
	rospy.init_node("barcode_identification", anonymous=False)
	qr = BarcodeIdentity_node()

	# Camera preview
	while not rospy.is_shutdown():
		qr.getMode()
