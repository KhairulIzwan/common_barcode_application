#!/usr/bin/env python

################################################################################
## {Description}: Record the QR/Bar Code on CustomerDatabase
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
from std_msgs.msg import Int32

from pyzbar import pyzbar
import datetime
import time
import os
import rospkg
import csv

class BarcodeValidate:
	def __init__(self):

		self.rospack = rospkg.RosPack()
		self.scanStatus = String()
		self.boxID_active = Int32()

		self.code_received = False
		self.mode_recieved = False
		self.box_recieved = False

		# initialize the output directory path and create the output directory
		self.p = os.path.sep.join([self.rospack.get_path('self_collect_machine')])
		self.outputDir = os.path.join(self.p, "csv")

		try:
			os.makedirs(self.outputDir)
		except OSError as e:
			print(e)

		self.csv_filename = self.outputDir + "/customer_barcode" + ".csv"
		self.csv = open(self.csv_filename, "a")
		self.found = set()

		self.csv_filename_store = self.outputDir + "/store_barcode" + ".csv"

		# Subscribe String msg
		code_topic = "/scanned_barcode"
		self.code_sub = rospy.Subscriber(code_topic, String, self.cbCode)

		# Subscribe String msg
		mode_topic = "/scan_mode"
		self.mode_sub = rospy.Subscriber(mode_topic, String, self.cbQRmode)

		# Publish String msg
		status_topic = "/scan_status"
		self.status_pub = rospy.Publisher(status_topic, String, queue_size=10)

		# Publish Int32 msg
		active_topic = "/boxID_activation"
		self.active_pub = rospy.Publisher(active_topic, Int32, queue_size=10)

	def cbCode(self, msg):

		try:
			qrcode = msg.data
		except KeyboardInterrupt as e:
			print(e)

		self.code_received = True
		self.qr = qrcode

	def cbQRmode(self, msg):

		try:
			typeQR = msg.data
		except KeyboardInterrupt as e:
			print(e)

		self.mode_recieved = True
		self.mode = typeQR

	def pubStatus(self):

		self.scanStatus = "Scanned!"
		self.status_pub.publish(self.scanStatus)

		# Sleep to give the last log messages time to be sent
#		rospy.sleep(0.1)

	def pubActive(self):

		self.boxID_active.data = int(self.row[3]) + 1
		self.active_pub.publish(self.boxID_active)

		# Sleep to give the last log messages time to be sent
#		rospy.sleep(0.1)

	def update(self):

		if self.mode_recieved and self.mode == "customer" and len(self.qr.split(",")) == 1:
			# if the barcode text is currently not in our CSV file, write
			# the timestamp + barcode to disk and update the set
			if self.code_received and self.qr.split(",")[0] not in self.found:
				self.csv.write("{},{}\n".format(datetime.datetime.now(), self.qr))
				self.csv.flush()
				self.found.add(self.qr.split(",")[0])
				with open(self.csv_filename_store, 'rt') as f_obj:
					reader = csv.reader(f_obj, delimiter=',')
					# Iterates through the rows of your csv
					for row in reader:
						if str(self.qr) == row[1]:
							# TODO: Un-comment for troubleshoot
							rospy.loginfo("Package(s) in box no {}".format(row[3]))
							self.row = row
							# Publishing
							self.pubActive()
			else:
				# Publishing
				self.pubStatus()

if __name__ == '__main__':

	# Initializing your ROS Node
	rospy.init_node("barcode_validate", anonymous=False)
	valid = BarcodeValidate()

	# Camera preview
	while not rospy.is_shutdown():
		valid.update()
