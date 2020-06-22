#!/usr/bin/env python

################################################################################
## {Description}: Record the QR/Bar Code on CustomerDatabase and Sent Email
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

from std_msgs.msg import String, Int32
from common_barcode_application.msg import boxStatus

from pyzbar import pyzbar
import datetime
import time
import os
import rospkg
import numpy as np

import pyqrcode

import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from email.mime.base import MIMEBase
from email import encoders
import os.path

class BarcodeRecord:
	def __init__(self):

		self.rospack = rospkg.RosPack()
		self.scanStatus = String()
		self.boxPos = Int32()

		self.code_received = False
		self.mode_recieved = False
		self.box_recieved = False

		# initialize the output directory path and create the output directory
		self.p = os.path.sep.join([self.rospack.get_path('common_barcode_application')])
		self.outputDir = os.path.join(self.p, "csv")

		try:
			os.makedirs(self.outputDir)
		except OSError as e:
			print(e)

		self.csv_filename = self.outputDir + "/store_barcode" + ".csv"
		self.csv = open(self.csv_filename, "a")
		self.found = set()

		self.outputQRDir = os.path.join(self.p, "qr_code")

		try:
			os.makedirs(self.outputQRDir)
		except OSError as e:
			print(e)

		# Subscribe String msg
		code_topic = "/scanned_barcode"
		self.code_sub = rospy.Subscriber(code_topic, String, self.cbCode)

		# Subscribe String msg
		mode_topic = "/scan_mode"
		self.mode_sub = rospy.Subscriber(mode_topic, String, self.cbQRmode)

		# Subscribe boxStatus msg
		box_topic = "/box_available"
		self.box_sub = rospy.Subscriber(box_topic, boxStatus, self.cbBoxID)

		# Publish String msg
		status_topic = "/scan_status"
		self.status_pub = rospy.Publisher(status_topic, String, queue_size=10)

		# Publish Int32 msg
		position_topic = "/box_position"
		self.boxPos_pub = rospy.Publisher(position_topic, Int32, queue_size=10)

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

	def cbBoxID(self, msg):

		try:
			boxPos = msg.data
		except KeyboardInterrupt as e:
			print(e)

		self.box_recieved = True
		self.position = boxPos

		self.boxID = np.array(self.position)
		self.boxID = np.where(self.boxID == 1)[0]

	def pubBoxPos(self):

		# Publish
		self.boxPos.data = self.boxID[0] + 1
		self.boxPos_pub.publish(self.boxPos)

		# Sleep to give the last log messages time to be sent
#		rospy.sleep(0.5)

	def generateQR(self):

		# Generate QR Code
		self.custQR = self.qr.rsplit(',', 1)[0]
		self.urlcustomer = pyqrcode.create(self.custQR)
		self.customerQR = self.outputQRDir + "/" + self.custQR + ".png"
		self.urlcustomer.png(self.customerQR, scale=8)

	def pushEmail(self):

		# Email QR Code
		email = 'wansnap@gmail.com' # Your email
		password = 'Kh@irulizwan1984' # Your email account password
		send_to_emails = [self.qr.rsplit(',', 1)[1], 'shafikahdarwis@gmail.com'] # List of email addresses
		subject = "Order ID " + self.custQR # The subject line
		message = "Please proceed to our Self Collect Machine, and show the Code" # The message in the email
		file_location = self.customerQR

		# Create the attachment file (only do it once)
		filename = os.path.basename(file_location)
		attachment = open(file_location, "rb")
		part = MIMEBase('application', 'octet-stream')
		part.set_payload(attachment.read())
		encoders.encode_base64(part)
		part.add_header('Content-Disposition', "attachment; filename= %s" % filename)

		# Connect and login to the email server
		server = smtplib.SMTP('smtp.gmail.com', 587)
		server.starttls()
		server.login(email, password)

		# Loop over each email to send to
		for send_to_email in send_to_emails:
			# Setup MIMEMultipart for each email address (if we 
			# don't do this, the emails will concat on each email sent)
			msg = MIMEMultipart()
			msg['From'] = email
			msg['To'] = send_to_email
			msg['Subject'] = subject

			# Attach the message to the MIMEMultipart object
			msg.attach(MIMEText(message, 'plain'))
			# Attach the attachment file
			msg.attach(part)

			# Send the email to this specific email address
			server.sendmail(email, send_to_email, msg.as_string()) 

		# Quit the email server when everything is done
#		rospy.logwarn("Detail has been emailed")
		server.quit()

	def pubStatus(self):

		self.status_pub.publish(self.scanStatus)

		# Sleep to give the last log messages time to be sent
		rospy.sleep(0.5)

	def update(self):

		if self.mode_recieved and self.mode == "store":
			if self.box_recieved and len(self.boxID) > 0:
				# if the barcode text is currently not in our CSV file, write
				# the timestamp + barcode to disk and update the set
				if self.code_received and self.qr not in self.found:
					self.csv.write("{},{},{}\n".format(datetime.datetime.now(), 
						self.qr, int(self.boxID[0])))

					self.csv.flush()
					self.found.add(self.qr)

					# Publish
					self.pubBoxPos()

					# Generate QR Code
					self.generateQR()

					# Email QR Code
					self.pushEmail()

#					self.code_received = False

				else:
					# Publish
					self.scanStatus = "Scanned!"
					self.pubStatus()

#				self.box_recieved = False

			else:
				self.scanStatus = "No Empty Box Available"
				self.pubStatus()

#			self.mode_recieved = False

if __name__ == '__main__':

	# Initializing your ROS Node
	rospy.init_node('barcode_record', anonymous=False)
	rec = BarcodeRecord()

	# Camera preview
	while not rospy.is_shutdown():
		rec.update()
