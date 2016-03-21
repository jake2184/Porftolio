#!/usr/bin/env python

import rospy
import time
import thread
import urlparse
import cgi
import logging
import ctypes
import serial
from core.msg import Status

from std_msgs.msg import String
from std_msgs.msg import Bool
from core.msg import Coordinates
from std_msgs.msg import Float32
from std_msgs.msg import UInt16

class arm:

	def __init__(self):
		# set callbacks
		rospy.Subscriber("armcontrol/wave", Bool, self.waveArm)
		#rospy.Subscriber("armcontrol/direct", String, self.directArm)
		rospy.Subscriber("armcontrol/stop", Bool, self.stopArm)
		rospy.Subscriber("user_request/bearing", Float32, self.directArm)
		rospy.Subscriber("status", Status, self.getStatus)

		self.armServoPublisher = rospy.Publisher("servo", UInt16, queue_size=10)
		self.heading = 0

		# and spin..
		rospy.spin()

	def waveArm(self, msg):
		self.armServoPublisher.publish(1)
		print 'Wave'

	def getStatus(self, msg):
		self.heading = msg.heading

	def directArm(self, msg):
		bearing = msg.data+self.heading
		bearing = bearing % 360
		if bearing == 0 :
			self.armServoPublisher.publish(2)
		elif (bearing < 180 and bearing > 0):
			self.armServoPublisher.publish(4)
		else:
			self.armServoPublisher.publish(3)

		print 'Directing arm'

	def stopArm(self, msg):
		self.armServoPublisher.publish(5)
		print 'Stop'



if __name__ == "__main__":
	rospy.init_node('arm')
	try:
		arm()
	except Exception as e:
		print e
