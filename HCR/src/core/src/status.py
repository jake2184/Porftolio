#!/usr/bin/env python

# Collects robot status and publishes it. Created to allow other nodes easy
# access to data with a provided function. Also reduces data traffic.
# Shutdown not needed, very simple node
# Room for expansion of more data types

import rospy
from core.msg import Status
from core.msg import Coordinates
from std_msgs.msg import Float32

class StatusNode:

	def __init__(self):
		self.mode = 1
		self.heading = 0.0
		self.long = 0.0
		self.lat = 0.0
		
		rospy.init_node("status")		

		statusPub = rospy.Publisher("status", Status, queue_size=10)

		rospy.Subscriber("android/coordinates", Coordinates, self.updateCoordinates)
		rospy.Subscriber("android/heading", Float32, self.updateHeading)
				
		r = rospy.Rate(1)
		while True:
			msg = Status()
			msg.mode = self.mode
			msg.heading = self.heading
			msg.long = self.long
			msg.lat = self.lat
			statusPub.publish(msg)
			r.sleep()

	def updateCoordinates(self, msg):
		self.long = msg.long
		self.lat = msg.lat

	def updateHeading(self, msg):
		self.heading = msg.data

if __name__== '__main__':
	try:
		StatusNode()
	except rospy.ROSInterruptException: pass