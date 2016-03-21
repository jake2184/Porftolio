#!/usr/bin/env python

import roslib;
import rospy
import math
import time
import numpy as np
import threading

import tf
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry
from core.msg import ToSpeech
from geometry_msgs.msg import Point, Quaternion, PoseStamped

REFERENCE_FRAME = "/openni_depth_frame"


class visionController:

	def __init__(self):
		self.maxDist = 6
		self.minDist = 0.001

		self.last_value = [(self.maxDist+1, self.maxDist+1, self.maxDist+1)]*11
		self.sentCommand = threading.Event()
		self.movementCompleted = threading.Event()
		self.requestReceived = threading.Event()

		self.detectPublisher = rospy.Publisher('vision/detection', Odometry, queue_size=10)
		self.userPublisher = rospy.Publisher('vision/conversation_ended', Bool, queue_size=10)
		self.movPublisher = rospy.Publisher('vision/conversation_started', Bool, queue_size=10)

		self.currentUser = 0;

		# Subscribe to whether movement is finished
		rospy.Subscriber('movement/completed', Bool, self.handleMovement)

		# Subscribe to speech
		rospy.Subscriber("speech/synth", ToSpeech, self.handleRequestStart)
		rospy.Subscriber('android/speech_finished', Bool, self.handleRequestEnd)
		
		r = rospy.Rate(10.0)
		listener = tf.TransformListener()

		while not rospy.is_shutdown():
			trans = [(self.maxDist+1, self.maxDist+1, self.maxDist+1)]*11
			rot = [(0,0,0,0)]*11

			# Search for all users
			for i in range(1,11):
				try:
					# We want the position relative to the center of the camera, if the target is slightly to the left/right, we turn towards them
	        		 #position as a translation (x, y, z) and orientation as a quaternion (x, y, z, w)
	        			(trans[i],rot[i]) = listener.lookupTransform('/torso_%d' % i, REFERENCE_FRAME, rospy.Time())
	        		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
	        			continue
        		self.handleCamera(trans, rot)
			time.sleep(0.5)
	
	def handleMovement(self, msg):
		self.movementCompleted.set()

	def handleRequestStart(self, msg):
		self.requestReceived.set()

	def handleRequestEnd(self, msg):
		self.requestReceived.clear()

	def handleCamera(self, trans, rot):
		# If we already sent a previous command, but movement hasn't been completed, ignore current frame
		# If the robot is currently in the middle of speaking, ignore current frame
		if self.requestReceived.isSet() or (self.sentCommand.isSet() and not self.movementCompleted.isSet())	:
			print "Waiting..."
			return

		trans_x = np.array([t[0] for t in trans])
		trans_y = np.array([t[1] for t in trans])

		# If all users are too far away, do nothing, set current user to 0
		if not trans_x.size or not trans_y.size or abs(trans_x.min()) >= self.maxDist or abs(trans_y.min()) >= self.maxDist:
			print "There's no one in range."

			# There are no users close enough
			self.userPublisher.publish(True) if self.currentUser is not 0 else None
			self.currentUser = 0
		else:
			
			# Choose the closest user
			distances = np.add(np.square(trans_x), np.square(trans_y))
			index = np.where((distances >= self.minDist) & (distances == distances.min()))[0]

			if not index:
				print "All users too close."
				return

			index = index[0]

			flag = 0
			# Change current user if necessary
			if self.currentUser is not index and distances[self.currentUser] > self.maxDist**2:
				self.userPublisher.publish(True)
				self.currentUser = index
				print("User is now %d" % self.currentUser)
			else: # Else continue with same user
				index = self.currentUser
				flag = 1

			# Preparing the message to send
			odom = Odometry()
			odom.header.stamp = rospy.Time.now()
			odom.pose.pose.position = Point(*trans[index])

			if trans[index][0] == 0:
				trans[index] = (0.5, trans[index][1], trans[index][2])

			angle = trans[index][1]/float(abs(trans[index][0]))
			rot[index] = tf.transformations.quaternion_from_euler(0, 0, math.atan(angle))

			odom.pose.pose.orientation =Quaternion(*rot[index])

			# If the angle isn't the same value as the previous frame, don't send the command
			# (needed because of the camera lagging)
			if angle != 0 and cmp(trans[index], self.last_value[index]) != 0:
				self.movementCompleted.clear()
				self.sentCommand.set()
				self.movPublisher.publish(True) if not flag	else None # Publish that we're moving
				self.detectPublisher.publish(odom)	# Publish the movement to make
				self.last_value[index] = trans[index]	# Save the value
				print ("Sending movement command.")
			else:
				self.movPublisher.publish(False)	# Publish that we're not moving

if __name__ =="__main__":
	rospy.init_node('visionController')
	try:
		visionController()
	except Exception as e:
		print "Failure: %s" % e
