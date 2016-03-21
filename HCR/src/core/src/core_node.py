#!/usr/bin/env python

# Controls interaction flow (in theory). 
# Shutdown not needed to release anything

import random
import roslib 
import rospy
import math
import time

from time import sleep

from std_msgs.msg import String
from std_msgs.msg import Bool
from core.msg import Status
from core.msg import ToSpeech
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8
from std_msgs.msg import UInt16

class hcrCore:

	def __init__(self):
		self.facing = False
		self.interacting = False

        # Publishers
		self.toSynth = rospy.Publisher("speech/synth", ToSpeech, queue_size=10)
		self.armWave = rospy.Publisher("armcontrol/wave", Bool, queue_size=10)
		self.pubDestination = rospy.Publisher("user_request/destination", String, queue_size=10)

    # Subscribers
		rospy.Subscriber("movement/completed", Bool, self.updateFacing)
		rospy.Subscriber("vision/conversation_started", Bool, self.updateInteracting)
		rospy.Subscriber("vision/conversation_ended", Bool, self.updateConvEnd)
		rospy.Subscriber("status", Status, self.updateStatus)		

		rospy.init_node("hcrCore")
		self.facing = False
		self.interacting = False

	def updateFacing(self, msg):
		self.facing = True

    # Called when new user is seen, wait until we are facing before greeting
	def updateInteracting(self, msg):
		if msg.data:
			self.facing = False		
			while not self.facing:
				sleep(0.5)
			self.interacting = True
		else:
			self.facing = True
			self.interacting = True
    # Called when user departs of 'bye' is heard
	def updateConvEnd(self, msg):
		self.facing = False
		self.interacting = False

	def updateStatus(self, msg):
		self.mode = msg.mode
		self.heading = msg.heading
		self.long = msg.long
		self.lat = msg.lat 

		
	def attraction(self):
		#Reset destination
		self.pubDestination.publish("")

		print "In attraction"
		while not self.interacting:
			time.sleep(1)	

	def interaction(self):
		print "In interaction"
		self.armWave.publish(True)
		# Varied introduction to help initiate interaction and suggest robot's use
		possible_replies = ["Hello",
							"Oh, hi! I didn't see you there.",
							"Howdy, stranger!",
							"Ah! A new friend!",
							"Hi.",
							"Hello there!",
							"Hey!",
							"Oh look, a human! Hello",
							"Do I smell people?",
							"Greetings!",
							"Good day!"
							]
		reply = possible_replies[random.randrange(1, len(possible_replies)-1, 1)]
		self.toSynth.publish(False, reply)

		possible_replies = ["My name is",
							"I'm called",
							"You can call me",
							"I go by the name",
							"I am the",
							"I'm the",
							]
		reply = possible_replies[random.randrange(1, len(possible_replies)-1, 1)]
		self.toSynth.publish(False, (reply + " PeopleBot."))


		##
		reply = "I am a navigational robot that can "
		possible_replies = ["help you with directions to nearby landmarks.",
							"help you if you are lost.",
							"give you directions to nearby attractions.",
							"provide navigational help if you are lost.",
							"point you towards where you need to go.",
							"help you find your destination."
							]	
		reply += possible_replies[random.randrange(1, len(possible_replies)-1, 1)]
		self.toSynth.publish(False, reply)
		time.sleep(3)


		possible_replies = ["How can I help you?",
							"What can I do for you today?",
							"Do you need any help?",
							"Do you need any help today?",
							"What are you up to today?",
							"Could I possibly offer you some assistance?",
							"May I help you?",
							"Do you need any assistance?"
							]
		reply = possible_replies[random.randrange(1, len(possible_replies)-1, 1)]
		self.toSynth.publish(True, reply)		
	
	    # Wait until signal from vision before exiting to allow new interaction/greeting
		while self.interacting:
			time.sleep(1)
		
	def mainLoop(self):
		while True:
			self.attraction()
			self.interaction()

if __name__ =="__main__":
	try:
		core = hcrCore()
		core.mainLoop()
		
	except Exception as e:
		print e
		print "Fail"