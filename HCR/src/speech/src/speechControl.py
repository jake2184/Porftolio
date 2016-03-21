#!/usr/bin/env python




import roslib; roslib.load_manifest('pocketsphinx')
import rospy
import math
import time
import random

from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from core.msg import ToSpeech

class speechController:

	def __init__(self):

		# Current destination
		self.destination = ""

		# Subscribe to updated destination
		rospy.Subscriber("user_request/desination", String, self.updateDestination)
		
		# Subscribe to Google Speech node output		
		rospy.Subscriber("gspeech/transcript", String, self.handleMessage)
				
		# Information and speech synthesis publishers
		self.request_received = rospy.Publisher('user_request/request_received', Bool, queue_size = 10)
		self.not_understood = rospy.Publisher('user_request/not_understood', Bool, queue_size=10)
		self.toSynth = rospy.Publisher('speech/synth', ToSpeech, queue_size=10)		
		
		# Division of topics to help user_requests
		self.pub_destination = rospy.Publisher('user_request/directionsQuery', String, queue_size=10)
		self.pub_weather = rospy.Publisher('user_request/weather', String, queue_size=10)
		self.pub_question = rospy.Publisher('user_request/question', String, queue_size=10)

		# Wave goodbye
		self.armWave = rospy.Publisher("armcontrol/wave", Bool, queue_size=10)

		rospy.spin()

	def updateDestination(self, msg):
		self.destination = msg.data

	def sayIntro(self):
	    # Range of replies used to increase human likeness.
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

	def handleMessage(self, msg):
		message = msg.data;

        # Perhaps not suitable, as would catch eg 'hello, where is ...'
        # Undesired behaviour would occur. But kept for testing, refinement for 
        # future. Perhaps put later in switch statement
        # Greeting
		if "hello" in message.lower() \
		or "hey" in message.lower():
			self.armWave.publish(True)
			possible_replies = ["Hello",
							"Oh, hi! I didn't see you there.",
							"Howdy, stranger!",
							"Ah! A new friend!",
							"Hi.",
							"Hello there!",
							"Hey!",
							"Oh look, a human! Hello.",
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
			self.sayIntro()

        # Status question
		elif "how are you" in message.lower() \
		or "how are things" in message.lower() \
		or "sup" in message.lower() \
		or "are you alright" in message.lower() \
		or "are you okay" in message.lower():
			possible_replies = ["I'm great thank you!",
							"I'm very good thanks",
							"I'm alright.",
							"I'm fine thank you.",
							"I'm fine thanks.",
							"I'm okay.",
							"I'm bored of talking to humans.",
							"I'm sad, because no one is asking me for directions.",
							"I could be better. My battery is running out.",
							"I'm happy because you came to talk to me.",
							"I'm very happy to be talking to you.",
							"I'm so happy to be talking to such a beautiful human."
							]
			reply = possible_replies[random.randrange(1, len(possible_replies)-1, 1)]
			self.toSynth.publish(False, reply)
			self.sayIntro()
			

		# Direction request
		elif message.lower().find("where") > -1 \
		or message.lower().find("get ") > -1 \
		or message.lower().find("way") > -1 \
		or message.lower().find("direct") > -1 \
		or message.lower().find("route") >-1 \
		or message.lower().find("find") >-1 \
		or message.lower().find("go ") >-1 \
		or message.lower().find("going") >-1 \
			:
			self.pub_destination.publish(message)
			self.request_received.publish(True)
			print ("Direction request")
		
		# Weather request
		elif message.lower().find("weather") > -1 \
			 or message.find("forecast") > -1 \
			:
			self.pub_weather.publish(message)
			self.request_received.publish(True)
			# User Requests cannot handle weather questions
			self.toSynth.publish(True, "The weather is clear and sunny.")
			print("Weather request")	

		# Open question
		elif message.lower().find("who") > -1 \
		     or message.lower().find("what") > -1 \
		     or message.lower().find("why") > -1 \
		     or message.lower().find("when") > -1 \
			or message.lower().find("how") > -1 \
			:
			self.pub_question.publish(message)
			self.request_received.publish(True)
			# User Requests cannot handle open questions
			self.toSynth.publish(True, "I'm sorry, I can only help with directions.")
			print("Open Question request.")

		# End conversation
		elif message.lower().find("bye") > -1 \
		or message.lower().find("thanks") > -1 \
		or message.lower().find("thank you") > -1 \
		or message.lower().find("cheers") > -1 \
		:
			if self.destination:
				possible_replies = ["I hope you have a nice time at ",
							"I hope you enjoy your time at ",
							"Have fun at ",
							"Have a great time at ",
							]
				reply = possible_replies[random.randrange(1, len(possible_replies)-1, 1)]
				reply = (reply + self.destination + ".")
				self.toSynth.publish(False, reply)
			possible_replies = ["Goodbye!",
						"Bye!",
						"See you again next time!",
						]
			reply = possible_replies[random.randrange(1, len(possible_replies)-1, 1)]
			self.toSynth.publish(True, reply)
			self.armWave.publish(True)
			print("Ending conversation.")
			convo_end = rospy.Publisher("vision/conversation_ended", Bool, queue_size=10)
			convo_end.publish(True)
		
		else :
			self.not_understood.publish(True)
			possible_replies = ["I'm sorry, I didn't understand the question. Can I give you directions today?",
						"Can you say that again please? Are you sure you asked me for directions?",
						"I didn't get that. Please ask me for directions.",
						"Sorry, I didn't hear that. Please can you try asking me for directions?",
						"Sorry, what did you say? Did you ask for directions?",
						"Sorry, I didn't catch that. Do you need help finding where to go?"]
			reply = possible_replies[random.randrange(0, len(possible_replies)-1, 1)]
			self.toSynth.publish(True, reply)

if __name__ =="__main__":
	rospy.init_node('speechController')
	try:
		speechController()
	except Exception as e:
		print e