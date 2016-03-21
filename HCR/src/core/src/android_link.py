#!/usr/bin/env python

# Handles communication between Android tablet and ROS framework.
# Provides a message stack and a HTTP server, which the tablet regularly polls.

import rospy
import time
import thread
import urlparse
import cgi
import logging

from BaseHTTPServer import HTTPServer, BaseHTTPRequestHandler

from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from core.msg import Coordinates
from core.msg import ToSpeech

class PostHandler(BaseHTTPRequestHandler):

	messageStack = None
	coordinateCallback = None
	orientationCallback = None
	speechFinishedCallback = None

	def do_GET(self):
		parsed_path = urlparse.urlparse(self.path)
		query = parsed_path.query
		query_components = dict(qc.split("=") for qc in query.split("&"))
		action = query_components["action"]
		if action == "COORDINATE":
			lat = float(query_components["lat"])
			long = float(query_components["long"])
			coordinate = Coordinates()
			coordinate.lat = lat
			coordinate.long = long
			if self.coordinateCallback is not None:
				print 'Publishing new coordinates: ', coordinate
				self.coordinateCallback.publish(coordinate)
		elif action == "ORIENTATION":
			if self.orientationCallback is not None:
				orientation = float(query_components["orientation"])
				print 'Publishing new orientation: ', orientation
				self.orientationCallback.publish(orientation)
		elif action == "SPEECH_FINISHED":
			self.speechFinishedCallback.publish(True)
		else:
			sequenceId = query_components["sequenceId"]
			print "Received request for: ", sequenceId
			if sequenceId is not None:
				sequenceId = int(sequenceId)
				print sequenceId, " messageStack ", len(self.messageStack)
				if sequenceId == len(self.messageStack):
					print "Is up to date."
					self.send_response(200)
					self.end_headers()
					self.wfile.write("UPTODATE")
				elif sequenceId < len(self.messageStack) and sequenceId >= 0:
					print "Has to update."
					self.send_response(200)
					self.end_headers()
					self.wfile.write(self.messageStack[sequenceId])
				else:
					print "Not in sequence..."
					self.send_response(403)
					self.end_headers()
			else:
				self.send_response(403)
				self.end_headers()		
		return

class android_link:

	def __init__(self):
		# Initialise stack
		self.messageStack = []
		PostHandler.messageStack = self.messageStack

		# Initalise publishers
		self.publishCoordinates = rospy.Publisher("android/coordinates", Coordinates, queue_size=10)
		self.publishHeading = rospy.Publisher("android/heading", Float32, queue_size=10)
		self.publishSpeechFinished = rospy.Publisher("android/speech_finished", Bool, queue_size=10)
		PostHandler.coordinateCallback = self.publishCoordinates
		PostHandler.orientationCallback = self.publishHeading
		PostHandler.speechFinishedCallback = self.publishSpeechFinished

		server = HTTPServer(('0.0.0.0', 8000), PostHandler)
		print 'Starting server, user <Ctrl-C> to stop'
		thread.start_new_thread(server.serve_forever, ())
		print 'Starting ROS node'
	
		# set callbacks
		rospy.Subscriber("speech/synth", ToSpeech, self.addSynthString)
		rospy.Subscriber("user_requests/show_map", Coordinates, self.showMap)
		rospy.Subscriber("user_requests/hide_map", Bool, self.hideMap) 

		# and spin..
		rospy.spin()

	def addSynthString(self, msg):
		print 'Synthesizing ' + msg.toSay
		self.messageStack.append("TEXT:" + str(msg.completed) + "---" + msg.toSay)


	def showMap(self, msg):
		self.messageStack.append("SHOW_MAP:" +  str(msg.long) + "---" + str(msg.lat))

	def hideMap(self, msg):
		self.messageStack.append("HIDE_MAP")

if __name__ == "__main__":
	rospy.init_node('android_link')
	try:
		android_link()
	except Exception as e:
		print e