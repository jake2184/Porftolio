#!/usr/bin/env python

import roslib
import rospy
from core.msg import Status
from core.msg import ToSpeech
from std_msgs.msg import String
from std_msgs.msg import Float32
import math
import googlemaps
import pprint
from bs4 import BeautifulSoup
import re
from textblob import TextBlob

class directionsHandler:

	# Update values from status node
	def updateStatus(self, msg):
		self.mode = msg.mode
		self.heading = msg.heading
		self.long = msg.long
		self.lat = msg.lat
		self.location = (msg.lat, msg.long)

	# Find a place given a place name
	def getPlaceLatLon(self, place):
		print "Google place search: ",
		print place
		place = self.gmaps.places(place, location=self.location, radius=self.baseRadius)
		try:
			# Assume top result is desired place
			place = place['results'][0]
			#pprint.pprint(place, depth=4)
			# Try and return additional information - opening hours and rating
			try: return (place['geometry']['location'], place['name'], place['opening_hours'], place['rating'])
			# Otherwise just return basic information
			except: return (place['geometry']['location'], place['name'])
		except IndexError:
			print "No place result found"
			return None


	# Get directions from an origin to a destination
	def getDirections(self, origin, destination):
		# returns copyrights, legs, overview_polyline, summary, warnings, waypoint_order
		# Apply constraints on result - walking results only, avoiding indoor steps, and within the UK
		result = self.gmaps.directions(origin, destination[0], mode="walking", avoid="indoor", region="uk")
		try:
			# Assume first route result is best
			result = result[0]
			instruction_str = ""

			#pprint.pprint(result, depth=6)
			print "Directions to",
			print destination[1]
			# For each navigational step
			for i in range(0, len(result['legs'][0]['steps'])):
				instruction = result['legs'][0]['steps'][i]['html_instructions']	# extract HTML instructions
				instruction = re.sub(r'<div.*?>', "\n", instruction)	# regex to replace div tags with line breaks
				soup = BeautifulSoup(instruction, "lxml")
				instruction = soup.get_text()	# remove remaining HTML tags
				instruction = re.sub(r'\/[ABM]\d+', "", instruction)	# regex to remove road numbers
				#print instruction
				dist = (result['legs'][0]['steps'][i]['distance']['text'], result['legs'][0]['steps'][i]['distance']['value'])
				dur = (result['legs'][0]['steps'][i]['duration']['text'], result['legs'][0]['steps'][i]['duration']['value'])
				
				# Append distance and duration details
				if "Head " in instruction: instruction += " for " + dist[0] + " taking " + dur[0]
				elif " will be " in instruction: instruction += " after " + dist[0] + " and " + dur[0]
				elif "Continue " in instruction: instruction += " for " + dist[0] + " taking " + dur[0]
				elif "Turn " in instruction: instruction += " after " + dist[0] + " and " + dur[0]
				elif "Slight " in instruction: instruction += " after " + dist[0] + " and " + dur[0]
				
				# Expand abbreviations
				instruction = instruction.replace(" N ", " North ")
				instruction = instruction.replace(" E ", " East ")
				instruction = instruction.replace(" S ", " South ")
				instruction = instruction.replace(" W ", " West ")
				instruction = instruction.replace(" Rd", " Road")
				instruction = instruction.replace(" St", " Street")
				instruction = instruction.replace(" Dr", " Drive")
				instruction = instruction.replace(" m ", " metres ")
				instruction = instruction.replace(" km ", " kilometres ")
				instruction = instruction.replace(" min", " minute")
				instruction = instruction.replace(" mins", " minutes")
				instruction = instruction.replace("Slight ", "Turn slightly ")
				instruction = instruction.replace("Destination", destination[1])
				instruction = instruction.replace("\n", ".\n")
				instruction += ".\n"
				instruction_str += instruction
			wayp0LatLon = tuple(result['legs'][0]['steps'][0]['end_location'].values())	# coordinates of first waypoint
			overviewPolyline = result['overview_polyline']['points']
			return (instruction_str, wayp0LatLon, destination[1], overviewPolyline)

		except IndexError:
			print "No directions result found"


	# Extract noun phrases from full sentences
	def getNounPhrases(self, message):
		blob = TextBlob(message)
		searchString = ""
		# For each noun phrase, append it to search string
		for np in blob.noun_phrases:
			searchString += (np + " ")
			print "Noun phrases: ",
			print searchString
		# If no noun phrases found, return original message
		if searchString == "":
			searchString = message
		return searchString


	# Check whether places in directory exist in query, and return place if so
	def matchKnownPlaces(self, message):
		places = ["natural history museum", "science museum", "v and a", "victoria and albert", "imperial", "royal albert hall", "hyde park", "albert memorial"]
		found = False
		for place in places:
			if place in message.lower():
				message = place
				found = True
				print "Known place: ",
				print message
		# Special case to deal with the names 'tube' and 'underground'
		if "tube" in message.lower() or "underground" in message.lower():
			message = "south kensington station"
			found = True
			print "Known place: ",
			print message
		if found is False:
			message = None
			print "Unknown place"
		return (found, message)


	# Get a bearing between two coordinates
	def getBearing(self, loc1, loc2):
		lat1 = math.radians(loc1[0])
		lat2 = math.radians(loc2[0])
		diffLon = math.radians(loc2[1] - loc1[1])

		y = math.sin(diffLon) * math.cos(lat2)
		x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(diffLon)

		bearing = math.degrees(math.atan2(y,x))
		
		# Constrain to 0 <= bearing < 360
		bearing = (bearing + 360) % 360
		return bearing		


	# Take navigational query and return directions
	def recvDestination(self, msg):
		try:
			message = msg.data
			#message = msg
			print "Input message: ",
			print message
			# Check whether directory provides a match	
			matchResult = self.matchKnownPlaces(message)
			if matchResult[0]:
				# Take result from directory
				searchString = matchResult[1]
			else:
				# Extract noun phrases from sentence
				searchString = self.getNounPhrases(msg.data)
				if searchString == "please ": searchString = message # special case to deal with erroneous 'please'

			# Get coordinates
			destLatLon = self.getPlaceLatLon(searchString)		
			if destLatLon is not None:
				# Get directions
				directions = self.getDirections(self.location, destLatLon)
				instructions = directions[0]
				wayp0LatLon = directions[1]
				outDestination = directions[2]
				overviewPolyline = directions[3]
				bearing = self.getBearing(self.location, wayp0LatLon)
				if len(destLatLon) > 2:
					placeOpen = destLatLon[2]['open_now']
					rating = destLatLon[3]*2
				self.toSynth.publish(False,("Here are your directions to " + outDestination + "."))
				print instructions
				
				# Publish directions
				self.toSynth.publish(True,instructions)
				self.pubDirections.publish(instructions)
				
				# Publish destination
				self.pubDestination.publish(outDestination)
				
				# Publish bearing
				print bearing
				self.pubBearing.publish(bearing)
				
				# Publish overview polyline
				print overviewPolyline
				self.pubPolyline.publish(overviewPolyline)
				
				# Publish additional place information if it exists
				if len(destLatLon) > 2:
					print "Open: " + str(placeOpen)
					print "Rating: " + str(rating)
					outString = ""
					if placeOpen:
						infoString = (outDestination + " is currently open, and has a rating of " + str(rating) + " out of ten.")
					else:
						infoString = (outDestination + " is currently closed, but has a rating of " + str(rating) + " out of ten. Maybe you can come back tomorrow.")
					print infoString			
					self.toSynth.publish(True, infoString)
		except Exception as e:
			print "Problem: %s" % e
			outMessage = "Sorry I can't find that place. Please could you try again?"
			print outMessage
			self.toSynth.publish(True, outMessage)
			


	def __init__(self):
		rospy.init_node('directionsHandler')

		self.location = (51.498959, -0.174322);	# default robot base location
		self.baseRadius = 2000;				# search radius in metres
		
		# Dummy API key - replace this with your own
		self.gmaps = googlemaps.Client(key='AIzaSyBk9FOWJWQc-FjdYOC4Mt-BzkC9YMx0sPc')

		# Subscribe to query input from speech
		rospy.Subscriber('user_request/directionsQuery', String, self.recvDestination)

		# Subscribe to status to overwrite location coordinates
		rospy.Subscriber('status', Status, self.updateStatus)

		# Output publishers
		self.toSynth = rospy.Publisher("speech/synth", ToSpeech, queue_size=10)
		self.pubDirections = rospy.Publisher("user_request/directions", String, queue_size=10)
		self.pubBearing = rospy.Publisher("user_request/bearing", Float32, queue_size=10)
		self.pubPolyline = rospy.Publisher("user_request/polyline", String, queue_size=10)
		self.pubDestination = rospy.Publisher("user_request/destination", String, queue_size=10)

		rospy.spin()


if __name__ == '__main__':
	try:
		directionsHandler()
	except Exception as e:
		print "Directions handler failure: %s" % e
