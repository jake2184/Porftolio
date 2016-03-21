#!/usr/bin/env python

# See https://github.com/achuwilson/gspeech for original work.
# See https://github.com/vijaravind/ros-gspeech for original work.

# Adapted version of latter link above. Minor changes across file
# Major change in starting/stopping of recognition.


import requests
import json, os, shlex, socket, subprocess, sys
import pyaudio, wave
import audioop
from datetime import datetime
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from std_msgs.msg import String
from gspeech.msg import Speech


class GSpeech(object):
	"""Speech Recogniser using Google Speech API"""

	def __init__(self, _api_key, _threshold):
		"""Constructor"""
		# pyaudio setup
		self.pa_channels = 1
		self.pa_buf_len = 1024
		self.pa_format = pyaudio.paInt16
		self.pa_rate = 44100
		self.pa_wave_out_file = '/tmp/recording.wav'
		self.pa_flac_out_file = '/tmp/recording.flac'
		self.pa_handle = pyaudio.PyAudio()
		self.pa_in_device_info = self.pa_handle.get_default_input_device_info()
		print self.pa_in_device_info
		self.pa_stream = self.pa_handle.open(
			channels=self.pa_channels, frames_per_buffer=self.pa_buf_len,
			format=self.pa_format, input=True,
			input_device_index=self.pa_in_device_info['index'],
			rate=self.pa_rate, start=False
		)
		self.audio_frames = list()    # to store audio capture frames
		# configure system commands
		self.api_key = _api_key
		self.threshold = _threshold
		self.flac_convert = "flac -f {wav_file} -o {flac_file}".format(
			wav_file=self.pa_wave_out_file, flac_file=self.pa_flac_out_file
		)
		self.wget_cmd = ("wget -q -U \"Mozilla/5.0\" ") + \
			("--post-file {flac_file} ") + \
			("--header=\"Content-Type: audio/x-flac; rate=44100\" -O - ") + \
			("\"https://www.google.com/speech-api/v2/recognize") + \
			("?output=json&lang=en-us&key={api_key}\"")
		self.wget_cmd = self.wget_cmd.format(
			api_key=self.api_key, flac_file=self.pa_flac_out_file
		)
		self.wget_args = shlex.split(self.wget_cmd)
		# start ROS node
		rospy.init_node('gspeech', log_level=rospy.DEBUG)
		# configure ROS settings
		rospy.on_shutdown(self.shutdown)
		self.pub_confidence = rospy.Publisher('~confidence', Int8, queue_size=1)
		self.pub_jsonstr = rospy.Publisher('~jsonstr', String, queue_size=1)
		self.pub_recognising = rospy.Publisher('~recognising', Bool, queue_size=1)
		self.pub_speech = rospy.Publisher('~speech', Speech, queue_size=1)
		self.pub_transcript = rospy.Publisher('~transcript', String, queue_size=1)
		
				
		self.sub_stop = rospy.Subscriber("vision/conversation_ended", Bool, self.stop_callback)
		self.sub_start  = rospy.Subscriber('android/speech_finished', Bool, self.start_callback)
		# run speech recognition
		self.recognising = False
		self.started = False


	def start_callback(self, msg):
		"""Start speech recognition"""
		self.started = True
		if not self.recognising:
			self.do_recognition()	
	
	def stop_callback(self, msg):
		self.started = False


	def shutdown(self):
		"""Stop all system process before killing node"""
		self.sub_start.unregister()
		self.sub_stop.unregister()
		self.pa_stream.close()
		self.pa_handle.terminate()

	def do_recognition(self):
		"""Do speech recognition"""
		self.recognising = True

		del self.audio_frames[:]    # empty the list
		self.pa_stream.start_stream()
		rospy.logdebug("recording started")

        # Take background noise sample to help calibrate threshold
		noiseSample = self.pa_stream.read(self.pa_buf_len)
		thresh = audioop.rms(noiseSample, 2) 
		noiseSample = self.pa_stream.read(self.pa_buf_len)
		thresh = thresh + audioop.rms(noiseSample, 2) 
		noiseSample = self.pa_stream.read(self.pa_buf_len)
		thresh = thresh + audioop.rms(noiseSample, 2) 

		thresh = thresh / 3
		print "Background noise: " + str(thresh)	
        # Print it for user to view. Actual value is set manually, sampling 
        # above is a guide of what the value should be set to

		thresh = self.threshold

		belowThresh = 0
		startTime = datetime.now().time().second
		runTime = 0
		# < 10 secs to satify API limits, and stop when we either lose the user
		# (calls stop_callback) or we detect nothing above the threshold for a 
		# reasonable amount of time
		while runTime < 10 and belowThresh < 60 and self.started:			
			self.pub_recognising.publish(self.recognising)
			frame = self.pa_stream.read(self.pa_buf_len)
			# rms - Root Mean Square, power of signal
			# Don't count if <2, to give interacter time to start speaking
			if audioop.rms(frame, 2) < thresh and runTime > 2:
				belowThresh = belowThresh + 1
			else:
				belowThresh = max(0, belowThresh - 5)
			self.audio_frames.append(frame)
			runTime = datetime.now().time().second - startTime 

		#print datetime.now().time().second - startTime
		#print belowThresh
	    # Then process the buffer similar to original fike
		
		self.recognising = False

		self.pa_stream.stop_stream()

		if not self.started:
			return
	
		rospy.logdebug("recording stopped")
		
		#Save speech
		self.save_speech()
		

		# send audio data to google
		wget_out, wget_err = self.send_speech()
		# deal with response
		if not wget_err and len(wget_out) > 16:
			wget_out = wget_out.split('\n', 1)[1]
			a = json.loads(wget_out)['result'][0]
			transcript = ""
			confidence = 0
			if 'confidence' in a['alternative'][0]:
				confidence = a['alternative'][0]['confidence']
				confidence = confidence * 100
				self.pub_confidence.publish(confidence)
				rospy.loginfo("confidence: {}".format(confidence))
			if 'transcript' in a['alternative'][0]:
				transcript = a['alternative'][0]['transcript']
				self.pub_transcript.publish(String(transcript))
				rospy.loginfo("transcript: {}".format(transcript))
			self.pub_speech.publish(
				Speech(transcript=transcript, confidence=confidence)
			)
			self.pub_jsonstr.publish(String(wget_out))
		else:
			rospy.loginfo("There has been an error " + wget_err + " or " + wget_out)
			self.pub_transcript.publish("NULL")
			
		self.recognising = False
		self.pub_recognising.publish(Bool(self.recognising))
		self.pa_stream.stop_stream()

	def save_speech(self):
		"""Save first to WAVE file in `/tmp` folder and convert FLAC format"""
		rospy.logdebug("saving wav file")
		wav_file = wave.open(self.pa_wave_out_file, 'wb')
		wav_file.setnchannels(self.pa_channels)
		wav_file.setsampwidth(self.pa_handle.get_sample_size(self.pa_format))
		wav_file.setframerate(self.pa_rate)
		wav_file.writeframes(b''.join(self.audio_frames))
		wav_file.close()
		rospy.logdebug("converting to flac file")
		os.system(self.flac_convert)

	def send_speech(self):
		"""Send speech (FLAC format) to Google servers and return response"""
		rospy.logdebug("sending speech")
		print self.wget_args
		wget_out, wget_err = subprocess.Popen(
			self.wget_args, stdout=subprocess.PIPE, stderr=subprocess.PIPE
		).communicate()
		rospy.logdebug("Sent speech")
		return wget_out, wget_err


def is_connected():
	"""Check if connected to Internet"""
	try:
		# check if DNS can resolve hostname
		remote_host = socket.gethostbyname("www.google.com")
		# check if host is reachable
		s = socket.create_connection(address=(remote_host, 80), timeout=5)
		return True
	except:
		pass
	return False


def usage():
	"""Print Usage"""
	print("Usage:")
	print("rosrun gspeech gspeech.py <API_KEY>")


def main():
	if len(sys.argv) < 2:
		usage()
		sys.exit("No API_KEY provided")
	if not is_connected():
		sys.exit("No Internet connection available")
	api_key = str(sys.argv[1])
	threshold = sys.argv[2]
	speech = GSpeech(api_key, threshold)
	rospy.spin()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
	except KeyboardInterrupt:
		sys.exit(0)