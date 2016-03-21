# script (outdated?) that collected tweets live. A tweet would occur, Bluemix
# would receive it, and publish to an MQTT topic that this script subscribes to.
# LoRa on the ground reads the files this script writes when sending tweets
# Written by Jonathon Zheng


import json
import ibmiotf.device
import time
import re

def myEventCallback(cmd):
	#try:
		payload = str(cmd.payload)
		tweet = re.search(r'\$\$TWEET.*\$\$ENDTWEET', payload)
		if(tweet):
			text = tweet.group()[7:-10]
			print(text)
			
			f = open("tweets.txt", 'r')
			lines = [text + "\n"]
			for i in range(0,9):
				lines.append(f.readline())
			f.close()
			f = open("tweetsToSend.txt", 'w')
			for i in range(0,10):
				f.write(lines[i].replace(" #bluemixballoon", "").replace("#bluemixballoon ", ""))
			f.close()
			
				
	#except Exception:
	
try:
	options = {
		"org": "obbs3n",
		"type": "pi",
		"id": "b827eba7c08e",
		"auth-method": "token",
		"auth-token": "QhYC@WZhChR80zI?DK"
	}
	client = ibmiotf.device.Client(options)
except ibmiotf.ConnectionException as e:
	print("ERROR :(", e)
	
client.connect()
client.commandCallback = myEventCallback

myData={"name" : "ICARUS"}
while(1):
	client.publishEvent("status", myData)
	time.sleep(300)


