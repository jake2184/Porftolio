# Uploads telemetry data to Bluemix, via IoT Foundation
# Regularly checks the LoRa log files (1 per module), uploads any new sentence

import time

import ibmiotf.device

def splitData(line):
	inData = line.split(",")
	outData = dict()
	outData['callsign'] = (inData[0])[2:]
	outData['seqno'] = inData[1]
	outData['pitime'] = inData[2]
	outData['latitude'] = inData[3]
	outData['longitude'] = inData[4]
	outData['altitude'] = inData[5]
	outData['speed'] = inData[6]
	outData['direction'] = inData[7]
	outData['satellites'] = inData[8]
	outData['gpsinternaltemp'] = inData[9]
	outData['batteryvoltage'] = inData[10]
	outData['humidity'] = inData[11]
	outData['x'] = inData[12]
	outData['y'] = inData[13]
	outData['z'] = inData[14]
	outData['light'] = inData[15]
	outData['infrared'] = inData[16]
	outData['pressure'] = inData[17]
	outData['temperature'] = (inData[18])[:5]
	return outData






if __name__ == "__main__":
	try:
		options = ibmiotf.device.ParseConfigFile("mqttconfig.txt")
		client = ibmiotf.device.Client(options)
	except ibmiotf.ConnectionException as e:
		ignore

	client.connect()

	lastLineRead1 = -1
	lastLineRead2 = -1

	log1 = True
	log2 = True

	try:
		logFile1 = open("log1.txt", 'r')
		logFile1.close()
	except IOError:
		print("LogFile1 Error. Does not exist?")
		log1 = False
	try:
		logFile2 = open("log2.txt", 'r')
		logFile2.close()
	except IOError:
		print("LogFile2 Error. Does not exist?")
		log2 = False

	while(1):
		if log1:
			logFile1 = open("log1.txt", 'r')
			for i, line in enumerate(logFile1):
				if(i > lastLineRead1):
					lastLineRead1 = i
					client.publishEvent("status", splitData(line))
			logFile1.close()
			
		if log2:
			logFile2 = open("log2.txt", 'r')
			for i, line in enumerate(logFile2):
				if(i > lastLineRead2):
					lastLineRead2 = i
					client.publishEvent("status", splitData(line))
			logFile2.close()

		time.sleep(2)