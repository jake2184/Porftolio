// The executable for the LoRa ground receiver. 
// Received telemetry is logged (logs/log*.txt) and uploaded to Bluemix
// via logs/uploadTelemetryData.py

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "../lora.h"
#include "../misc.h"


struct TConfig Config;

int main(int argc, char **argv){

	initialiseLoRa();

	printLoRaSettings();
	ShowPacketCounts(0);
	ShowPacketCounts(1);

	constantReceiving();
}