#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <ctype.h>
#include <stdio.h>   	// Standard input/output definitions
#include <string.h>  	// String function definitions
#include <unistd.h>  	// UNIX standard function definitions
#include <fcntl.h>   	// File control definitions
#include <errno.h>   	// Error number definitions
#include <termios.h> 	// POSIX terminal control definitions
#include <stdint.h>
#include <stdlib.h>
#include <dirent.h>
#include <math.h>
#include <pthread.h>
#include <wiringPi.h>
#include <curses.h>
#include <stdlib.h>
#include <sys/time.h>
#include "gps.h"
#include "lora.h"
//#include "MQTTClient.h"
#include "sensors/sen.h"

#include <wiringPi.h>
#include <wiringPiSPI.h>


// RFM98
uint8_t currentMode = 0x81;
int PreviousImageNumber,  PreviousPacketNumber;
uint32_t PreviousCallsignCode;

struct TLoRaConfiguration LoraConfig;

#pragma pack(1)
struct TBinaryPacket
{
	uint8_t 	PayloadIDs;
	uint16_t	Counter;
	uint16_t	Seconds;
	uint16_t 	BiSeconds;
	float		Latitude;
	float		Longitude;
	uint16_t	Altitude;
};

int Records;

void sendTweet(int channel);

// Misc Functions

void channelLogMessage(int channel, int row, int column, const char *format, ...){
	if(!LoraConfig.ground){
		va_list args;
		va_start(args, format);	
		printf(format, args);
		va_end(args);
		return;
	}

	char Buffer[80];
	
    va_list args;
    va_start(args, format);

    vsprintf(Buffer, format, args);

    va_end(args);

	mvwaddstr(LoraConfig.LoRaDevices[channel].Window, row, column, Buffer);
	
	wrefresh(LoraConfig.LoRaDevices[channel].Window);
}

void readString(FILE *fp, char *keyword, char *Result, int length, int NeedValue){
	char line[100], *token, *value;
 
	fseek(fp, 0, SEEK_SET);
	*Result = '\0';

	while (fgets(line, sizeof(line), fp) != NULL)
	{
		
		token = strtok(line, "=");
		if (strcasecmp(keyword, token) == 0)
		{
			value = strtok(NULL, "\n");
			strcpy(Result, value);
			return;
		}
	}

	if (NeedValue)
	{
		LogMessage("Missing value for '%s' in LoraConfig file\n", keyword);
		exit(1);
	}
}

char * readStringss(FILE *fp, char *keyword, char *Result, int length, int NeedValue){
	char line[100], *token, *value;
 
	fseek(fp, 0, SEEK_SET);
	*Result = '\0';

	while (fgets(line, sizeof(line), fp) != NULL)
	{
		
		token = strtok(line, "=");
		if (strcasecmp(keyword, token) == 0)
		{
			value = strtok(NULL, "\n");
			strcpy(Result, value);
			return Result;
		}
	}

	if (NeedValue)
	{
		LogMessage("Missing value for '%s' in LoraConfig file\n", keyword);
		exit(1);
	}
}

int readInteger(FILE *fp, char *keyword, int NeedValue, int DefaultValue){
	char Temp[64];

	readString(fp, keyword, Temp, sizeof(Temp), NeedValue);

	if (Temp[0])
	{
		return atoi(Temp);
	}
	
	return DefaultValue;
}

int readBoolean(FILE *fp, char *keyword, int NeedValue, int *Result){
	char Temp[32];

	readString(fp, keyword, Temp, sizeof(Temp), NeedValue);
	if (*Temp)
	{
		*Result = (*Temp == '1') || (*Temp == 'Y') || (*Temp == 'y') || (*Temp == 't') || (*Temp == 'T');
		return (*Temp == '1') || (*Temp == 'Y') || (*Temp == 'y') || (*Temp == 't') || (*Temp == 'T');
	}
	//return *Temp;
	return 0;
	
}

void writeRegister(int channel, uint8_t reg, uint8_t val){
	unsigned char data[2];
	
	data[0] = reg | 0x80;
	data[1] = val;
	wiringPiSPIDataRW(channel, data, 2);
}

uint8_t readRegister(int channel, uint8_t reg){
	unsigned char data[2];
	uint8_t val;
	
	data[0] = reg & 0x7F;
	data[1] = 0;
	wiringPiSPIDataRW(channel, data, 2);
	val = data[1];

    return val;
}

double timeDiff(struct timeval timeA, struct timeval timeB){
    double   endTime = timeA.tv_sec + (timeA.tv_usec / 1000000.0);
    double startTime = timeB.tv_sec + (timeB.tv_usec / 1000000.0);
    return endTime - startTime;
}

void sendAck(int channel, int no){
	unsigned char message[256];
	int length;
	sprintf(message, "&&ACK&&");

	LogMessage("Sending %s \n", message);
	SendLoRaData(channel, message, 7);
	delay(1000);
	//SendLoRaData(channel, message, length);
	startReceiving(channel);
}

bool receiveAck(int channel, int no){
	LoraConfig.LoRaDevices[channel].ackReceived = false;
	receiveForSetTime(3.0);

	if(LoraConfig.LoRaDevices[channel].ackReceived){
		return true;
	} else {
		return false;
	}
}

void shaveCRC(unsigned char* message){
	int i;
	for(i = 0; i < 5; i++){
		message[strlen(message)-1] = '\0';
	}
}

char toHex(char Character){
	char HexTable[] = "0123456789ABCDEF";
	
	return HexTable[Character];
}

// Setup Functions // 

void setMode(int channel, uint8_t newMode){
  if(newMode == currentMode)
    return;  
  
  switch (newMode) 
  {
    case RF98_MODE_TX:
      writeRegister(channel, REG_LNA, LNA_OFF_GAIN);  // TURN LNA OFF FOR TRANSMITT
      writeRegister(channel, REG_PA_CONFIG, LoraConfig.LoRaDevices[channel].Power);
      writeRegister(channel, REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
    case RF98_MODE_RX_CONTINUOUS:
      writeRegister(channel, REG_PA_CONFIG, PA_OFF_BOOST);  // TURN PA OFF FOR RECIEVE??
      writeRegister(channel, REG_LNA, LNA_MAX_GAIN);  // LNA_MAX_GAIN);  // MAX GAIN FOR RECIEVE
      writeRegister(channel, REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
    case RF98_MODE_SLEEP:
      writeRegister(channel, REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
    case RF98_MODE_STANDBY:
      writeRegister(channel, REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
    default: return;
  } 
  
	/*if(newMode != RF98_MODE_SLEEP && newMode != RF98_MODE_TX)
	{
		LogMessage("Waiting for Mode Change\n");
		while(digitalRead(LoraConfig.LoRaDevices[channel].DIO5) == 0)
		{
		} 
		// LogMessage("Mode change completed\n");
	}*/
	
	return;
}
 
void setLoRaMode(int channel){
	double Frequency;
	unsigned long FrequencyValue;

	LogMessage("Setting LoRa Mode\n");
	setMode(channel, RF98_MODE_SLEEP);
	writeRegister(channel, REG_OPMODE, 0x80);

	setMode(channel, RF98_MODE_SLEEP);
  
	if (sscanf(LoraConfig.LoRaDevices[channel].Frequency, "%lf", &Frequency))
	{
		FrequencyValue = (unsigned long)(Frequency * 7110656 / 434);
		LogMessage("FrequencyValue = %06Xh\n", FrequencyValue);
		writeRegister(channel, 0x06, (FrequencyValue >> 16) & 0xFF);		// Set frequency
		writeRegister(channel, 0x07, (FrequencyValue >> 8) & 0xFF);
		writeRegister(channel, 0x08, FrequencyValue & 0xFF);
	}
    setMode(channel, RF98_MODE_SLEEP);
    delay(1000);
	LogMessage("Mode = %x\n", readRegister(channel, REG_OPMODE));
}

void setupRFM98(int channel){
	if (LoraConfig.LoRaDevices[channel].InUse)
	{
		// initialize the pins
		LogMessage("channel %d DIO0=%d DIO5=%d\n", channel, LoraConfig.LoRaDevices[channel].DIO0, LoraConfig.LoRaDevices[channel].DIO5);
		pinMode(LoraConfig.LoRaDevices[channel].DIO0, INPUT);
		pinMode(LoraConfig.LoRaDevices[channel].DIO5, INPUT);

		if (wiringPiSPISetup(channel, 500000) < 0)
		{
			fprintf(stderr, "Failed to open SPI port.  Try loading spi library with 'gpio load spi'");
			exit(1);
		}

		// LoRa mode 
		setLoRaMode(channel);

		writeRegister(channel, REG_MODEM_CONFIG, LoraConfig.LoRaDevices[channel].ImplicitOrExplicit | LoraConfig.LoRaDevices[channel].ErrorCoding | LoraConfig.LoRaDevices[channel].Bandwidth);
		writeRegister(channel, REG_MODEM_CONFIG2, LoraConfig.LoRaDevices[channel].SpreadingFactor | CRC_ON);
		writeRegister(channel, REG_MODEM_CONFIG3, 0x04 | LoraConfig.LoRaDevices[channel].LowDataRateOptimize);									// 0x04: AGC sets LNA gain
		writeRegister(channel, REG_DETECT_OPT, (readRegister(channel, REG_DETECT_OPT) & 0xF8) | ((LoraConfig.LoRaDevices[channel].SpreadingFactor == SPREADING_6) ? 0x05 : 0x03));	// 0x05 For SF6; 0x03 otherwise
		writeRegister(channel, REG_DETECTION_THRESHOLD, (LoraConfig.LoRaDevices[channel].SpreadingFactor == SPREADING_6) ? 0x0C : 0x0A);		// 0x0C for SF6, 0x0A otherwise
		
		writeRegister(channel, REG_PAYLOAD_LENGTH, LoraConfig.LoRaDevices[channel].PayloadLength);
		writeRegister(channel, REG_RX_NB_BYTES, LoraConfig.LoRaDevices[channel].PayloadLength);

		writeRegister(channel, REG_FIFO_ADDR_PTR, 0);		// woz readRegister(channel, REG_FIFO_RX_BASE_AD));   

		// writeRegister(channel, REG_DIO_MAPPING_1,0x40);
		writeRegister(channel, REG_DIO_MAPPING_2,0x00);
		
		//setMode(channel, RF98_MODE_STANDBY);
		setMode(channel, RF98_MODE_RX_CONTINUOUS);
		LogMessage("Channel %d  mode %x \n", channel, readRegister(channel, REG_OPMODE));
	}
}

/*
void connectToMQTT(){
	int rc;

	LogMessage("Connecting to MQTT Broker\n");
	LogMessage("%s %s %s %s\n", LoraConfig.MQTTSettings.address, 
		LoraConfig.MQTTSettings.clientID, LoraConfig.MQTTSettings.username, LoraConfig.MQTTSettings.password);

	MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;

	MQTTClient_create(&LoraConfig.MQTTSettings.client, LoraConfig.MQTTSettings.address, LoraConfig.MQTTSettings.clientID,
        MQTTCLIENT_PERSISTENCE_NONE, NULL);

	conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;
    conn_opts.username = LoraConfig.MQTTSettings.username;
    conn_opts.password = LoraConfig.MQTTSettings.password;

    if ((rc = MQTTClient_connect(LoraConfig.MQTTSettings.client, &conn_opts)) != MQTTCLIENT_SUCCESS)
    {
    	LoraConfig.MQTTSettings.upload = false;
        printf("Failed to connect, return code %d\n", rc);
    } else {
    	LoraConfig.MQTTSettings.upload = true;
    }
}
*/

void loadConfigFile(){
	FILE *fp;
	char *filename = "config.txt";
	char Keyword[32];
	int channel, Temp;
	char TempString[16];

	LoraConfig.EnableHabitat = 1;
	LoraConfig.EnableSSDV = 1;
	LoraConfig.ftpServer[0] = '\0';
	LoraConfig.ftpUser[0] = '\0';
	LoraConfig.ftpPassword[0] = '\0';
	LoraConfig.ftpFolder[0] = '\0';
	LoraConfig.LastCommandNo = 0;

	LoraConfig.LoRaDevices[0].DIO0 = 6;
	LoraConfig.LoRaDevices[0].DIO5 = 5;
		
	LoraConfig.LoRaDevices[1].DIO0 = 27;
	LoraConfig.LoRaDevices[1].DIO5 = 26;

	if ((fp = fopen(filename, "r")) == NULL)
	{
		LogMessage("\nFailed to open config file %s (error %d - %s).\nPlease check that it exists and has read permission.\n", filename, errno, strerror(errno));
		exit(1);
	}
	LoraConfig.ground = readBoolean(fp, "ground", 0, &LoraConfig.ground);
	
	strcpy(LoraConfig.Tracker, readStringss(fp, "tracker", LoraConfig.Tracker, sizeof(LoraConfig.Tracker), 1));
	
	readString(fp, "tracker", LoraConfig.Tracker, sizeof(LoraConfig.Tracker), 1);	 
	
	readString(fp, "tracker", LoraConfig.channels[0].PayloadID, sizeof(LoraConfig.Tracker), 1);
	readString(fp, "tracker", LoraConfig.channels[1].PayloadID, sizeof(LoraConfig.Tracker), 1);
	
	readBoolean(fp, "EnableHabitat", 0, &LoraConfig.EnableHabitat);
	readBoolean(fp, "EnableSSDV", 0, &LoraConfig.EnableSSDV);
		

	readString(fp, "ftpserver", LoraConfig.ftpServer, sizeof(LoraConfig.ftpServer), 0);
	readString(fp, "ftpUser", LoraConfig.ftpUser, sizeof(LoraConfig.ftpUser), 0);
	readString(fp, "ftpPassword", LoraConfig.ftpPassword, sizeof(LoraConfig.ftpPassword), 0);
	readString(fp, "ftpFolder", LoraConfig.ftpFolder, sizeof(LoraConfig.ftpFolder), 0);	

	LoraConfig.MQTTSettings.address[0] = '\0';
	readString(fp, "MQTTAddress", LoraConfig.MQTTSettings.address, sizeof(LoraConfig.MQTTSettings.address), 0);	
	readString(fp, "MQTTClientID", LoraConfig.MQTTSettings.clientID, sizeof(LoraConfig.MQTTSettings.clientID), 0);	
	readString(fp, "MQTTUsername", LoraConfig.MQTTSettings.username, sizeof(LoraConfig.MQTTSettings.username), 0);	
	readString(fp, "MQTTPassword", LoraConfig.MQTTSettings.password, sizeof(LoraConfig.MQTTSettings.password), 0);
	readString(fp, "MQTTTopic", LoraConfig.MQTTSettings.topic, sizeof(LoraConfig.MQTTSettings.topic), 0);	

	if(LoraConfig.MQTTSettings.address[0]){
		//connectToMQTT();
	}

	for (channel=0; channel<=1; channel++)
	{
		// Defaults
		LoraConfig.LoRaDevices[channel].Frequency[0] = '\0';
		
		sprintf(Keyword, "frequency_%d", channel);
		readString(fp, Keyword, LoraConfig.LoRaDevices[channel].Frequency, sizeof(LoraConfig.LoRaDevices[channel].Frequency), 0);
		if (LoraConfig.LoRaDevices[channel].Frequency[0])
		{
			LoraConfig.LoRaDevices[channel].ImplicitOrExplicit = EXPLICIT_MODE;
			LoraConfig.LoRaDevices[channel].ErrorCoding = ERROR_CODING_4_8;
			LoraConfig.LoRaDevices[channel].Bandwidth = BANDWIDTH_20K8;
			LoraConfig.LoRaDevices[channel].SpreadingFactor = SPREADING_11;
			LoraConfig.LoRaDevices[channel].LowDataRateOptimize = 0x00;		
			
			LogMessage("channel %d frequency set to %s\n", channel, LoraConfig.LoRaDevices[channel].Frequency);
			LoraConfig.LoRaDevices[channel].InUse = 1;

			// DIO0 / DIO5 overrides
			sprintf(Keyword, "DIO0_%d", channel);
			LoraConfig.LoRaDevices[channel].DIO0 = readInteger(fp, Keyword, 0, LoraConfig.LoRaDevices[channel].DIO0);

			sprintf(Keyword, "DIO5_%d", channel);
			LoraConfig.LoRaDevices[channel].DIO5 = readInteger(fp, Keyword, 0, LoraConfig.LoRaDevices[channel].DIO5);

			LogMessage("LoRa channel %d DIO0=%d DIO5=%d\n", channel, LoraConfig.LoRaDevices[channel].DIO0, LoraConfig.LoRaDevices[channel].DIO5);
			
			LoraConfig.LoRaDevices[channel].SpeedMode = 0;
			sprintf(Keyword, "mode_%d", channel);
			LoraConfig.LoRaDevices[channel].SpeedMode = readInteger(fp, Keyword, 0, 0);
			//channelLogMessage(channel, 1, 1, "channel %d %sMHz %d mode", channel, LoraConfig.LoRaDevices[channel].Frequency, LoraConfig.LoRaDevices[channel].SpeedMode); 

			if (LoraConfig.LoRaDevices[channel].SpeedMode == 4)
			{
				// Testing
				LoraConfig.LoRaDevices[channel].ImplicitOrExplicit = IMPLICIT_MODE;
				LoraConfig.LoRaDevices[channel].ErrorCoding = ERROR_CODING_4_5;
				LoraConfig.LoRaDevices[channel].Bandwidth = BANDWIDTH_250K;
				LoraConfig.LoRaDevices[channel].SpreadingFactor = SPREADING_6;
				LoraConfig.LoRaDevices[channel].LowDataRateOptimize = 0;		
			}
			else if (LoraConfig.LoRaDevices[channel].SpeedMode == 3)
			{
				// Normal mode for high speed images in 868MHz band
				LoraConfig.LoRaDevices[channel].ImplicitOrExplicit = EXPLICIT_MODE;
				LoraConfig.LoRaDevices[channel].ErrorCoding = ERROR_CODING_4_6;
				LoraConfig.LoRaDevices[channel].Bandwidth = BANDWIDTH_250K;
				LoraConfig.LoRaDevices[channel].SpreadingFactor = SPREADING_7;
				LoraConfig.LoRaDevices[channel].LowDataRateOptimize = 0;		
			}
			else if (LoraConfig.LoRaDevices[channel].SpeedMode == 2)
			{
				// Normal mode for repeater network
				LoraConfig.LoRaDevices[channel].ImplicitOrExplicit = EXPLICIT_MODE;
				LoraConfig.LoRaDevices[channel].ErrorCoding = ERROR_CODING_4_8;
				LoraConfig.LoRaDevices[channel].Bandwidth = BANDWIDTH_62K5;
				LoraConfig.LoRaDevices[channel].SpreadingFactor = SPREADING_8;
				LoraConfig.LoRaDevices[channel].LowDataRateOptimize = 0x00;		
			}
			else if (LoraConfig.LoRaDevices[channel].SpeedMode == 1)
			{
				// Normal mode for SSDV
				LoraConfig.LoRaDevices[channel].ImplicitOrExplicit = IMPLICIT_MODE;
				LoraConfig.LoRaDevices[channel].ErrorCoding = ERROR_CODING_4_5;
				LoraConfig.LoRaDevices[channel].Bandwidth = BANDWIDTH_20K8;
				LoraConfig.LoRaDevices[channel].SpreadingFactor = SPREADING_6;
				LoraConfig.LoRaDevices[channel].LowDataRateOptimize = 0;
			}
			else
			{
				LoraConfig.LoRaDevices[channel].ImplicitOrExplicit = EXPLICIT_MODE;
				LoraConfig.LoRaDevices[channel].ErrorCoding = ERROR_CODING_4_8;
				LoraConfig.LoRaDevices[channel].Bandwidth = BANDWIDTH_20K8;
				LoraConfig.LoRaDevices[channel].SpreadingFactor = SPREADING_11;
				LoraConfig.LoRaDevices[channel].LowDataRateOptimize = 0x08;		
			}
			sprintf(Keyword, "sf_%d", channel);
			Temp = readInteger(fp, Keyword, 0, 0);
			if ((Temp >= 6) && (Temp <= 12))
			{
				LoraConfig.LoRaDevices[channel].SpreadingFactor = Temp << 4;
				LogMessage("Setting SF=%d\n", Temp);
			}

			sprintf(Keyword, "bandwidth_%d", channel);
			readString(fp, Keyword, TempString, sizeof(TempString), 0);
			if (*TempString)
			{
				LogMessage("Setting BW=%s\n", TempString);
			}
			if (strcmp(TempString, "7K8") == 0)
			{
				LoraConfig.LoRaDevices[channel].Bandwidth = BANDWIDTH_7K8;
			}
			else if (strcmp(TempString, "10K4") == 0)
			{
				LoraConfig.LoRaDevices[channel].Bandwidth = BANDWIDTH_10K4;
			}
			else if (strcmp(TempString, "15K6") == 0)
			{
				LoraConfig.LoRaDevices[channel].Bandwidth = BANDWIDTH_15K6;
			}
			else if (strcmp(TempString, "20K8") == 0)
			{
				LoraConfig.LoRaDevices[channel].Bandwidth = BANDWIDTH_20K8;
			}
			else if (strcmp(TempString, "31K25") == 0)
			{
				LoraConfig.LoRaDevices[channel].Bandwidth = BANDWIDTH_31K25;
			}
			else if (strcmp(TempString, "41K7") == 0)
			{
				LoraConfig.LoRaDevices[channel].Bandwidth = BANDWIDTH_41K7;
			}
			else if (strcmp(TempString, "62K5") == 0)
			{
				LoraConfig.LoRaDevices[channel].Bandwidth = BANDWIDTH_62K5;
			}
			else if (strcmp(TempString, "125K") == 0)
			{
				LoraConfig.LoRaDevices[channel].Bandwidth = BANDWIDTH_125K;
			}
			else if (strcmp(TempString, "250K") == 0)
			{
				LoraConfig.LoRaDevices[channel].Bandwidth = BANDWIDTH_250K;
			}
			else if (strcmp(TempString, "500K") == 0)
			{
				LoraConfig.LoRaDevices[channel].Bandwidth = BANDWIDTH_500K;
			}

			switch (LoraConfig.LoRaDevices[channel].Bandwidth)
			{
				case  BANDWIDTH_7K8:	LoraConfig.LoRaDevices[channel].Reference = 7800; break;
				case  BANDWIDTH_10K4:   LoraConfig.LoRaDevices[channel].Reference = 10400; break;
				case  BANDWIDTH_15K6:   LoraConfig.LoRaDevices[channel].Reference = 15600; break;
				case  BANDWIDTH_20K8:   LoraConfig.LoRaDevices[channel].Reference = 20800; break;
				case  BANDWIDTH_31K25:  LoraConfig.LoRaDevices[channel].Reference = 31250; break;
				case  BANDWIDTH_41K7:   LoraConfig.LoRaDevices[channel].Reference = 41700; break;
				case  BANDWIDTH_62K5:   LoraConfig.LoRaDevices[channel].Reference = 62500; break;
				case  BANDWIDTH_125K:   LoraConfig.LoRaDevices[channel].Reference = 125000; break;
				case  BANDWIDTH_250K:   LoraConfig.LoRaDevices[channel].Reference = 250000; break;
				case  BANDWIDTH_500K:   LoraConfig.LoRaDevices[channel].Reference = 500000; break;
			}
			sprintf(Keyword, "implicit_%d", channel);
			if (readBoolean(fp, Keyword, 0, &Temp))
			{
				if (Temp)
				{
					LoraConfig.LoRaDevices[channel].ImplicitOrExplicit = IMPLICIT_MODE;
				}
			}
			
			sprintf(Keyword, "coding_%d", channel);
			Temp = readInteger(fp, Keyword, 0, 0);
			if ((Temp >= 5) && (Temp <= 8))
			{
				LoraConfig.LoRaDevices[channel].ErrorCoding = (Temp-4) << 1;
				LogMessage("Setting Error Coding=%d\n", Temp);
			}

			sprintf(Keyword, "lowopt_%d", channel);
			if (readBoolean(fp, Keyword, 0, &Temp))
			{
				if (Temp)
				{
					LoraConfig.LoRaDevices[channel].LowDataRateOptimize = 0x08;
				}
			}
			LoraConfig.LoRaDevices[channel].logging[0] = '\0';
			sprintf(Keyword, "logging_%d", channel);
			readString(fp, Keyword, TempString, sizeof(TempString), 0);
			if(*TempString){
				strncpy(LoraConfig.LoRaDevices[channel].logging, TempString, 16);
			}
		}
	}
	fclose(fp);
}

void setupSensors(){
	initialise_sensors();
}

void initialiseLoRa(){
	
	if (wiringPiSetup() == -1)
	{
		exit (1);
	}
	
	system("gpio load spi");
	
	loadConfigFile();
	
	setupRFM98(0);
	setupRFM98(1);
	
}

// Display Functions //

void ShowPacketCounts(int channel){
	if(!LoraConfig.ground) return;


	if (LoraConfig.LoRaDevices[channel].InUse)
	{
		channelLogMessage(channel, 6, 1, "Telem Packets = %d", LoraConfig.LoRaDevices[channel].TelemetryCount);
		channelLogMessage(channel, 7, 1, "Image Packets = %d", LoraConfig.LoRaDevices[channel].SSDVCount);
		channelLogMessage(channel, 8, 1, "Bad CRC = %d Bad Type = %d", LoraConfig.LoRaDevices[channel].BadCRCCount, LoraConfig.LoRaDevices[channel].UnknownCount);
	}
}

WINDOW * InitDisplay(void){
    WINDOW * mainwin;
	int channel;

    /*  Initialize ncurses  */

    if ( (mainwin = initscr()) == NULL ) {
	fprintf(stderr, "Error initialising ncurses.\n");
	exit(EXIT_FAILURE);
    }

    start_color();                    /*  Initialize colours  */

	init_pair(1, COLOR_WHITE, COLOR_BLUE);
	init_pair(2, COLOR_YELLOW, COLOR_BLUE);

	color_set(1, NULL);
	// bkgd(COLOR_PAIR(1));
	// attrset(COLOR_PAIR(1) | A_BOLD);

	// Title bar
    mvaddstr(0, 19, " LoRa Habitat and SSDV Gateway by daveake ");
    refresh();

	// Windows for LoRa live data
	for (channel=0; channel<=1; channel++)
	{
		LoraConfig.LoRaDevices[channel].Window = newwin(14, 38, 1, channel ? 41 : 1);
		wbkgd(LoraConfig.LoRaDevices[channel].Window, COLOR_PAIR(2));
		
		// wcolor_set(LoraConfig.LoRaDevices[channel].Window, 2, NULL);
		// waddstr(LoraConfig.LoRaDevices[channel].Window, "WINDOW");
		// mvwaddstr(LoraConfig.LoRaDevices[channel].Window, 0, 0, "Window");
		wrefresh(LoraConfig.LoRaDevices[channel].Window);
	}
	
	curs_set(0);
		   
	return mainwin;
}	

void CloseDisplay(WINDOW * mainwin){
    /*  Clean up after ourselves  */
    delwin(mainwin);
    endwin();
    refresh();
}

void printLoRaSettings(){
	int channel;
	LogMessage("InUse\tDIO0\tDIO5\tSpeedMode\tFrequency\n");
	for(channel =0; channel <=1; channel++){
		LogMessage("%d\t%d\t%d\t%d\t\t%s\n", 
				LoraConfig.LoRaDevices[channel].InUse,
				LoraConfig.LoRaDevices[channel].DIO0,
				LoraConfig.LoRaDevices[channel].DIO5,
				LoraConfig.LoRaDevices[channel].SpeedMode,
				LoraConfig.LoRaDevices[channel].Frequency
				
				);
	
	}

}

void LogMessage(const char *format, ...){
	if(LoraConfig.ground){	
		static WINDOW *Window=NULL;
		char Buffer[200];
		
		if (Window == NULL)
		{
			// Window = newwin(25, 30, 0, 50);
			Window = newwin(30, 80, 16, 0);
			scrollok(Window, TRUE);		
		}
		
		va_list args;
		va_start(args, format);

		vsprintf(Buffer, format, args);

		va_end(args);

		waddstr(Window, Buffer);
		
		wrefresh(Window);
	} else {
		char Buffer[200];
		va_list args;
		va_start(args, format);
		vsprintf(Buffer, format, args);
		va_end(args);
		printf("%s", Buffer);
	}
}



// Building Packets

int BuildLoRaSentence(char *TxLine, int channel, struct TGPS *GPS){
    int Count, i, j;
    unsigned char c;
    unsigned int CRC, xPolynomial;
	char TimeBuffer1[12], TimeBuffer2[10], ExtraFields[80];
	
	LoraConfig.channels[channel].SentenceCounter++;
	
	sprintf(TimeBuffer1, "%06ld", GPS->Time);
	TimeBuffer2[0] = TimeBuffer1[0];
	TimeBuffer2[1] = TimeBuffer1[1];
	TimeBuffer2[2] = ':';
	TimeBuffer2[3] = TimeBuffer1[2];
	TimeBuffer2[4] = TimeBuffer1[3];
	TimeBuffer2[5] = ':';
	TimeBuffer2[6] = TimeBuffer1[4];
	TimeBuffer2[7] = TimeBuffer1[5];
	TimeBuffer2[8] = '\0';
	
	ExtraFields[0] = '\0';
	/*
	if (LoraConfig.EnableBMP085)
	{
		sprintf(ExtraFields, ",%.1f,%.0f", GPS->ExternalTemperature, GPS->Pressure);
	}
	*/
    sprintf(TxLine, "$$%s,%d,%s,%7.5lf,%7.5lf,%5.5u,%d,%d,%d,%d,%d,%s",
            LoraConfig.channels[channel].PayloadID,
            LoraConfig.channels[channel].SentenceCounter,
			TimeBuffer2,
            GPS->Latitude,
            GPS->Longitude,
            GPS->Altitude,
			(GPS->Speed * 13) / 7,
			GPS->Direction,
			GPS->Satellites,
			LoraConfig.LoRaDevices[channel].GroundCount,
			LoraConfig.LoRaDevices[channel].AirCount,
			LoraConfig.LoRaDevices[channel].LastCommand);

    Count = strlen(TxLine);

    CRC = 0xffff;           // Seed
    xPolynomial = 0x1021;
   
     for (i = 2; i < Count; i++)
     {   // For speed, repeat calculation instead of looping for each bit
        CRC ^= (((unsigned int)TxLine[i]) << 8);
        for (j=0; j<8; j++)
        {
            if (CRC & 0x8000)
                CRC = (CRC << 1) ^ 0x1021;
            else
                CRC <<= 1;
        }
     }

    TxLine[Count++] = '*';
    TxLine[Count++] = toHex((CRC >> 12) & 15);
    TxLine[Count++] = toHex((CRC >> 8) & 15);
    TxLine[Count++] = toHex((CRC >> 4) & 15);
    TxLine[Count++] = toHex(CRC & 15);
	TxLine[Count++] = '\n';  
	TxLine[Count++] = '\0';
	
	return strlen(TxLine) + 1;
}

int BuildLoRaPacket(char *TxLine, int channel, struct TGPS *GPS){
	int Count, i, j;
    unsigned char c;
    unsigned int CRC, xPolynomial;
	char TimeBuffer1[12], TimeBuffer2[10], ExtraFields[80];
	
	LoraConfig.channels[channel].SentenceCounter++;
	
	sprintf(TimeBuffer1, "%06ld", GPS->Time);
	TimeBuffer2[0] = TimeBuffer1[0];
	TimeBuffer2[1] = TimeBuffer1[1];
	TimeBuffer2[2] = ':';
	TimeBuffer2[3] = TimeBuffer1[2];
	TimeBuffer2[4] = TimeBuffer1[3];
	TimeBuffer2[5] = ':';
	TimeBuffer2[6] = TimeBuffer1[4];
	TimeBuffer2[7] = TimeBuffer1[5];
	TimeBuffer2[8] = '\0';
	
	ExtraFields[0] = '\0';
	/*
	if (LoraConfig.EnableBMP085)
	{
		sprintf(ExtraFields, ",%.1f,%.0f", GPS->ExternalTemperature, GPS->Pressure);
	}
*/
	
	sprintf(TxLine, "$$%s,%d,%s,%7.5lf,%7.5lf,%5.5u,%d,%d,%d,%2.2f,%2.2f,%.3f,%.3f,%.3f,%.3f,%2.2f,%2.2f,%d,%2.2f",
            LoraConfig.channels[channel].PayloadID, //%s
            LoraConfig.channels[channel].SentenceCounter, //%d
			TimeBuffer2, //%s
            GPS->Latitude, //%7.5lf
            GPS->Longitude, //%7.5lf
            GPS->Altitude, //%5.5u
			(GPS->Speed * 13) / 7, //%d
			GPS->Direction, //%d
			GPS->Satellites,   //%d         
            GPS->InternalTemperature, //%2.2f
            GPS->BatteryVoltage, //%2.2f
			sensors.humidity, //%.3f
			sensors.x,//%.3f
			sensors.y, //%.3f
			sensors.z, //%.3f
			sensors.light, //%2.2f
			sensors.infrared, //%2.2f
			sensors.pressure, //%d
			sensors.temp //%2.2f
			);
	
    Count = strlen(TxLine);
    CRC = 0xffff;           // Seed
    xPolynomial = 0x1021;
   
     for (i = 2; i < Count; i++)
     {   // For speed, repeat calculation instead of looping for each bit
        CRC ^= (((unsigned int)TxLine[i]) << 8);
        for (j=0; j<8; j++)
        {
            if (CRC & 0x8000)
                CRC = (CRC << 1) ^ 0x1021;
            else
                CRC <<= 1;
        }
     }

    TxLine[Count++] = '*';
    TxLine[Count++] = toHex((CRC >> 12) & 15);
    TxLine[Count++] = toHex((CRC >> 8) & 15);
    TxLine[Count++] = toHex((CRC >> 4) & 15);
    TxLine[Count++] = toHex(CRC & 15);
	//TxLine[Count++] = '\n';  
	TxLine[Count++] = '\0';
	return strlen(TxLine) + 1;
}

int BuildLoRaString(char* TxLine, char* message, int channel){
	int Count, i, j;
    unsigned char c;
    unsigned int CRC, xPolynomial;
	char ExtraFields[80];
	
	LoraConfig.channels[channel].SentenceCounter++;
	
	ExtraFields[0] = '\0';
	
	memcpy(TxLine, message, strlen(message));

    Count = strlen(message);

    CRC = 0xffff;           // Seed
    xPolynomial = 0x1021;
   
     for (i = 2; i < Count; i++)
     {   // For speed, repeat calculation instead of looping for each bit
        CRC ^= (((unsigned int)TxLine[i]) << 8);
        for (j=0; j<8; j++)
        {
            if (CRC & 0x8000)
                CRC = (CRC << 1) ^ 0x1021;
            else
                CRC <<= 1;
        }
     }

    TxLine[Count++] = '*';
    TxLine[Count++] = toHex((CRC >> 12) & 15);
    TxLine[Count++] = toHex((CRC >> 8) & 15);
    TxLine[Count++] = toHex((CRC >> 4) & 15);
    TxLine[Count++] = toHex(CRC & 15);
	//TxLine[Count++] = '\n';  
	TxLine[Count++] = '\0';
	
	return strlen(TxLine) + 1;
}

int BuildLoRaPositionPacket(char *TxLine, int channel, struct TGPS *GPS){
	int OurID;
	struct TBinaryPacket BinaryPacket;
	
	OurID = LoraConfig.LoRaDevices[channel].Slot;
	
	LoraConfig.channels[channel].SentenceCounter++;

	BinaryPacket.PayloadIDs = 0xC0 | (OurID << 3) | OurID;
	BinaryPacket.Counter = LoraConfig.channels[channel].SentenceCounter;
	BinaryPacket.Seconds = GPS->Seconds;
	BinaryPacket.Latitude = GPS->Latitude;
	BinaryPacket.Longitude = GPS->Longitude;
	BinaryPacket.Altitude = GPS->Altitude;

	memcpy(TxLine, &BinaryPacket, sizeof(BinaryPacket));
	
	return sizeof(struct TBinaryPacket);
}



// Processing Packets //

void processTelemetrymessage(int channel, unsigned char* data, int length){
	
	// If logging, write to file 
	if(LoraConfig.LoRaDevices[channel].logging[0] != '\0'){
		FILE* fp;
		if(fp = fopen(LoraConfig.LoRaDevices[channel].logging, "a")){
			fwrite(data, 1, length, fp);
			fwrite("\n", 1, 1, fp);
			fclose(fp);
			LogMessage("Written to file\n");
		} else {
			LogMessage("Error opening log file %s\n", LoraConfig.LoRaDevices[channel].logging);
		}
	} else {
	    LogMessage("Not Logging. \n");
	}
	/*
	// If uploading, upload to mqtt broker
	if(LoraConfig.MQTTSettings.upload){
		MQTTClient_deliveryToken token;
		int rc;		
		MQTTClient_message message = MQTTClient_message_initializer;
		message.payload = data;
		message.payloadlen = strlen(message.payload);
		message.qos = 0;
		message.retained = 0;
		rc = MQTTClient_publishMessage(LoraConfig.MQTTSettings.client, LoraConfig.MQTTSettings.topic, &message, &token);
		LogMessage("Error code %d \n", rc); 
		rc = MQTTClient_waitForCompletion(LoraConfig.MQTTSettings.client, token, 10000);
    	LogMessage("Message with delivery token %d delivered, %d\n", token, rc);
	}
    */

	// Update LoRaDevices info
	//ProcessLine(channel, data);
	//DoPositionCalcs(channel);							
	LoraConfig.LoRaDevices[channel].TelemetryCount++;

	// Save and send data?

	// Print message
	//data[strlen(data+1)] = '\0';
	//LogMessage("Ch %d: %s\n", channel, data);
}

void processSSDVPacket(int channel, unsigned char *Data, int length){

	int ImageNumber, PacketNumber;
	uint32_t CallsignCode;

	// SSDV packet
	char Callsign[7], *FileMode, filename[100];
	FILE* fp;
	int output_length;
	
	
	Data[0] = 0x55;
	
	CallsignCode = Data[2]; CallsignCode <<= 8;
	CallsignCode |= Data[3]; CallsignCode <<= 8;
	CallsignCode |= Data[4]; CallsignCode <<= 8;
	CallsignCode |= Data[5];
	
	//decode_callsign(Callsign, CallsignCode);
								
	ImageNumber = Data[6];
	PacketNumber = Data[8];

	// Create new file ?
	if ((ImageNumber != PreviousImageNumber) || (PacketNumber <= PreviousPacketNumber) || (CallsignCode != PreviousCallsignCode))
	{
		LogMessage("Cause: %d %d %d     ",(ImageNumber != PreviousImageNumber),(PacketNumber <= PreviousPacketNumber),(CallsignCode != PreviousCallsignCode));
		LogMessage("%d %d %d %d \n", ImageNumber, PreviousImageNumber, CallsignCode, PreviousCallsignCode);
		// New image so new file
		// FileMode = "wb";
		FileMode = "ab";
		LoraConfig.LoRaDevices[channel].SSDVMissing = PacketNumber;
	}
	else
	{
		FileMode = "ab";
		if (PacketNumber > (PreviousPacketNumber+1))
		{
			LoraConfig.LoRaDevices[channel].SSDVMissing += PacketNumber - PreviousPacketNumber - 1;
		}
	}

	LogMessage("SSDV Packet, Callsign %s, Image %d, Packet %d, %d Missing\n", Callsign, Data[6], Data[7] * 256 + Data[8], LoraConfig.LoRaDevices[channel].SSDVMissing);
	channelLogMessage(channel, 3, 1, "SSDV Packet %d bytes  ", length);
	
	PreviousImageNumber = ImageNumber;
	PreviousPacketNumber = PacketNumber;
	PreviousCallsignCode = CallsignCode;

	// Save to file
	
	sprintf(filename, "/tmp/%s_%d.bin", Callsign, ImageNumber);

	if (fp = fopen(filename, FileMode))
	{
		fwrite(Data, 1, 256, fp); 
		fclose(fp);
	}

	LoraConfig.LoRaDevices[channel].SSDVCount++;
}


void processBinaryTelemetryPacket(int channel, unsigned char* message, int length){
/*
	struct TBinaryPacket BinaryPacket;
	char Data[100], Sentence[100];
	int SourceID, SenderID;
	
	SourceID = message[1] & 0x07;
	SenderID = (message[1] >> 3) & 0x07;

	channelLogMessage(channel, 3, 1, "Binary Telemetry");

	memcpy(&BinaryPacket, message+1, sizeof(BinaryPacket));
	
	strcpy(LoraConfig.LoRaDevices[channel].Payload, "Binary");
	LoraConfig.LoRaDevices[channel].Seconds = (unsigned long) BinaryPacket.BiSeconds * 2L;
	LoraConfig.LoRaDevices[channel].Counter = BinaryPacket.Counter;
	LoraConfig.LoRaDevices[channel].Latitude = BinaryPacket.Latitude;
	LoraConfig.LoRaDevices[channel].Longitude = BinaryPacket.Longitude;
	LoraConfig.LoRaDevices[channel].Altitude = BinaryPacket.Altitude;

	// Update LoRa Devices info
	//DoPositionCalcs(channel);	
	LoraConfig.LoRaDevices[channel].TelemetryCount++;

	// Print message
	LogMessage("Ch %d: Sender %d Source %d (%s) Position %8.5lf, %8.5lf, %05u\n",
		channel,
		SenderID,
		SourceID,
		Payloads[SourceID].Payload,
		LoraConfig.LoRaDevices[channel].Latitude,
		LoraConfig.LoRaDevices[channel].Longitude,
		LoraConfig.LoRaDevices[channel].Altitude);
		*/
}


void processCommandLine(int channel, unsigned char* CommandLine, int length){
	
	if(LoraConfig.LastCommandNo != (int)CommandLine[2] - '0'){
		LoraConfig.LastCommandNo = (int)CommandLine[2] - '0';
		LogMessage("System: %s \n", CommandLine+4);
		system(CommandLine+4);
		if(CommandLine[4] == 'p' && CommandLine[5] == 'y'){
			LogMessage("Got Tweet\n");
			LoraConfig.LoRaDevices[channel].tweetReceived = true;
		}
		
	}
	sendAck(channel,LoraConfig.LastCommandNo);
}

void processRequest(int channel, unsigned char* request, int length){
	shaveCRC(request);
	if(!strcmp(request, "==command==")){
		readAndSendCommandLine(channel, "commands.txt", 4);
	}
	else if(!strcmp(request, "==tweet==")){
		sendTweet(channel);
	}
	else {
		LogMessage("Unknown Request type: %s\n", request);
	}
}

void processPacket(int channel, unsigned char* message, int length){
	if (length > 0)
	{
		//LogMessage("Line = '%s'\n", message);

		if (message[1] == '!')
		{
			LogMessage("Ch %d: Uploaded message %s\n", channel, message);
		}
		else if (message[1] == '$')
		{
			// Telemetry packet
			processTelemetrymessage(channel, message, length);												
		}
		else if ((message[1] & 0xC0) == 0xC0)
		{
			// Binary telemetry packet
			processBinaryTelemetryPacket(channel, message, length);
		}
		else if (message[1] == 0x66)
		{
			// SSDV packet
			processSSDVPacket(channel, message, length);
		}
		else if(message[1] == '+'){
			// Command Line
			processCommandLine(channel, message, length);
		}
		else if(message[1] == '&'){
			LoraConfig.LoRaDevices[channel].ackReceived = !strcmp(message, "&&ACK&&");	
		}
		else if(message[1] == '='){
			// Request
			LogMessage("Received request\n");
			processRequest(channel, message, length);
		}
		else
		{
			LogMessage("Unknown packet type is %x, RSSI %d\n", message[1], readRegister(channel, REG_PACKET_RSSI) - 157);
			channelLogMessage(channel, 3, 1, "Unknown Packet %d, %d bytes", message[0], length);
			LoraConfig.LoRaDevices[channel].UnknownCount++;
		}						
		LoraConfig.LoRaDevices[channel].LastPacketAt = time(NULL);
		ShowPacketCounts(channel);
	}
}



// Sending/Receiving Packets

int receiveMessage(int channel, unsigned char *message){
	int i, Bytes, currentAddr, x;
	unsigned char data[257];

	Bytes = 0;
	
	x = readRegister(channel, REG_IRQ_FLAGS);
  
	// clear the rxDone flag
	writeRegister(channel, REG_IRQ_FLAGS, 0x40); 
   
	// check for payload crc issues (0x20 is the bit we are looking for
	if((x & 0x20) == 0x20)
	{
		// CRC Error
		writeRegister(channel, REG_IRQ_FLAGS, 0x20);		// reset the crc flags
		LoraConfig.LoRaDevices[channel].BadCRCCount++;
		LogMessage("Bad CRC Count?\n");
	}
	//else
	{
		currentAddr = readRegister(channel, REG_FIFO_RX_CURRENT_ADDR);
		Bytes = readRegister(channel, REG_RX_NB_BYTES);

		// channelLogMessage(channel,  9, 1, "Packet   SNR = %4d   ", (char)(readRegister(channel, REG_PACKET_SNR)) / 4);
		// channelLogMessage(channel, 10, 1, "Packet  RSSI = %4d   ", readRegister(channel, REG_PACKET_RSSI) - 157);
		// channelLogMessage(channel, 11, 1, "Freq. Error = %4.1lfkHz ", FrequencyError(channel) / 1000);

		writeRegister(channel, REG_FIFO_ADDR_PTR, currentAddr);   
		
		data[0] = REG_FIFO;
		wiringPiSPIDataRW(channel, data, Bytes+1);
		for (i=0; i<=Bytes; i++)
		{
			message[i] = data[i+1];
		}
		
		message[Bytes] = '\0';
	} 

	// Clear all flags
	writeRegister(channel, REG_IRQ_FLAGS, 0xFF); 
  
	return Bytes;
}

void startReceiving(int channel){
	if (LoraConfig.LoRaDevices[channel].InUse)
	{
	    LogMessage("Receiving\n");
		writeRegister(channel, REG_DIO_MAPPING_1, 0x00);		// 00 00 00 00 maps DIO0 to RxDone
	
		writeRegister(channel, REG_FIFO_RX_BASE_AD, 0);
		writeRegister(channel, REG_FIFO_ADDR_PTR, 0);
	  
		// Setup Receive Continuous Mode
		setMode(channel, RF98_MODE_RX_CONTINUOUS); 
		LoraConfig.LoRaDevices[channel].LoRaMode = lmListening;
	}
}

void receiveForSetTime(double duration){
    unsigned char  Command[200], Telemetry[100], filename[100], *dest, *src;
	int Bytes, ImageNumber, PreviousImageNumber, PacketNumber, PreviousPacketNumber, i;
	uint32_t CallsignCode, PreviousCallsignCode, LoopCount[2];
	struct timeval currentTime, startTime;
		
	gettimeofday(&startTime, NULL);
	gettimeofday(&currentTime, NULL);
	
	startReceiving(0);

	PreviousImageNumber = -1;
	PreviousCallsignCode = 0;
	PreviousPacketNumber = 0;

	LoopCount[0] = 0;
	LoopCount[1] = 0;

	LogMessage("Receiving %d %d, duration %f\n", readRegister(0, REG_OPMODE), readRegister(1, REG_OPMODE), duration);

	while (timeDiff(currentTime, startTime) < duration)
	{
	    //LogMessage("%f ", timeDiff(currentTime, startTime));
	    int channel;
		delay(5);
		for (channel=0; channel<=1; channel++)
		{
			if (LoraConfig.LoRaDevices[channel].InUse)
			{
				if (readRegister(channel, REG_IRQ_FLAGS) & 0x40 || digitalRead(LoraConfig.LoRaDevices[channel].DIO0))
				{
					unsigned char message[257];
					memset(message, 0, 257);
					Bytes = receiveMessage(channel, message);
					LogMessage("Received %d bytes, '%s'\n", Bytes, message);
					

					processPacket(channel, message, Bytes);

					// Clear flags
					//writeRegister(channel, REG_IRQ_FLAGS, 0x00);
				}
				
				if (++LoopCount[channel] > 1000000)
				{
					LoopCount[channel] = 0;
					ShowPacketCounts(channel);
					channelLogMessage(channel, 12, 1, "Current RSSI = %4d   ", readRegister(channel, REG_CURRENT_RSSI) - 157);
					if (LoraConfig.LoRaDevices[channel].LastPacketAt > 0)
					{
						channelLogMessage(channel, 5, 1, "%us since last packet   ", (unsigned int)(time(NULL) - LoraConfig.LoRaDevices[channel].LastPacketAt));
					}
				}
			}
		}
		gettimeofday(&currentTime, NULL);
 	}
}	

void constantReceiving(){
	unsigned char  Command[200], Telemetry[100], filename[100], *dest, *src;
	int Bytes, ImageNumber, PreviousImageNumber, PacketNumber, PreviousPacketNumber, i;
	uint32_t CallsignCode, PreviousCallsignCode, LoopCount[2];
		
	startReceiving(0);

	PreviousImageNumber = -1;
	PreviousCallsignCode = 0;
	PreviousPacketNumber = 0;

	LoopCount[0] = 0;
	LoopCount[1] = 0;

	LogMessage("Receiving %x %x\n", readRegister(0, REG_OPMODE), readRegister(1, REG_OPMODE));

    while (1)
	{
		int channel;
		delay(5);
		for (channel=0; channel<=1; channel++)
		{
			if (LoraConfig.LoRaDevices[channel].InUse)
			{
				if (readRegister(channel, REG_IRQ_FLAGS) & 0x40 || digitalRead(LoraConfig.LoRaDevices[channel].DIO0))
				{
					unsigned char message[257];
					memset(message, 0, 257);
					Bytes = receiveMessage(channel, message);
					LogMessage("Received %d bytes, '%s'\n", Bytes, message);
					
					processPacket(channel, message, Bytes);

					// Clear flags
					//writeRegister(channel, REG_IRQ_FLAGS, 0x00);
				}
				
				if (++LoopCount[channel] > 1000000)
				{
					LoopCount[channel] = 0;
					ShowPacketCounts(channel);
					channelLogMessage(channel, 12, 1, "Current RSSI = %4d   ", readRegister(channel, REG_CURRENT_RSSI) - 157);
					if (LoraConfig.LoRaDevices[channel].LastPacketAt > 0)
					{
						channelLogMessage(channel, 5, 1, "%us since last packet   ", (unsigned int)(time(NULL) - LoraConfig.LoRaDevices[channel].LastPacketAt));
					}
				}
			}
		}
 	}
}

void SendLoRaData(int channel, unsigned char *buffer, int Length){
	unsigned char data[257];
	int i;
	LogMessage("LoRa channel %d Sending %d bytes, %s\n", channel, Length, buffer);
	setMode(channel, RF98_MODE_STANDBY);
	writeRegister(channel, REG_DIO_MAPPING_1, 0x40);		// 01 00 00 00 maps DIO0 to TxDone

	writeRegister(channel, REG_FIFO_TX_BASE_AD, 0x00);  // Update the address ptr to the current tx base address
	writeRegister(channel, REG_FIFO_ADDR_PTR, 0x00); 
	data[0] = REG_FIFO | 0x80;
	for (i=0; i<Length; i++)
	{
		data[i+1] = buffer[i];
	}
	wiringPiSPIDataRW(channel, data, Length+1);
	// LogMessage("Set Tx Mode\n");

	// Set the length. For implicit mode, since the length needs to match what the receiver expects, we have to set a value which is 255 for an SSDV packet
	writeRegister(channel, REG_PAYLOAD_LENGTH, LoraConfig.LoRaDevices[channel].PayloadLength ? LoraConfig.LoRaDevices[channel].PayloadLength : Length);
	// go into transmit mode
	setMode(channel, RF98_MODE_TX);
	while(!(readRegister(channel, REG_IRQ_FLAGS) & 0x08)){
		delay(10);
	}
	LoraConfig.LoRaDevices[channel].LoRaMode = lmSending;
}

int SendLoRaImage(int LoRachannel){
    unsigned char Buffer[256];
    size_t Count;
    int SentSomething = 0;

    if (LoraConfig.channels[LoRachannel].ImageFP != NULL)
    {
        Count = fread(Buffer, 1, 256, LoraConfig.channels[LoRachannel].ImageFP);
        if (Count > 0)
        {
            LogMessage("Record %d, %d bytes\r\n", ++Records, Count);

			LoraConfig.channels[LoRachannel].ImagePacketCount++;
			
            LogMessage("LORA SSDV record %d of %d\r\n", ++LoraConfig.channels[LoRachannel].SSDVRecordNumber, LoraConfig.channels[LoRachannel].SSDVTotalRecords);
			
			SendLoRaData(LoRachannel, Buffer+1, 255);
			
            SentSomething = 1;
        }
        else
        {
            fclose(LoraConfig.channels[LoRachannel].ImageFP);
            LoraConfig.channels[LoRachannel].ImageFP = NULL;
        }
    }

    return SentSomething;
}

void sendTweet(int channel){
	FILE* tweetFile; int length;
	unsigned char tweet[140], message[200], sentence[200];
	LogMessage("Tweeting..\n");
	if((tweetFile = fopen("tweetsToSend.txt", "r")) == NULL) {
		LogMessage("No file of tweets\n");
		return;
	}
	fgets(tweet, 140, tweetFile);
	LogMessage("Tweeting %s\n", tweet);
	sprintf(message, "sudo python screen.py \"%s\"", tweet);
	//length = BuildLoRaString(sentence, message, 0);
	//SendLoRaData(0, sentence, length);
	
	sendCommandLine(channel, message, strlen(message), 3);
	startReceiving(channel);
	
}

int sendCommandLine(int channel, unsigned char *commandLine, int length, int tries){
	int attempts;
	unsigned char commandLineMessage[256];

	sprintf(commandLineMessage, "++%d:%s", ++LoraConfig.LastCommandNo, commandLine);

	for(attempts = 0; attempts < tries; attempts++){
		SendLoRaData(channel, commandLineMessage, length+4);
		if(receiveAck(channel, LoraConfig.LastCommandNo)){
			LogMessage("Received ACK");
			attempts = tries;
		} 		
	}
}

void readAndSendCommandLine(int channel, char *filename, int tries){
    FILE * commandFile;
    unsigned char commandLine[250];
    //LogMessage("Sending command..\n");
    if((commandFile = fopen(filename, "r")) != NULL){ 
        fgets(commandLine, 250, commandFile);
        sendCommandLine(channel, commandLine, strlen(commandLine), tries);
    } else {
        LogMessage("Cannot open command file\n");
    }
}



// Legacy
int TimeToSendOnThischannel(int channel, struct TGPS *GPS){
	long CycleSeconds;
	
	if (LoraConfig.LoRaDevices[channel].CycleTime == 0)
	{
		// Not using time to decide when we can send
		return 1;
	}
	
	// Can't send till we have the time!
	if (GPS->Satellites > 0)
	{
		// Can't Tx twice at the same time
		if (GPS->Seconds != LoraConfig.LoRaDevices[channel].LastTxAt)
		{
			CycleSeconds = GPS->Seconds % LoraConfig.LoRaDevices[channel].CycleTime;
	
			if (CycleSeconds == LoraConfig.LoRaDevices[channel].Slot)
			{
				LoraConfig.LoRaDevices[channel].LastTxAt = GPS->Seconds;
				LoraConfig.LoRaDevices[channel].SendRepeatedPacket = 0;
				return 1;
			}

			if (LoraConfig.LoRaDevices[channel].PacketRepeatLength && (CycleSeconds == LoraConfig.LoRaDevices[channel].RepeatSlot))
			{
				LoraConfig.LoRaDevices[channel].LastTxAt = GPS->Seconds;
				LoraConfig.LoRaDevices[channel].SendRepeatedPacket = 1;
				return 1;
			}
			
			if (LoraConfig.LoRaDevices[channel].UplinkRepeatLength && (CycleSeconds == LoraConfig.LoRaDevices[channel].UplinkSlot))
			{
				LoraConfig.LoRaDevices[channel].LastTxAt = GPS->Seconds;
				LoraConfig.LoRaDevices[channel].SendRepeatedPacket = 2;
				return 1;
			}
			
		}
	}
	
	return 0;
}

void CheckForPacketOnListeningchannels(void){
	int channel;
	
	for (channel=0; channel<=1; channel++)
	{
		if (LoraConfig.LoRaDevices[channel].InUse)
		{
			if (LoraConfig.LoRaDevices[channel].LoRaMode == lmListening)
			{
				if (digitalRead(LoraConfig.LoRaDevices[channel].DIO0))
				{
					unsigned char Message[256];
					int Bytes;
					
					Bytes = receiveMessage(channel, Message);
					LogMessage ("Rx %d bytes\n", Bytes);
					
					if (Bytes > 0)
					{
						if (Message[0] == '$')
						{
							char Payload[32];

							LogMessage("Balloon message\n");
							if (sscanf(Message+2, "%32[^,]", Payload) == 1)
							{
								if (strcmp(Payload, LoraConfig.channels[channel].PayloadID) != 0)
								{
									// LogMessage ("%s\n", Message);
							
									strcpy(LoraConfig.LoRaDevices[channel].PacketToRepeat, Message);
									LoraConfig.LoRaDevices[channel].PacketRepeatLength = strlen(Message);
							
									LoraConfig.LoRaDevices[channel].AirCount++;

									Message[strlen(Message)] = '\0';
								}
							}
						}
						else if ((Message[0] & 0xC0) == 0xC0)
						{
							char Payload[32];
							int SourceID, OurID;
							
							OurID = LoraConfig.LoRaDevices[channel].Slot;
							SourceID = Message[0] & 0x07;
							
							if (SourceID == OurID)
							{
								LogMessage("Balloon Binary Message - ignored\n");
							}
							else
							{
								LogMessage("Balloon Binary Message from sender %d\n", SourceID);
								
								// Replace the sender ID with ours
								Message[0] = Message[0] & 0xC7 | (OurID << 3);
								LoraConfig.LoRaDevices[channel].PacketRepeatLength = sizeof(struct TBinaryPacket);
								memcpy(LoraConfig.LoRaDevices[channel].PacketToRepeat, Message, LoraConfig.LoRaDevices[channel].PacketRepeatLength);
							
								LoraConfig.LoRaDevices[channel].AirCount++;
							}
						}
						else if ((Message[0] & 0xC0) == 0x80)
						{
							int SenderID, TargetID, OurID;
							
							TargetID = Message[0] & 0x07;
							SenderID = (Message[0] >> 3) & 0x07;
							OurID = LoraConfig.LoRaDevices[channel].Slot;

							LogMessage("Uplink from %d to %d Message %s\n",
									SenderID,
									TargetID,
									Message+1);
									
							if (TargetID == OurID)
							{
								LogMessage("Message was for us!\n");
								strcpy(LoraConfig.LoRaDevices[channel].LastCommand, Message+1);
								LogMessage("Message is '%s'\n", LoraConfig.LoRaDevices[channel].LastCommand);
								LoraConfig.LoRaDevices[channel].GroundCount++;
							}
							else
							{
								LogMessage("Message was for another balloon\n");
								Message[0] = Message[0] & 0xC7 | (OurID << 3);
								LoraConfig.LoRaDevices[channel].UplinkRepeatLength = sizeof(struct TBinaryPacket);
								memcpy(LoraConfig.LoRaDevices[channel].UplinkPacket, Message, LoraConfig.LoRaDevices[channel].UplinkRepeatLength);
							}
						}
						else
						{
							LogMessage("Unknown message %02Xh\n", Message[0]);
						}
					}
				}
			}
		}
	}
}

int CheckForFreechannel(struct TGPS *GPS){
	int channel;
	
	for (channel=0; channel<=1; channel++)
	{
		if (LoraConfig.LoRaDevices[channel].InUse)
		{
			if ((LoraConfig.LoRaDevices[channel].LoRaMode != lmSending) || digitalRead(LoraConfig.LoRaDevices[channel].DIO0))
			{
				LogMessage ("LoRa channel %d is free\n", channel);
				// Either not sending, or was but now it's sent.  Clear the flag if we need to
				if (LoraConfig.LoRaDevices[channel].LoRaMode == lmSending)
				{
					// Clear that IRQ flag
					writeRegister(channel, REG_IRQ_FLAGS, 0x08); 
					LoraConfig.LoRaDevices[channel].LoRaMode = lmIdle;
				}
				// else if ((channel == 1) && (LoraConfig.LoRaDevices[channel].CycleTime == 0))
				// {
					// // Get here first time that channel 1 is in use
					// // Add a short delay to put the 2 channels out of sync, to make things easier at the rx end
					// delay(2000);
				// }
				
				// Mow we test to see if we're doing TDM or not
				// For TDM, if it's not a slot that we send in, then we should be in listening mode
				// Otherwise, we just send
				
				if (TimeToSendOnThischannel(channel, GPS))
				{
					// Either sending continuously, or it's our slot to send in
					// LogMessage("channel %d is free\n", channel);
					
					return channel;
				}
				else if (LoraConfig.LoRaDevices[channel].CycleTime > 0)
				{
					// TDM system and not time to send, so we can listen
					if (LoraConfig.LoRaDevices[channel].LoRaMode == lmIdle)
					{
						startReceiving(channel);
					}
				}
			}
		}
	}
	
	return -1;
}



// Tweets

bool requestTweet(int channel){
	int length, i; unsigned char request [30];
	
	// Send request	
	length = BuildLoRaString(request,"==tweet==",channel);
	SendLoRaData(channel, request, length);
	
	// Wait for receipt
	LoraConfig.LoRaDevices[channel].tweetReceived = false;
	for( i = 0; i < 4; i++){
		receiveForSetTime(2.0);
		if(LoraConfig.LoRaDevices[channel].tweetReceived){
			LogMessage("Received Tweet\n");
			i = 4;
		}
	}
	return LoraConfig.LoRaDevices[channel].tweetReceived;
	// Return 
	
	
}

void tweetFromFile(FILE* tweetFile, int lineNumber){
	int i; char tweet[140], commandLine[200];
	
	if(tweetFile == NULL) return;
	
	/*for(i = 0; i <= lineNumber; i++){
		fgets(tweet, 140, tweetFile);
	}*/
	if (fgets(tweet, 50, tweetFile) == NULL) {
	    // When Error occurs or EOF
	    rewind(tweetFile);
	    fgets(tweet, 50, tweetFile);
	}

	sprintf(commandLine, "sudo python screen.py \"%s\"", tweet);
	LogMessage("System: %s\n", commandLine);
	system(commandLine);
}

bool requestCommandLine(int channel){
	int length, i; unsigned char request [30];
	
	// Send request	
	length = BuildLoRaString(request,"==command==",channel);
	SendLoRaData(channel, request, length);
	
	// Wait for receipt
	LoraConfig.LoRaDevices[channel].commandReceived = false;
	for( i = 0; i < 4; i++){
		receiveForSetTime(2.0);
		if(LoraConfig.LoRaDevices[channel].commandReceived){
			LogMessage("Received Command\n");
			i = 4;
		}
	}
	return LoraConfig.LoRaDevices[channel].commandReceived;
	// Return 
	
	
}


// Loops

void *lora_loop(void *some_void_ptr){
	
	int length; unsigned char message[200];
	int tweetNumber = 0; FILE* tweetFile;
	struct TGPS *GPS;
		
	struct timeval lastTweet, timeNow;	

	
	GPS = (struct TGPS *) some_void_ptr;
	//loadConfigFile();
	LogMessage("Gona set up \n");
	
	//setupRFM98(0);
	//setupRFM98(1);
	initialiseLoRa();
	LogMessage("Setup LoRa\n");
	printLoRaSettings();
	
	if((tweetFile = fopen("tweets.txt", "r" )) == NULL){
		fprintf(stderr, "Error opening Tweet File");
	}
	
	gettimeofday(&lastTweet,NULL);
		
	while(1){
	
		length = BuildLoRaPacket(message, 0, GPS);
		SendLoRaData(0, message, length);
		delay(1000);
		
		//length = BuildLoRaPacket(message, 1, GPS);
		//SendLoRaData(1, message, length);
	    //delay(2000);
		
		//length = BuildLoRaPacket(message, 0);
		//SendLoRaData(0, message, length);
		
		gettimeofday(&timeNow, NULL);
		
		if(timeDiff(timeNow, lastTweet) > 10.0){
			if(!requestTweet(0)){
				tweetFromFile(tweetFile, tweetNumber++);
			}
			
			gettimeofday(&lastTweet, NULL);
		}
		
		
		
		
	
	}
	fclose(tweetFile);
}

/*
void *LoRaLoop(void *some_void_ptr){
	int ReturnCode, ImagePacketCount, fd, LoRachannel;
	unsigned long Sentence_Counter = 0;
	char Sentence[100], Command[100];
	struct stat st = {0};
	struct TGPS *GPS;

	GPS = (struct TGPS *)some_void_ptr;

	for (LoRachannel=0; LoRachannel<2; LoRachannel++)
	{
		setupRFM98(LoRachannel);
		if (LoraConfig.LoRaDevices[LoRachannel].SpeedMode == 2)
		{
			startReceiving(LoRachannel);
		}
	}

	ImagePacketCount = 0;
	
	while (1)
	{	
		delay(5);								// To stop this loop gobbling up CPU

		CheckForPacketOnListeningchannels();
		
		LoRachannel = CheckForFreechannel(GPS);		// 0 or 1 if there's a free channel and we should be sending on that channel now
		
		if (LoRachannel >= 0)
		{
			int MaxImagePackets;
			
			if (LoraConfig.LoRaDevices[LoRachannel].SendRepeatedPacket == 2)
			{
				LogMessage("Repeating uplink packet of %d bytes\n", LoraConfig.LoRaDevices[LoRachannel].UplinkRepeatLength);
				
				SendLoRaData(LoRachannel, LoraConfig.LoRaDevices[LoRachannel].UplinkPacket, LoraConfig.LoRaDevices[LoRachannel].UplinkRepeatLength);
				
				LoraConfig.LoRaDevices[LoRachannel].UplinkRepeatLength = 0;
			}
			else if (LoraConfig.LoRaDevices[LoRachannel].SendRepeatedPacket == 1)
			{
				LogMessage("Repeating balloon packet of %d bytes\n", LoraConfig.LoRaDevices[LoRachannel].PacketRepeatLength);
				
				SendLoRaData(LoRachannel, LoraConfig.LoRaDevices[LoRachannel].PacketToRepeat, LoraConfig.LoRaDevices[LoRachannel].PacketRepeatLength);
				
				LoraConfig.LoRaDevices[LoRachannel].PacketRepeatLength = 0;
			}
			else
			{
				StartNewFileIfNeeded(LoRachannel);
				
				MaxImagePackets = (GPS->Altitude > LoraConfig.SSDVHigh) ? LoraConfig.channels[LoRachannel].ImagePackets : 1;
				
				if ((LoraConfig.channels[LoRachannel].ImageFP == NULL) || (LoraConfig.channels[LoRachannel].ImagePacketCount >= MaxImagePackets))
				{
					int PacketLength;

					// Telemetry packet
					
					if (LoraConfig.LoRaDevices[LoRachannel].Binary)
					{
						PacketLength = BuildLoRaPositionPacket(Sentence, LoRachannel, GPS);
						LogMessage("LoRa%d: Binary packet %d bytes\n", LoRachannel, PacketLength);
					}
					else
					{
						PacketLength = BuildLoRaSentence(Sentence, LoRachannel, GPS);
						LogMessage("LoRa%d: %s", LoRachannel, Sentence);
					}
									
					SendLoRaData(LoRachannel, Sentence, PacketLength);		

					LoraConfig.channels[LoRachannel].ImagePacketCount = 0;
				}
				else
				{
					// Image packet
					
					LogMessage("LoRa%d: Send image packet\n", LoRachannel);
					SendLoRaImage(LoRachannel);
				}
			}
		}
	}
}
*/