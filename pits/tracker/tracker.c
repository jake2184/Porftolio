



/*------------------------------------------------------------------\
                            Main executable

main():
Initialises all devices and loads configuration from a file.
Starts threads to control external sensors, LoRa, GPS, camera, etc.
Note: Many threads are currently commented out to prevent them starting. 
Loops, sending RTTY telemetry and SSDV images. 

Heavily adapted from tracker.c from https://github.com/PiInTheSky
\------------------------------------------------------------------*/


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
#include <math.h>
#include <pthread.h>
#include <wiringPi.h>
#include "gps.h"
//#include "DS18B20.h"
//#include "adc.h"
#include "misc.h"
//#include "snapper.h"
//#include "led.h"
#include "bmp085.h"
//#include "aprs.h"
#include "lora.h"
#include "sensors/sen.h"

#include "auxiliary_threads.h"

struct TConfig Config;

// Pin allocations.  Do not change unless you're using your own hardware
#define NTX2B_ENABLE	0
#define UBLOX_ENABLE	2

struct termios options;
char *SSDVFolder="/home/pi/pits/tracker/images";
 
void BuildSentence(char *TxLine, int SentenceCounter, struct TGPS *GPS)
{
    int Count, i, j;
    unsigned char c;
    unsigned int CRC, xPolynomial;
	char TimeBuffer1[12], TimeBuffer2[10], ExtraFields[20];
	
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
	
	
	if (Config.EnableBMP085)
	{
		sprintf(ExtraFields, ",%.1f,%.0f", GPS->ExternalTemperature, GPS->Pressure);
	}
	
    sprintf(TxLine, "$$%s,%d,%s,%7.5lf,%7.5lf,%5.5u,%d,%d,%d,%2.2f,%2.2f,%2.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%d,%2.2f",
            Config.channels[RTTY_CHANNEL].PayloadID,
            SentenceCounter,
			TimeBuffer2,
            GPS->Latitude,
            GPS->Longitude,
            GPS->Altitude,
			(GPS->Speed * 13) / 7,
			GPS->Direction,
			GPS->Satellites,            
            GPS->InternalTemperature,
            GPS->BatteryVoltage,
			sensors.humidity,
			sensors.x,
			sensors.y,
			sensors.z,
			sensors.light,
			sensors.infrared,
			GPS->Pressure,
			GPS->ExternalTemperature	
			);

/*
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
*/
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
    TxLine[Count++] = Hex((CRC >> 12) & 15);
    TxLine[Count++] = Hex((CRC >> 8) & 15);
    TxLine[Count++] = Hex((CRC >> 4) & 15);
    TxLine[Count++] = Hex(CRC & 15);
	TxLine[Count++] = '\n';  
	TxLine[Count++] = '\0';  

    printf("RTTY: %s", TxLine);
}


void ReadString(FILE *fp, char *keyword, int channel, char *Result, int Length, int NeedValue)
{
	char line[100], FullKeyWord[64], *token, *value;
 
	if (channel >= 0)
	{
		sprintf(FullKeyWord, "%s_%d", keyword, channel);
	}
	else
	{
		strcpy(FullKeyWord, keyword);
	}
 
	fseek(fp, 0, SEEK_SET);
	*Result = '\0';

	while (fgets(line, sizeof(line), fp) != NULL)
	{
		line[strcspn(line, "\r")] = '\0';			// Ignore any CR (in case someone has edited the file from Windows with notepad)
		
		token = strtok(line, "=");
		if (strcasecmp(FullKeyWord, token) == 0)
		{
			value = strtok(NULL, "\n");
			strcpy(Result, value);
			return;
		}
	}

	if (NeedValue)
	{
		printf("Missing value for '%s' in configuration file\n", keyword);
		exit(1);
	}
}

int ReadInteger(FILE *fp, char *keyword, int channel, int NeedValue, int DefaultValue)
{
	char Temp[64];
	
	ReadString(fp, keyword, channel, Temp, sizeof(Temp), NeedValue);

	if (Temp[0])
	{
		return atoi(Temp);
	}
	
	return DefaultValue;
}

int ReadBoolean(FILE *fp, char *keyword, int channel, int NeedValue, int *Result)
{
	char Temp[32];

	ReadString(fp, keyword, channel, Temp, sizeof(Temp), NeedValue);

	if (*Temp)
	{
		*Result = (*Temp == '1') || (*Temp == 'Y') || (*Temp == 'y') || (*Temp == 't') || (*Temp == 'T');
	}
	else
	{
		*Result = 0;
	}
	
	return *Temp;
}

int ReadBooleanFromString(FILE *fp, char *keyword, char *searchword)
{
	char Temp[100];

	ReadString(fp, keyword, -1, Temp, sizeof(Temp), 0);

	if (strcasestr(Temp, searchword)) return 1; else return 0;
}

speed_t BaudToSpeed(int baud)
{
	switch (baud)
	{
		case 50: return B50;
		case 75: return B75;
		case 150: return B150;
		case 200: return B200;
		case 300: return B300;
		case 600: return B600;
		case 1200: return B1200;
	}

	return 0;
}

void LoadConfigFile(struct TConfig *Config)
{
	const char *LoRaModes[5] = {"slow", "SSDV", "repeater", "turbo", "TurboX"};
	FILE *fp;
	int BaudRate, channel;
	char *filename = "/boot/pisky.txt";
	Config->GPSDevice[0] = '\0';
	
	if ((fp = fopen(filename, "r")) == NULL)
	{
		printf("\nFailed to open config file %s (error %d - %s).\nPlease check that it exists and has read permission.\n", filename, errno, strerror(errno));
		exit(1);
	}

	ReadBoolean(fp, "disable_monitor", -1, 0, &(Config->DisableMonitor));
	if (Config->DisableMonitor)
	{
		printf("HDMI/Composite outputs will be disabled\n");
	}

	ReadBoolean(fp, "Disable_RTTY", -1, 0, &(Config->DisableRTTY));
	Config->channels[RTTY_CHANNEL].Enabled = !Config->DisableRTTY;
	if (Config->DisableRTTY)
	{
		printf("RTTY Disabled\n");
	}
	else
	{
		ReadString(fp, "payload", -1, Config->channels[RTTY_CHANNEL].PayloadID, sizeof(Config->channels[RTTY_CHANNEL].PayloadID), 1);
		printf ("RTTY Payload ID = '%s'\n", Config->channels[RTTY_CHANNEL].PayloadID);
		
		ReadString(fp, "frequency", -1, Config->Frequency, sizeof(Config->Frequency), 0);
		
		BaudRate = ReadInteger(fp, "baud", -1, 1, 300);
		
		Config->channels[RTTY_CHANNEL].BaudRate = BaudRate;
		
		Config->TxSpeed = BaudToSpeed(BaudRate);
		if (Config->TxSpeed == B0)
		{
			printf ("Unknown baud rate %d\nPlease edit in configuration file\n", BaudRate);
			exit(1);
		}
		printf ("Radio baud rate = %d\n", BaudRate);
	}

	Config->EnableGPSLogging = ReadBooleanFromString(fp, "logging", "GPS");
	if (Config->EnableGPSLogging) printf("GPS Logging enabled\n");

	Config->EnableTelemetryLogging = ReadBooleanFromString(fp, "logging", "Telemetry");
	if (Config->EnableTelemetryLogging) printf("Telemetry Logging enabled\n");
	
	ReadBoolean(fp, "enable_bmp085", -1, 0, &(Config->EnableBMP085));
	if (Config->EnableBMP085)
	{
		printf("BMP085 Enabled\n");
	}


	ReadBoolean(fp, "camera", -1, 0, &(Config->Camera));
	printf ("Camera %s\n", Config->Camera ? "Enabled" : "Disabled");
	if (Config->Camera)
	{
		Config->SSDVHigh = ReadInteger(fp, "high", -1, 0, 2000);
		printf ("Image size changes at %dm\n", Config->SSDVHigh);
	
		Config->channels[0].ImageWidthWhenLow = ReadInteger(fp, "low_width", -1, 0, 320);
		Config->channels[0].ImageHeightWhenLow = ReadInteger(fp, "low_height", -1, 0, 240);
		printf ("RTTY Low image size %d x %d pixels\n", Config->channels[0].ImageWidthWhenLow, Config->channels[0].ImageHeightWhenLow);
		
		Config->channels[0].ImageWidthWhenHigh = ReadInteger(fp, "high_width", -1, 0, 640);
		Config->channels[0].ImageHeightWhenHigh = ReadInteger(fp, "high_height", -1, 0, 480);
		printf ("RTTY High image size %d x %d pixels\n", Config->channels[0].ImageWidthWhenHigh, Config->channels[0].ImageHeightWhenHigh);

		Config->channels[0].ImagePackets = ReadInteger(fp, "image_packets", -1, 0, 4);
		printf ("RTTY: 1 Telemetry packet every %d image packets\n", Config->channels[0].ImagePackets);
		
		Config->channels[0].ImagePeriod = ReadInteger(fp, "image_period", -1, 0, 60);
		printf ("RTTY: %d seconds between photographs\n", Config->channels[0].ImagePeriod);
	}
	
	ReadString(fp, "GPS_Serial", -1, Config->GPSDevice, sizeof(Config->GPSDevice), 0);
	
	// I2C overrides.  Only needed for users own boards, or for some of our prototypes
	if (ReadInteger(fp, "SDA", -1, 0, 0))
	{
		printf ("I2C SDA overridden to %d\n", Config->SDA);
		Config->SDA = ReadInteger(fp, "SDA", -1, 0, 0);
	}

	if (ReadInteger(fp, "SCL", -1, 0, 0))
	{
		Config->SCL = ReadInteger(fp, "SCL", -1, 0, 0);
		printf ("I2C SCL overridden to %d\n", Config->SCL);
	}
	
	// APRS settings
	ReadString(fp, "APRS_Callsign", -1, Config->APRS_Callsign, sizeof(Config->APRS_Callsign), 0);
	Config->APRS_ID = ReadInteger(fp, "APRS_ID", -1, 0, 0);
	Config->APRS_Period = ReadInteger(fp, "APRS_Period", -1, 0, 0);
	if (*(Config->APRS_Callsign) && Config->APRS_ID && Config->APRS_Period)
	{
		printf("APRS enabled for callsign %s:%d every %d minute%s\n", Config->APRS_Callsign, Config->APRS_ID, Config->APRS_Period, Config->APRS_Period > 1 ? "s" : "");
	}
	
	/*
	// LORA
	if (NewBoard())
	{
		// For dual card.  These are for the second prototype (earlier one will need overrides)

		Config->LoRaDevices[0].DIO0 = 6;
		Config->LoRaDevices[0].DIO5 = 5;
		
		Config->LoRaDevices[1].DIO0 = 31;
		Config->LoRaDevices[1].DIO5 = 26;
	}
	else
	{
		Config->LoRaDevices[0].DIO0 = 6;
		Config->LoRaDevices[0].DIO5 = 5;
		
		Config->LoRaDevices[1].DIO0 = 3;
		Config->LoRaDevices[1].DIO5 = 4;
	}

	Config->LoRaDevices[0].InUse = 0;
	Config->LoRaDevices[1].InUse = 0;
	
	Config->LoRaDevices[0].LoRaMode = lmIdle;
	Config->LoRaDevices[1].LoRaMode = lmIdle;


	for (channel=0; channel<=1; channel++)
	{
		int Temp;
		char TempString[64];
		
		strcpy(Config->LoRaDevices[channel].LastCommand, "None");
		
		Config->LoRaDevices[channel].Frequency[0] = '\0';
		ReadString(fp, "LORA_Frequency", channel, Config->LoRaDevices[channel].Frequency, sizeof(Config->LoRaDevices[channel].Frequency), 0);
		if (Config->LoRaDevices[channel].Frequency[0])
		{
			printf("LORA%d frequency set to %s\n", channel, Config->LoRaDevices[channel].Frequency);
			Config->LoRaDevices[channel].InUse = 1;
			Config->channels[LORA_CHANNEL+channel].Enabled = 1;

			ReadString(fp, "LORA_Payload", channel, Config->channels[LORA_CHANNEL+channel].PayloadID, sizeof(Config->channels[LORA_CHANNEL+channel].PayloadID), 1);
			printf ("LORA%d Payload ID = '%s'\n", channel, Config->channels[LORA_CHANNEL+channel].PayloadID);
			
			Config->LoRaDevices[channel].SpeedMode = ReadInteger(fp, "LORA_Mode", channel, 0, 0);
			printf("LORA%d %s mode\n", channel, LoRaModes[Config->LoRaDevices[channel].SpeedMode]);

			// DIO0 / DIO5 overrides
			Config->LoRaDevices[channel].DIO0 = ReadInteger(fp, "LORA_DIO0", channel, 0, Config->LoRaDevices[channel].DIO0);

			Config->LoRaDevices[channel].DIO5 = ReadInteger(fp, "LORA_DIO5", channel, 0, Config->LoRaDevices[channel].DIO5);

			printf("LORA%d DIO0=%d DIO5=%d\n", channel, Config->LoRaDevices[channel].DIO0, Config->LoRaDevices[channel].DIO5);
			
			Config->channels[LORA_CHANNEL+channel].ImageWidthWhenLow = ReadInteger(fp, "LORA_low_width", channel, 0, 320);
			Config->channels[LORA_CHANNEL+channel].ImageHeightWhenLow = ReadInteger(fp, "LORA_low_height", channel, 0, 240);
			printf ("LORA%d Low image size %d x %d pixels\n", channel, Config->channels[LORA_CHANNEL+channel].ImageWidthWhenLow, Config->channels[LORA_CHANNEL+channel].ImageHeightWhenLow);
			
			Config->channels[LORA_CHANNEL+channel].ImageWidthWhenHigh = ReadInteger(fp, "LORA_high_width", channel, 0, 640);
			Config->channels[LORA_CHANNEL+channel].ImageHeightWhenHigh = ReadInteger(fp, "LORA_high_height", channel, 0, 480);
			printf ("LORA%d High image size %d x %d pixels\n", channel, Config->channels[LORA_CHANNEL+channel].ImageWidthWhenHigh, Config->channels[LORA_CHANNEL+channel].ImageHeightWhenHigh);

			Config->channels[LORA_CHANNEL+channel].ImagePackets = ReadInteger(fp, "LORA_image_packets", channel, 0, 4);
			printf ("LORA%d: 1 Telemetry packet every %d image packets\n", channel, Config->channels[LORA_CHANNEL+channel].ImagePackets);
			
			Config->channels[LORA_CHANNEL+channel].ImagePeriod = ReadInteger(fp, "LORA_image_period", channel, 0, 60);
			printf ("LORA%d: %d seconds between photographs\n", channel, Config->channels[LORA_CHANNEL+channel].ImagePeriod);
			

			Config->LoRaDevices[channel].CycleTime = ReadInteger(fp, "LORA_Cycle", channel, 0, 0);			
			if (Config->LoRaDevices[channel].CycleTime > 0)
			{
				printf("LORA%d cycle time %d\n", channel, Config->LoRaDevices[channel].CycleTime);

				Config->LoRaDevices[channel].Slot = ReadInteger(fp, "LORA_Slot", channel, 0, 0);
				printf("LORA%d Slot %d\n", channel, Config->LoRaDevices[channel].Slot);

				Config->LoRaDevices[channel].RepeatSlot = ReadInteger(fp, "LORA_Repeat", channel, 0, 0);			
				printf("LORA%d Repeat Slot %d\n", channel, Config->LoRaDevices[channel].RepeatSlot);

				Config->LoRaDevices[channel].UplinkSlot = ReadInteger(fp, "LORA_Uplink", channel, 0, 0);			
				printf("LORA%d Uplink Slot %d\n", channel, Config->LoRaDevices[channel].UplinkSlot);

				ReadBoolean(fp, "LORA_Binary", channel, 0, &(Config->LoRaDevices[channel].Binary));			
				printf("LORA%d Set To %s\n", channel, Config->LoRaDevices[channel].Binary ? "Binary" : "ASCII");
			}

			if (Config->LoRaDevices[channel].SpeedMode == 4)
			{
				// Testing
				Config->LoRaDevices[channel].ImplicitOrExplicit = IMPLICIT_MODE;
				Config->LoRaDevices[channel].ErrorCoding = ERROR_CODING_4_5;
				Config->LoRaDevices[channel].Bandwidth = BANDWIDTH_250K;
				Config->LoRaDevices[channel].SpreadingFactor = SPREADING_6;
				Config->LoRaDevices[channel].LowDataRateOptimize = 0;		
				Config->channels[LORA_CHANNEL+channel].BaudRate = 16828;
			}
			else if (Config->LoRaDevices[channel].SpeedMode == 3)
			{
				// Normal mode for high speed images in 868MHz band
				Config->LoRaDevices[channel].ImplicitOrExplicit = EXPLICIT_MODE;
				Config->LoRaDevices[channel].ErrorCoding = ERROR_CODING_4_6;
				Config->LoRaDevices[channel].Bandwidth = BANDWIDTH_250K;
				Config->LoRaDevices[channel].SpreadingFactor = SPREADING_7;
				Config->LoRaDevices[channel].LowDataRateOptimize = 0;		
				Config->channels[LORA_CHANNEL+channel].BaudRate = 8000;		// check!!
			}
			else if (Config->LoRaDevices[channel].SpeedMode == 2)
			{
				// Normal mode for repeater network
				// 72 byte packet is approx 1.5 seconds so needs at least 30 seconds cycle time if repeating one balloon
				Config->LoRaDevices[channel].ImplicitOrExplicit = EXPLICIT_MODE;
				Config->LoRaDevices[channel].ErrorCoding = ERROR_CODING_4_8;
				Config->LoRaDevices[channel].Bandwidth = BANDWIDTH_62K5;
				Config->LoRaDevices[channel].SpreadingFactor = SPREADING_8;
				Config->LoRaDevices[channel].LowDataRateOptimize = 0;		
				Config->channels[LORA_CHANNEL+channel].BaudRate = 2000;		// Not used (only for SSDV modes)
			}
			else if (Config->LoRaDevices[channel].SpeedMode == 1)
			{
				// Normal mode for SSDV
				Config->LoRaDevices[channel].ImplicitOrExplicit = IMPLICIT_MODE;
				Config->LoRaDevices[channel].ErrorCoding = ERROR_CODING_4_5;
				Config->LoRaDevices[channel].Bandwidth = BANDWIDTH_20K8;
				Config->LoRaDevices[channel].SpreadingFactor = SPREADING_6;
				Config->LoRaDevices[channel].LowDataRateOptimize = 0;
				Config->channels[LORA_CHANNEL+channel].BaudRate = 1400;		// Used to calculate time till end of image
			}
			else
			{
				// Normal mode for telemetry
				Config->LoRaDevices[channel].ImplicitOrExplicit = EXPLICIT_MODE;
				Config->LoRaDevices[channel].ErrorCoding = ERROR_CODING_4_8;
				Config->LoRaDevices[channel].Bandwidth = BANDWIDTH_20K8;
				Config->LoRaDevices[channel].SpreadingFactor = SPREADING_11;
				Config->LoRaDevices[channel].LowDataRateOptimize = 0x08;		
				Config->channels[LORA_CHANNEL+channel].BaudRate = 60;		// Not used (only for SSDV modes)
			}
			
			Temp = ReadInteger(fp, "LORA_SF", channel, 0, 0);
			if ((Temp >= 6) && (Temp <= 12))
			{
				Config->LoRaDevices[channel].SpreadingFactor = Temp << 4;
				printf("LoRa Setting SF=%d\n", Temp);
			}

			ReadString(fp, "LORA_Bandwidth", channel, TempString, sizeof(TempString), 0);
			if (*TempString)
			{
				printf("LoRa Setting BW=%s\n", TempString);
			}
			if (strcmp(TempString, "7K8") == 0)
			{
				Config->LoRaDevices[channel].Bandwidth = BANDWIDTH_7K8;
			}
			if (strcmp(TempString, "10K4") == 0)
			{
				Config->LoRaDevices[channel].Bandwidth = BANDWIDTH_10K4;
			}
			if (strcmp(TempString, "15K6") == 0)
			{
				Config->LoRaDevices[channel].Bandwidth = BANDWIDTH_15K6;
			}
			if (strcmp(TempString, "20K8") == 0)
			{
				Config->LoRaDevices[channel].Bandwidth = BANDWIDTH_20K8;
			}
			if (strcmp(TempString, "31K25") == 0)
			{
				Config->LoRaDevices[channel].Bandwidth = BANDWIDTH_31K25;
			}
			if (strcmp(TempString, "41K7") == 0)
			{
				Config->LoRaDevices[channel].Bandwidth = BANDWIDTH_41K7;
			}
			if (strcmp(TempString, "62K5") == 0)
			{
				Config->LoRaDevices[channel].Bandwidth = BANDWIDTH_62K5;
			}
			if (strcmp(TempString, "125K") == 0)
			{
				Config->LoRaDevices[channel].Bandwidth = BANDWIDTH_125K;
			}
			if (strcmp(TempString, "250K") == 0)
			{
				Config->LoRaDevices[channel].Bandwidth = BANDWIDTH_250K;
			}
			if (strcmp(TempString, "500K") == 0)
			{
				Config->LoRaDevices[channel].Bandwidth = BANDWIDTH_500K;
			}
			
			if (ReadBoolean(fp, "LORA_Implicit", channel, 0, &Temp))
			{
				if (Temp)
				{
					Config->LoRaDevices[channel].ImplicitOrExplicit = IMPLICIT_MODE;
				}
			}
			
			Temp = ReadInteger(fp, "LORA_Coding", channel, 0, 0);
			if ((Temp >= 5) && (Temp <= 8))
			{
				Config->LoRaDevices[channel].ErrorCoding = (Temp-4) << 1;
				printf("LoRa Setting Error Coding=%d\n", Temp);
			}

			if (ReadBoolean(fp, "LORA_LowOpt", channel, 0, &Temp))
			{
				if (Temp)
				{
					Config->LoRaDevices[channel].LowDataRateOptimize = 0x08;
				}
			}

			Config->LoRaDevices[channel].Power = ReadInteger(fp, "LORA_Power", channel, 0, PA_MAX_UK);
			printf("LORA%d power set to %02Xh\n", channel, Config->LoRaDevices[channel].Power);

			Config->LoRaDevices[channel].PayloadLength = Config->LoRaDevices[channel].ImplicitOrExplicit == IMPLICIT_MODE ? 255 : 0;
		}
		else
		{
			Config->LoRaDevices[channel].InUse = 0;
		}
	}
	*/
	
	fclose(fp);
}

void SetMTX2Frequency(char *FrequencyString)
{
	float _mtx2comp;
	int _mtx2int;
	long _mtx2fractional;
	char _mtx2command[17];
	int fd;
	double Frequency;

	pinMode (NTX2B_ENABLE, OUTPUT);
	digitalWrite (NTX2B_ENABLE, 0);
	delayMilliseconds (100);
	
	fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd >= 0)
	{
		tcgetattr(fd, &options);

		options.c_cflag &= ~CSTOPB;
		cfsetispeed(&options, B9600);
		cfsetospeed(&options, B9600);
		options.c_cflag |= CS8;
		options.c_oflag &= ~ONLCR;
		options.c_oflag &= ~OPOST;

		tcsetattr(fd, TCSANOW, &options);

		delayMilliseconds (100);
		pinMode (NTX2B_ENABLE, INPUT);
		pullUpDnControl(NTX2B_ENABLE, PUD_OFF);
		delayMilliseconds (100);
		
		if (strlen(FrequencyString) < 3)
		{
			// Convert channel number to frequency
			Frequency = strtol(FrequencyString, NULL, 16) * 0.003125 + 434.05;
		}
		else
		{
			Frequency = atof(FrequencyString);
		}
		
		printf("RTTY Frequency set to %8.4fMHz\n", Frequency);
		
		_mtx2comp=(Frequency+0.0015)/6.5;
		_mtx2int=_mtx2comp;
		_mtx2fractional = ((_mtx2comp-_mtx2int)+1) * 524288;
		snprintf(_mtx2command,17,"@PRG_%02X%06lX\r",_mtx2int-1, _mtx2fractional);
		write(fd, _mtx2command, strlen(_mtx2command)); 

		delayMilliseconds (100);
		printf("MTX2 command is %s\n", _mtx2command);

		close(fd);
	}
}

void SetNTX2BFrequency(char *FrequencyString)
{
	int fd, Frequency;
	char Command[16];

	fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd >= 0)
	{
		tcgetattr(fd, &options);

		options.c_cflag &= ~CSTOPB;
		cfsetispeed(&options, B4800);
		cfsetospeed(&options, B4800);
		options.c_cflag |= CS8;
		options.c_oflag &= ~ONLCR;
		options.c_oflag &= ~OPOST;

		tcsetattr(fd, TCSANOW, &options);

		pinMode (NTX2B_ENABLE, INPUT);
		pullUpDnControl(NTX2B_ENABLE, PUD_OFF);

		if (strlen(FrequencyString) < 3)
		{
			// Already a channel number
			Frequency = strtol(FrequencyString, NULL, 16);
		}
		else
		{
			// Convert from MHz to channel number
			Frequency = (int)((atof(FrequencyString) - 434.05) / 0.003124);
		}
		
		sprintf(Command, "%cch%02X\r", 0x80, Frequency);
		write(fd, Command, strlen(Command)); 

		printf("NTX2B-FA transmitter now set to channel %02Xh which is %8.4lfMHz\n", Frequency, (double)(Frequency) * 0.003125 + 434.05);

		close(fd);
	}
}

void SetFrequency(char *Frequency)
{
	if (NewBoard())
	{
		SetMTX2Frequency(Frequency);
	}
	else
	{
		SetNTX2BFrequency(Frequency);
	}
}

int OpenSerialPort(void)
{
	int fd;

	fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd >= 0)
	{
		/* get the current options */
		tcgetattr(fd, &options);

		/* set raw input */
		options.c_lflag &= ~ECHO;
		options.c_cc[VMIN]  = 0;
		options.c_cc[VTIME] = 10;

		options.c_cflag |= CSTOPB;
		cfsetispeed(&options, Config.TxSpeed);
		cfsetospeed(&options, Config.TxSpeed);
		options.c_cflag |= CS8;
		options.c_oflag &= ~ONLCR;
		options.c_oflag &= ~OPOST;
	
		tcsetattr(fd, TCSANOW, &options);
	}

	return fd;
}

void SendSentence(char *TxLine)
{
	int fd;

	
	if ((fd = OpenSerialPort()) >= 0)
	{
		write(fd, TxLine, strlen(TxLine));
		
		if (close(fd) < 0)
		{
			printf("NOT Sent - error %d\n", errno);
		}
		
		if (Config.EnableTelemetryLogging)
		{
			WriteLog("telemetry.txt", TxLine);
		}
	}
	else
	{
		printf("Failed to open serial port\n");
	}
	
}

int SendRTTYImage()
{
    unsigned char Buffer[256];
    size_t Count;
    int SentSomething = 0;
	int fd;

	StartNewFileIfNeeded(RTTY_CHANNEL);
	
    if (Config.channels[RTTY_CHANNEL].ImageFP != NULL)
    {
        Count = fread(Buffer, 1, 256, Config.channels[RTTY_CHANNEL].ImageFP);
        if (Count > 0)
        {
            printf("RTTY SSDV record %d of %d\r\n", ++Config.channels[RTTY_CHANNEL].SSDVRecordNumber, Config.channels[RTTY_CHANNEL].SSDVTotalRecords);

			Config.channels[RTTY_CHANNEL].ImagePacketCount++;

			if ((fd = OpenSerialPort()) >= 0)
			{
				write(fd, Buffer, Count);
				close(fd);
			}

            SentSomething = 1;
        }
        else
        {
            fclose(Config.channels[RTTY_CHANNEL].ImageFP);
            Config.channels[RTTY_CHANNEL].ImageFP = NULL;
        }
    }

    return SentSomething;
}


int main(void)
{
	int fd, ReturnCode, i;
	unsigned long Sentence_Counter = 0;
	char Sentence[300], Command[100];
	struct stat st = {0};
	struct TGPS GPS;
	pthread_t SensorThread, LoRaThread, APRSThread, GPSThread, DS18B20Thread, ADCThread, CameraThread, BMP085Thread, LEDThread;

	printf("\n\nRASPBERRY PI-IN-THE-SKY FLIGHT COMPUTER\n");
	printf(    "=======================================\n\n");
	
	char commandLine[200];
	sprintf(commandLine, "sudo python screen.py \"ICARUS\"");
	LogMessage("System: %s\n", commandLine);
	system(commandLine);

	if (NewBoard())
	{
		printf("RPi Model A+ or B+\n\n");
		
		Config.LED_OK = 25;
		Config.LED_Warn = 24;
		
		Config.SDA = 2;
		Config.SCL = 3;
	}
	else
	{
		printf("RPi Model A or B\n\n");

		Config.LED_OK = 4;
		Config.LED_Warn = 11;
		
		Config.SDA = 5;
		Config.SCL = 6;
	}

	LoadConfigFile(&Config);

	if (Config.DisableMonitor)
	{
		system("/opt/vc/bin/tvservice -off");
	}

	GPS.Time = 0.0;
	GPS.Longitude = 0.0;
	GPS.Latitude = 0.0;
	GPS.Altitude = 0;
	GPS.Satellites = 0;
	GPS.Speed = 0.0;
	GPS.Direction = 0.0;
	GPS.InternalTemperature = 0.0;
	GPS.BatteryVoltage = 0.0;
	GPS.ExternalTemperature = 0.0;
	GPS.Pressure = 0.0;
	GPS.BoardCurrent = 0.0;

	
	// Set up I/O
	if (wiringPiSetup() == -1)
	{
		exit (1);
	}

	// Switch on the GPS
	if (!NewBoard())
	{
		pinMode (UBLOX_ENABLE, OUTPUT);
		digitalWrite (UBLOX_ENABLE, 0);
	}
	
	if (!Config.DisableRTTY)
	{
		if (*Config.Frequency)
		{
			SetFrequency(Config.Frequency);
		}
	
		if ((fd = OpenSerialPort()) < 0)
		{
			printf("Cannot open serial port - check documentation!\n");
			exit(1);
		}
		close(fd);
	}
	
	// Switch on the radio
	pinMode (NTX2B_ENABLE, OUTPUT);
	digitalWrite (NTX2B_ENABLE, !Config.DisableRTTY);
	
	// Set up DS18B20
	system("sudo modprobe w1-gpio");
	system("sudo modprobe w1-therm");
	
	// SPI for ADC, LoRa
	system("gpio load spi");
	
	// SSDV Folders
	sprintf(Config.channels[0].SSDVFolder, "%s/RTTY", SSDVFolder);
	*Config.channels[1].SSDVFolder = '\0';										// No folder for APRS images
	sprintf(Config.channels[2].SSDVFolder, "%s/LORA0", SSDVFolder);
	sprintf(Config.channels[3].SSDVFolder, "%s/LORA1", SSDVFolder);
	sprintf(Config.channels[4].SSDVFolder, "%s/FULL", SSDVFolder);
	
	// Create SSDV Folders
	if (stat(SSDVFolder, &st) == -1)
	{
		mkdir(SSDVFolder, 0777);
	}	
	for (i=0; i<5; i++)
	{
		if (*Config.channels[i].SSDVFolder)
		{
			if (stat(Config.channels[i].SSDVFolder, &st) == -1)
			{
				mkdir(Config.channels[i].SSDVFolder, 0777);
			}
		}	
	}

	// Set up full-size image parameters
	Config.channels[4].ImageWidthWhenLow = 2592;
	Config.channels[4].ImageHeightWhenLow = 1944;
	Config.channels[4].ImageWidthWhenHigh = 2592;
	Config.channels[4].ImageHeightWhenHigh = 1944;
	Config.channels[4].ImagePeriod = 60;
	Config.channels[4].ImagePackets = 1;

/*
	if(pthread_create(&SensorThread, NULL, sensors_loop, NULL)){
		fprintf(stderr, "Error creating Sensor Thread\n");
		return 1;
	}
*/
	if (pthread_create(&GPSThread, NULL, GPSLoop, &GPS))
	{
		fprintf(stderr, "Error creating GPS thread\n");
		return 1;
	}
/*
	if (*(Config.APRS_Callsign) && Config.APRS_ID && Config.APRS_Period)
	{
		if (pthread_create(&APRSThread, NULL, APRSLoop, &GPS))
		{
			fprintf(stderr, "Error creating APRS thread\n");
			return 1;
		}
	}
*//*
	if(pthread_create(&LoRaThread, NULL, lora_loop, &GPS)){
		fprintf(stderr, "Error creating LoRa thread\n");
		return 1;
	}
*/	

	/*
	if (Config.LoRaDevices[0].InUse || Config.LoRaDevices[1].InUse)
	{
		if (pthread_create(&LoRaThread, NULL, LoRaLoop, &GPS))
		{
			fprintf(stderr, "Error creating LoRa thread\n");
			return 1;
		}
	} 
	
	if (pthread_create(&DS18B20Thread, NULL, DS18B20Loop, &GPS))
	{
		fprintf(stderr, "Error creating DS18B20s thread\n");
		return 1;
	}
	*/
/*
	if (!Config.LoRaDevices[0].InUse)
	{
		// DO NOT TRY to use SPI ADC (on channel 0) if we are using LoRa channel 0
		if (pthread_create(&ADCThread, NULL, ADCLoop, &GPS))
		{
			fprintf(stderr, "Error creating ADC thread\n");
			return 1;
		}
	}
	*/
	/*
	if (Config.Camera)
	{
		if (pthread_create(&CameraThread, NULL, CameraLoop, &GPS))
		{
			fprintf(stderr, "Error creating camera thread\n");
			return 1;
		}
	}
	*/
	/*
	if (pthread_create(&LEDThread, NULL, LEDLoop, &GPS))
	{
		fprintf(stderr, "Error creating LED thread\n");
		return 1;
	}*/
	/*
	if (Config.EnableBMP085)
	{
		if (pthread_create(&BMP085Thread, NULL, BMP085Loop, &GPS))
		{
			fprintf(stderr, "Error creating BMP085 thread\n");
			return 1;
		}
	}
	*/	
	while (1)
	{	
		if (Config.DisableRTTY)
		{
			delayMilliseconds (200);
		}
		else		
		{
			delay(5000);
			BuildSentence(Sentence, ++Sentence_Counter, &GPS);
			SendSentence(Sentence);
			
			if (Config.channels[0].ImagePackets > 0)
			{
			
				for (i=0; i< ((GPS.Altitude > Config.SSDVHigh) ? Config.channels[0].ImagePackets : 1); i++)
				{
					SendRTTYImage();
				}
			}
			
		}
	}
	return 0;
}