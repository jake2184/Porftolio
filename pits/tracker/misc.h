// Globals
#ifndef MISC_H
#define MISC_H

#include <termios.h>
#include <curses.h>

// Structure for the LoRa devices.  The LoRa board has 1 or 2 LoRa modems.
// Current labellibng is "1" for SPI channel 0 and "2" for SPI channel 1
// If the board is used with PITS then only "2" can be used (though both can be populated)


/*
struct TLoRaDevice
{
	int InUse;
	int DIO0;
	int DIO5;
	char Frequency[8];
	int SpeedMode;
	int Power;
	int PayloadLength;
	int ImplicitOrExplicit;
	int ErrorCoding;
	int Bandwidth;
	int SpreadingFactor;
	int LowDataRateOptimize;
	int CycleTime;
	int Slot;
	int RepeatSlot;
	int UplinkSlot;
	int Binary;
	int LastTxAt;
	int LastRxAt;
	int AirCount;
	int GroundCount;
	int BadCRCCount;
	char LastCommand[128];
	unsigned char PacketToRepeat[256];
	unsigned char UplinkPacket[256];
	int PacketRepeatLength;
	int UplinkRepeatLength;
	int SendRepeatedPacket;
	tLoRaMode LoRaMode;
	bool ackReceived;
	bool tweetReceived;
	bool commandReceived;

	WINDOW* Window;

	unsigned int TelemetryCount, SSDVCount, UnknownCount, SSDVMissing, SentPackets;
	
	char Payload[16], Time[12];
	unsigned int Counter;
	unsigned long Seconds;
	double Longitude, Latitude;
	unsigned int Altitude, PreviousAltitude;
	unsigned int Satellites;
	unsigned long LastPositionAt;
	time_t LastPacketAt;
	float AscentRate;

	char logging[16];
	double Reference;
};
*/
// Structure for all possible radio devices
// 0 is RTTY
// 1 is APRS
// 2/3 are for LoRa
struct TChannel
{
	int Enabled;
	unsigned int SentenceCounter;
	char PayloadID[16];
	int SendTelemetry;						// TRUE to send telemetry on this channel
	char SSDVFolder[200];
	int ImagePackets;						// Image packets per telemetry packet
	int ImagePacketCount;					// Image packets since last telemetry packet
	int ImageWidthWhenLow;
	int ImageHeightWhenLow;
	int ImageWidthWhenHigh;
	int ImageHeightWhenHigh;
	int ImagePeriod;						// Time in seconds between photographs
	int	TimeSinceLastImage;
	unsigned int BaudRate;
	char SSDVFileName[200];
	FILE *ImageFP;
	int SSDVRecordNumber;
	int SSDVTotalRecords;
	int NextSSDVFileReady;
	int SSDVFileNumber;
	int ImagesRequested;

};

#define RTTY_CHANNEL 0
#define APRS_CHANNEL 1
#define LORA_CHANNEL 2

struct TConfig
{
	int DisableMonitor;
	int Camera;
	int SSDVHigh;
	int EnableBMP085;
	int EnableGPSLogging;
	int EnableTelemetryLogging;
	int LED_OK;
	int LED_Warn;
	
	// GPS Settings
	int SDA;
	int SCL;
	char GPSDevice[32];
	
	// RTTY Settings
	int DisableRTTY;
	char Frequency[8];
	speed_t TxSpeed;
	
	char APRS_Callsign[16];
	int APRS_ID;
	int APRS_Period;
		
	//struct TLoRaDevice LoRaDevices[2];
	
	struct TChannel channels[5];		// 0 is RTTY, 1 is APRS, 2/3 are LoRa, 4 is for full-size images

	int ground;
	char Tracker[16];
	int EnableHabitat;
	int EnableSSDV;
	char ftpServer[100];
	char ftpUser[32];
	char ftpPassword[32];
	char ftpFolder[64];
	
	int LastCommandNo;


};

struct TPayload
{
	int InUse;
	char Payload[32];
};

extern struct TConfig Config;
extern char *SSDVFolder;

char Hex(char Character);
void WriteLog(char *FileName, char *Buffer);


#endif