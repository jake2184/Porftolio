#ifndef LORA_H
#define LORA_H

#define REG_FIFO                    0x00
#define REG_FIFO_ADDR_PTR           0x0D 
#define REG_FIFO_TX_BASE_AD         0x0E
#define REG_FIFO_RX_BASE_AD         0x0F
#define REG_RX_NB_BYTES             0x13
#define REG_OPMODE                  0x01
#define REG_FIFO_RX_CURRENT_ADDR    0x10
#define REG_IRQ_FLAGS               0x12
#define REG_PACKET_SNR				0x19
#define REG_PACKET_RSSI				0x1A
#define REG_CURRENT_RSSI			0x1B
#define REG_DIO_MAPPING_1           0x40
#define REG_DIO_MAPPING_2           0x41
#define REG_MODEM_CONFIG            0x1D
#define REG_MODEM_CONFIG2           0x1E
#define REG_MODEM_CONFIG3           0x26
#define REG_PAYLOAD_LENGTH          0x22
#define REG_IRQ_FLAGS_MASK          0x11
#define REG_HOP_PERIOD              0x24
#define REG_FREQ_ERROR				0x28
#define REG_DETECT_OPT				0x31
#define	REG_DETECTION_THRESHOLD		0x37
#define REG_SYMB_TIMEOUT			0x1F

// MODES
#define RF98_MODE_RX_CONTINUOUS     0x85
#define RF98_MODE_TX                0x83
#define RF98_MODE_SLEEP             0x80
#define RF98_MODE_STANDBY           0x81
#define RF98_MODE_RX_SINGLE			0x86

#define PAYLOAD_LENGTH              255

// Modem Config 1
#define EXPLICIT_MODE               0x00
#define IMPLICIT_MODE               0x01

#define ERROR_CODING_4_5            0x02
#define ERROR_CODING_4_6            0x04
#define ERROR_CODING_4_7            0x06
#define ERROR_CODING_4_8            0x08

#define BANDWIDTH_7K8               0x00
#define BANDWIDTH_10K4              0x10
#define BANDWIDTH_15K6              0x20
#define BANDWIDTH_20K8              0x30
#define BANDWIDTH_31K25             0x40
#define BANDWIDTH_41K7              0x50
#define BANDWIDTH_62K5              0x60
#define BANDWIDTH_125K              0x70
#define BANDWIDTH_250K              0x80
#define BANDWIDTH_500K              0x90

// Modem Config 2

#define SPREADING_6                 0x60
#define SPREADING_7                 0x70
#define SPREADING_8                 0x80
#define SPREADING_9                 0x90
#define SPREADING_10                0xA0
#define SPREADING_11                0xB0
#define SPREADING_12                0xC0

#define CRC_OFF                     0x00
#define CRC_ON                      0x04

// POWER AMPLIFIER CONFIG
#define REG_PA_CONFIG               0x09
#define PA_MAX_BOOST                0x8F
#define PA_LOW_BOOST                0x81
#define PA_MED_BOOST                0x8A
#define PA_MAX_UK                   0x88
#define PA_OFF_BOOST                0x00
#define RFO_MIN                     0x00

// LOW NOISE AMPLIFIER
#define REG_LNA                     0x0C
#define LNA_MAX_GAIN                0x23  // 0010 0011
#define LNA_OFF_GAIN                0x00
#define LNA_LOW_GAIN                0xC0  // 1100 0000

// Pin allocations.  Do not change unless you're using your own hardware
#define NTX2B_ENABLE	0
#define UBLOX_ENABLE	2

#include <stdint.h>
#include <curses.h>
//#include "MQTTClient.h"
#include "gps.h"
#include <sys/time.h>

typedef enum {lmIdle, lmListening, lmSending} tLoRaMode;

struct MQTTSructure{
	bool upload;
	char address[100];
	char clientID[100];
	char username[100];
	char password[100];
	char topic[100];
	//MQTTClient client;
};


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

struct Channel
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

// Structure for all possible radio devices
// 0 is RTTY
// 1 is APRS
// 2/3 are for LoRa




struct TLoRaConfiguration
{
	int EnableTelemetryLogging;
	
	struct TLoRaDevice LoRaDevices[2];
	
	struct Channel channels[2];	

	int ground;
	char Tracker[16];
	int EnableHabitat;
	int EnableSSDV;
	
	char ftpServer[100];
	char ftpUser[32];
	char ftpPassword[32];
	char ftpFolder[64];
	
	int LastCommandNo;

	struct MQTTSructure MQTTSettings;

};


extern char **LoRaModes;
extern struct TLoRaConfiguration LoraConfig;

//////////////////////////
// Setting Up Functions //
//////////////////////////

// Call before all other functions. Loads "config.txt" with all settings
void loadConfigFile();

// Set mode of LoRa Module, to send and receive.
void setMode(int channel, uint8_t newMode);

// Initialise the modules with settings from loadConfigFile()
void setupRFM98(int channel);


///////////////////////
// Display Functions //
///////////////////////

// Call to initialise window
WINDOW * InitDisplay(void);

// Call to close window and free memory
void CloseDisplay(WINDOW * mainwin);

// Log Message. Prints either to stout or Window
// Can optionally be made to log to file. (to do)
void LogMessage(const char *format, ...);

// Prints the current configuration for LoRa Devices
// (According to the settings, not the chips themselves)
void printLoRaSettings();



/////////////////////////
// Packet Construction //
/////////////////////////

// Original function from PITS
int BuildLoRaSentence(char *TxLine, int channel, struct TGPS *GPS);

// Our function, adapted from above, to include sensor data
int BuildLoRaPacket(char *TxLine, int channel, struct TGPS *GPS);

// Perform CRC and append to message
int BuildLoRaString(char *TxLine, char* message, int channel);

// Build binary position packet
int BuildLoRaPositionPacket(char *TxLine, int channel, struct TGPS *GPS);



//////////////////////
// Receive and Send //
//////////////////////

// Set up the modules to start receiving
void startReceiving(int channel);

// Receive and process messages for a set time 
void receiveForSetTime(double duration);

// Receive and process messages while(1). Blocking. 
void constantReceiving();

// Send data over a LoRa channel. Buffer should have a CRC check at the end, 
// appended by a Build function
void SendLoRaData(int channel, unsigned char *buffer, int Length);

// Erm.
int SendLoRaImage(int LoRachannel);

// Read a tweet from a file and send it. 
void sendTweet(int channel);

// Send a command line. Will send and wait for an ack. If no ack received,
// will try again for 'tries' repeats. 
int sendCommandLine(int channel, unsigned char *commandLine, int length, int tries);

// Read commandline from a file and send using function above
void readAndSendCommandLine(int channel, char *filename, int tries);


////////////////////
// Misc Functions //
////////////////////

// Send a 'request' message to the ground, requesting a tweet message
bool requestTweet(int channel);

// Reads a tweet from a file, and displays on our screen
void tweetFromFile(FILE* tweetFile, int lineNumber);



////////////////////
// Loop Functions //

// A loop for sending data/handling commandlines and tweets
void *lora_loop(void *some_void_ptr);

// Original Function from PITS. 
void *LoRaLoop(void *some_void_ptr);

#endif