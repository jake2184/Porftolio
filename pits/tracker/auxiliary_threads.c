// To streamline project, combines many of the other c files which are run as
// threads from tracker.c. Other files remain as a backup for eventual removal.
// includes DS18B20.c, adc.c, aprs.c, bmp085.c, led.c, snapper.c


#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <errno.h>
#include <wiringPiSPI.h>
#include <gertboard.h>
#include <ctype.h>
#include <errno.h>   	// Error number definitions
#include <termios.h> 	// POSIX terminal control definitions
#include <stdint.h>
#include <math.h>
#include <linux/i2c-dev.h>
#include <stdarg.h>
#include <wiringPi.h>


#include "gps.h"
#include "misc.h"

#define BMP085_ADDRESS 0x77  		// I2C address of BMP085 pressure sensor


// Camera

int TimeTillImageCompleted(int channel)
{
	// First, if we aren't sending a file, then we need a new one NOW!
	if (Config.channels[channel].ImageFP == NULL || 
	    (Config.channels[channel].SSDVTotalRecords == 0))
	{
		printf ("Convert image now for channel %d!\n", channel);
		return 0;
	}

	// If we're on the last packet, convert anyway
	if (Config.channels[channel].SSDVRecordNumber >= (Config.channels[channel].SSDVTotalRecords-1))
	{
		return 0;
	}
		
	return (Config.channels[channel].SSDVTotalRecords - Config.channels[channel].SSDVRecordNumber) * 256 * 10 / Config.channels[channel].BaudRate;
}

void FindAndConvertImage(int channel)
{
	static char *SubFolder[4] = {"RTTY", "APRS", "LORA0", "LORA1"};
	size_t LargestFileSize;
	char LargestFileName[100], FileName[100], CommandLine[200];
	DIR *dp;
	struct dirent *ep;
	struct stat st;
	char *SSDVFolder;
	
	LargestFileSize = 0;
	SSDVFolder = Config.channels[channel].SSDVFolder;
	
	dp = opendir(SSDVFolder);
	if (dp != NULL)
	{
		while (ep = readdir (dp))
		{
			if (strstr(ep->d_name, ".jpg") != NULL)
			{
				sprintf(FileName, "%s/%s", SSDVFolder, ep->d_name);
				stat(FileName, &st);
				if (st.st_size > LargestFileSize)
				{
					LargestFileSize = st.st_size;
					strcpy(LargestFileName, FileName);
				}
			}
		}
		(void) closedir (dp);
	}

	if (LargestFileSize > 0)
	{
		char Date[20], SavedImageFolder[100], SSDVScriptName[200];
		time_t now;
		struct tm *t;
		FILE *fp;
		
		printf("Found file %s to convert\n", LargestFileName);
		
		// Now create a script to convert the file
		Config.channels[channel].SSDVFileNumber++;
		Config.channels[channel].SSDVFileNumber = Config.channels[channel].SSDVFileNumber & 255;
		
		// Set name of SSDV output file
		sprintf(Config.channels[channel].SSDVFileName, "/home/pi/pits/tracker/snap_%d_%d.bin", channel, Config.channels[channel].SSDVFileNumber);
		
		// Set name of SSDV script file
		sprintf(SSDVScriptName, "/home/pi/pits/tracker/convert_%d", channel);
				
		// Create SSDV script file
		if ((fp = fopen(SSDVScriptName, "wt")) != NULL)
		{
			fprintf(fp, "ssdv -e -c %s -i %d %s %s\n", Config.channels[channel].PayloadID, Config.channels[channel].SSDVFileNumber, LargestFileName, Config.channels[channel].SSDVFileName);
			fprintf(fp, "mkdir -p %s/$1\n", SSDVFolder);
			fprintf(fp, "mv %s/*.jpg %s/$1\n", SSDVFolder, SSDVFolder);
			fclose(fp);
			chmod(SSDVScriptName, S_IRUSR | S_IWUSR | S_IXUSR | S_IRGRP | S_IWGRP | S_IXGRP | S_IROTH | S_IWOTH | S_IXOTH); 
		}
		Config.channels[channel].NextSSDVFileReady = 1;
	}
}

void *CameraLoop(void *some_void_ptr)
{
	int width, height;
	struct TGPS *GPS;
	char filename[100];
	int channel;
	FILE *fp;

	GPS = (struct TGPS *)some_void_ptr;
	
	for (channel=0; channel<5; channel++)
	{
		Config.channels[channel].TimeSinceLastImage = Config.channels[channel].ImagePeriod;
	}

	while (1)
	{
		for (channel=0; channel<5; channel++)
		{
			if (Config.channels[channel].Enabled && (Config.channels[channel].ImagePackets > 0))
			{
				// channel using SSDV
				
				if (++Config.channels[channel].TimeSinceLastImage >= Config.channels[channel].ImagePeriod)
				{
					// Time to take a photo on this channel

					Config.channels[channel].TimeSinceLastImage = 0;
					
					if (GPS->Altitude >= Config.SSDVHigh)
					{
						width = Config.channels[channel].ImageWidthWhenHigh;
						height = Config.channels[channel].ImageHeightWhenHigh;
					}
					else
					{
						width = Config.channels[channel].ImageWidthWhenLow;
						height = Config.channels[channel].ImageHeightWhenLow;
					}

					// Create name of file
					sprintf(filename, "/home/pi/pits/tracker/take_pic_%d", channel);
					
					// Leave it alone if it exists (this means that the photo has not been taken yet)
					if (access(filename, F_OK ) == -1)
					{				
						// Doesn't exist, so create it.  Script will run it next time it checks
						if ((fp = fopen(filename, "wt")) != NULL)
						{
							if (channel == 4)
							{
								// Full size images are saved in dated folder names
								fprintf(fp, "mkdir -p %s/$2\n", Config.channels[channel].SSDVFolder);
								fprintf(fp, "raspistill -w %d -h %d -t 3000 -ex auto -mm matrix -o %s/$2/$1.jpg\n", width, height, Config.channels[channel].SSDVFolder);
							}
							else
							{
								fprintf(fp, "raspistill -w %d -h %d -t 3000 -ex auto -mm matrix -o %s/$1.jpg\n", width, height, Config.channels[channel].SSDVFolder);
							}
							
							fclose(fp);
							chmod(filename, S_IRUSR | S_IWUSR | S_IXUSR | S_IRGRP | S_IWGRP | S_IXGRP | S_IROTH | S_IWOTH | S_IXOTH); 
						}
					}
					Config.channels[channel].ImagesRequested++;
				}
				
				// Now check if we need to convert the "best" image before the current SSDV file is fully sent
				
				if (Config.channels[channel].ImagesRequested && !Config.channels[channel].NextSSDVFileReady)
				{
					if (TimeTillImageCompleted(channel) < 5)
					{
						FindAndConvertImage(channel);
					}
				}
			}
		}
		
		sleep(1);
	}
}


// LED

void *LEDLoop(void *some_void_ptr)
{
	struct TGPS *GPS;
	int Flash;

	GPS = (struct TGPS *)some_void_ptr;

	// We have 2 LED outputs
	pinMode (Config.LED_Warn, OUTPUT);
	pinMode (Config.LED_OK, OUTPUT);
	
	while (1)
	{
		digitalWrite (Config.LED_Warn, GPS->Satellites < 4);	
		digitalWrite (Config.LED_OK, (GPS->Satellites >= 4) && (GPS->Altitude < 2000) && (Flash ^= 1));	

		sleep(1);
	}

	return 0;
}


// GPS Temperature

void *DS18B20Loop(void *some_void_ptr)
{
    DIR *dir;
    struct dirent *dp;
	char *folder = "/sys/bus/w1/devices";
	FILE *fp;
	char line[100], filename[100];
	char *token, *value;
	float Temperature;
	struct TGPS *GPS;

	GPS = (struct TGPS *)some_void_ptr;

	while (1)
	{
		// printf ("DS18B20...\n");
		if ((dir = opendir(folder)) != NULL)
		{
			// printf("Opened folder\n");
			while ((dp = readdir(dir)) != NULL)
			{
				if (strlen(dp->d_name) > 3)
				{
					if (dp->d_name[2] == '-')
					{
						sprintf(filename, "%s/%s/w1_slave", folder, dp->d_name);
						if ((fp = fopen(filename, "r")) != NULL)
						{
							// 44 02 4b 46 7f ff 0c 10 ee : crc=ee YES
							// 44 02 4b 46 7f ff 0c 10 ee t=36250
							if (fgets(line, sizeof(line), fp) != NULL)
							{
								if (strstr(line, "YES") != NULL)
								{
									if (fgets(line, sizeof(line), fp) != NULL)
									{
										token = strtok(line, "=");
										value = strtok(NULL, "\n");
										Temperature = atof(value) / 1000;
										// printf("%5.3fC\n", Temperature);
										GPS->InternalTemperature = Temperature;
									}
								}
							}
							
							fclose(fp);
						}
						else
						{
							printf("COULD NOT OPEN DS18B20 FILE\n");
						}
					}
				}
			}
			closedir(dir);
		}
		else
		{
			printf("COULD NOT OPEN DS18B20 FOLDER\n");
		}
		sleep(5);
	}
}

// GPS Pressure and Temperature

struct TBMP
{
	short fd;
	short ac1;
	short ac2; 
	short ac3; 
	unsigned short ac4;
	unsigned short ac5;
	unsigned short ac6;
	short B1; 
	short B2;
	short Mb;
	short Mc;
	short Md;
};

void bmp085Calibration(struct TBMP *bmp);
double bmp085GetTemperature(struct TBMP *bmp);
double bmp085GetPressure(struct TBMP *bmp, double Temperature);
short bmp085ReadInt(short fd, unsigned char address);
unsigned short bmp085ReadUT(short fd);
double bmp085ReadUP(short fd);


short open_i2c(int address)
{
	short fd;
	char i2c_dev[16];

	sprintf(i2c_dev, "/dev/i2c-%d", piBoardRev()-1);

	if ((fd = open(i2c_dev, O_RDWR)) < 0)
	{                                        // Open port for reading and writing
		printf("Failed to open i2c port\n");
		return 0;
	}

	if (ioctl(fd, I2C_SLAVE, address) < 0)                                 // Set the port options and set the address of the device we wish to speak to
	{
		printf("Unable to get bus access to talk to slave on address %02Xh\n", address);
		return 0;
	}

	return fd;
}

// External Temp/Pressure

void *BMP085Loop(void *some_void_ptr)
{
	struct TBMP bmp;
	struct TGPS *GPS;

	GPS = (struct TGPS *)some_void_ptr;

	// Initialise BMP085
	if (bmp.fd = open_i2c(BMP085_ADDRESS))
	{
		bmp085Calibration(&bmp);
		close(bmp.fd);
	}
	else
	{
		return 0;
	}
	
	while (1)
	{
		if (bmp.fd = open_i2c(BMP085_ADDRESS))
		{
			GPS->ExternalTemperature = bmp085GetTemperature(&bmp);
			GPS->Pressure = bmp085GetPressure(&bmp, GPS->ExternalTemperature);

			// printf("Temperature is %5.2lf\n", GPS->ExternalTemperature);
			// printf("Pressure is %5.2lf\n", GPS->Pressure);

			close(bmp.fd);
		}

		sleep(10);
	}

    return 0;
}

void bmp085Calibration(struct TBMP *bmp)
{
	bmp->ac1 = bmp085ReadInt(bmp->fd, 0xAA);
	bmp->ac2 = bmp085ReadInt(bmp->fd, 0xAC);
	bmp->ac3 = bmp085ReadInt(bmp->fd, 0xAE);
	bmp->ac4 = bmp085ReadInt(bmp->fd, 0xB0);
	bmp->ac5 = bmp085ReadInt(bmp->fd, 0xB2);
	bmp->ac6 = bmp085ReadInt(bmp->fd, 0xB4);
	bmp->B1 = bmp085ReadInt(bmp->fd, 0xB6);
	bmp->B2 = bmp085ReadInt(bmp->fd, 0xB8);
	bmp->Mb = bmp085ReadInt(bmp->fd, 0xBA);
	bmp->Mc = bmp085ReadInt(bmp->fd, 0xBC);
	bmp->Md = bmp085ReadInt(bmp->fd, 0xBE);

	// printf ("Values are %d %d %d %u %u %u %d %d %d %d %d\n", ac1, ac2, ac3, ac4, ac5, ac6, B1, B2, Mb, Mc, Md);
}

// Calculate temperature given ut.
// Value returned will be in units of 0.1 deg C
double bmp085GetTemperature(struct TBMP *bmp)
{
	double c5, alpha, mc, md;
	unsigned short ut;

	ut = bmp085ReadUT(bmp->fd);

	c5 = (double)bmp->ac5 / (32768 * 160);

	alpha = c5 * (double)(ut - bmp->ac6);

	mc = 2048 * bmp->Mc / (160 * 160);

	md = (double)bmp->Md / 160;

	return alpha + mc / (alpha + md);
}

// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
double bmp085GetPressure(struct TBMP *bmp, double Temperature)
{
	double Pu, s, x0, x1, x2, c3, c4, b1, y0, y1, y2, p0, p1, p2, x, y, z, P;

	Pu = bmp085ReadUP(bmp->fd);
	// printf("Pu = %lf\n", Pu);

	s = Temperature - 25;
	x0 = bmp->ac1;
	x1 = (double)bmp->ac2 * 160 / 8192;
	x2 = (double)bmp->B2 * 160 * 160 / 33554432;
	c3 = (double)bmp->ac3 * 160 / 32768;
	c4 = (double)bmp->ac4 / 32768000;
	b1 = (double)bmp->B1 * 160 * 160 /1073741824;
	y0 = c4 * 32768;
	y1 = c4 * c3;
	y2 = c4 * b1;
	p0 = (3791.0 - 8.0) / 1600.0;
	p1 = 1.0 - 7357.0 / 1048576.0;
	p2 = 303800.0 / 68719476740.0;
	x = x2 * s * s + x1 * s + x0;
	y = y2 * s * s + y1 * s + y0;
	z = (Pu - x) / y;
	P = p2 * z * z + p1 * z + p0;


	return P;
}


// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
short bmp085ReadInt(short fd, unsigned char address)
{
	unsigned char buf[10];

	buf[0] = address;

	if ((write(fd, buf, 1)) != 1) {								// Send register we want to read from	
		printf("Error writing to i2c slave\n");
		exit(1);
	}
	
	if (read(fd, buf, 2) != 2) {								// Read back data into buf[]
		printf("Unable to read from slave\n");
		exit(1);
	}

	return (short) buf[0]<<8 | buf[1];
}

// Read the uncompensated temperature value
unsigned short bmp085ReadUT(short fd)
{
 	unsigned short ut;
	unsigned char buf[10];
  
  // Write 0x2E into Register 0xF4
  // This requests a temperature reading

	buf[0] = 0xF4;
	buf[1] = 0x2E;

	if ((write(fd, buf, 2)) != 2) {								// Send register we want to read from	
		printf("Error writing to i2c slave\n");
		exit(1);
	}

	usleep(5000);
	
	ut = bmp085ReadInt(fd, 0xF6);

	// printf("ut = %u\n", ut);

	return ut;
}

// Read the uncompensated pressure value
double bmp085ReadUP(short fd)
{
  unsigned char msb, lsb, xlsb;
  double up;
	unsigned char buf[10];
  
  // Write 0x34 into register 0xF4
  // Request a pressure reading w/ oversampling setting

	buf[0] = 0xF4;
	buf[1] = 0x34;

	if ((write(fd, buf, 2)) != 2) {								// Send register we want to read from	
		printf("Error writing to i2c slave\n");
		exit(1);
	}

	usleep(30000);

	buf[0] = 0xF6;

	if ((write(fd, buf, 1)) != 1) {								// Send register we want to read from	
		printf("Error writing to i2c slave\n");
		exit(1);
	}
	
	if (read(fd, buf, 3) != 3) {								// Read back data into buf[]
		printf("Unable to read from slave\n");
		exit(1);
	}

	msb = buf[0];
	lsb = buf[1];
	xlsb = buf[2];

	// printf("Pressure values are %02X %02X %02X\n", msb, lsb, xlsb);

	up = (double)msb * 256 + (double)lsb + (double)xlsb / 256;


	return up;
}



// Sine wave table for tone generation
uint8_t _sine_table[] = {
#include "sine_table.h"
};

typedef unsigned int	UI;
typedef unsigned long int	UL;
typedef unsigned short int	US;
typedef unsigned char	UC;
typedef signed int		SI;
typedef signed long int	SL;
typedef signed short int	SS;
typedef signed char	SC;
 
#define attr(a) __attribute__((a))
 
#define packed attr(packed)
 
/* WAV header, 44-byte total */
typedef struct{
 UL riff	packed;
 UL len	packed;
 UL wave	packed;
 UL fmt	packed;
 UL flen	packed;
 US one	packed;
 US chan	packed;
 UL hz	packed;
 UL bpsec	packed;
 US bpsmp	packed;
 US bitpsmp	packed;
 UL dat	packed;
 UL dlen	packed;
}WAVHDR;
 
// APRS / AFSK variables
  
 
/* "converts" 4-char string to long int */
#define dw(a) (*(UL*)(a))


static uint8_t *_ax25_callsign(uint8_t *s, char *callsign, char ssid)
{
  char i;
  for(i = 0; i < 6; i++)
  {
    if(*callsign) *(s++) = *(callsign++) << 1;
    else *(s++) = ' ' << 1;
  }
  *(s++) = ('0' + ssid) << 1;
  return(s);
}

char *ax25_base91enc(char *s, uint8_t n, uint32_t v)
{
  /* Creates a Base-91 representation of the value in v in the string */
  /* pointed to by s, n-characters long. String length should be n+1. */

  for(s += n, *s = '\0'; n; n--)
  {
    *(--s) = v % 91 + 33;
    v /= 91;
  }

  return(s);
}

uint8_t *ax25_frame(int *length, char *scallsign, char sssid, char *dcallsign, char dssid, char *path1, char ttl1, char *path2, char ttl2, char *data, ...)
{
	static uint8_t frame[200];
  uint8_t *s, j;
  uint16_t CRC;
  va_list va;

  va_start(va, data);

  /* Write in the callsigns and paths */
  s = _ax25_callsign(frame, dcallsign, dssid);
  s = _ax25_callsign(s, scallsign, sssid);
  if(path1) s = _ax25_callsign(s, path1, ttl1);
  if(path2) s = _ax25_callsign(s, path2, ttl2);

  /* Mark the end of the callsigns */
  s[-1] |= 1;

  *(s++) = 0x03; /* Control, 0x03 = APRS-UI frame */
  *(s++) = 0xF0; /* Protocol ID: 0xF0 = no layer 3 data */

  // printf ("Length A is %d\n", s - frame);
  
  // printf ("Adding %d bytes\n", 
  vsnprintf((char *) s, 200 - (s - frame) - 2, data, va);
  va_end(va);

  // printf ("Length B is %d\n", strlen(frame));
  
	// Calculate and append the checksum

	for (CRC=0xffff, s=frame; *s; s++)
    {
		CRC ^= ((unsigned int)*s);
		
        for (j=0; j<8; j++)
        {
			if (CRC & 1)
			{
				CRC = (CRC >> 1) ^ 0x8408;
			}
			else
			{
				CRC >>= 1;
			}
        }
    }
	/* 
	for (CRC=0xffff, s=frame; *s; s++)
    {
  	  CRC = ((CRC) >> 8) ^ ccitt_table[((CRC) ^ *s) & 0xff];
	}
	*/
	
	*(s++) = ~(CRC & 0xFF);
	*(s++) = ~((CRC >> 8) & 0xFF);
  
	// printf ("Checksum = %02Xh %02Xh\n", *(s-2), *(s-1));

	*length = s - frame;
  
	return frame;
}

/* Makes 44-byte header for 8-bit WAV in memory
usage: wavhdr(pointer,sampleRate,dataLength) */
 
void wavhdr(void*m,UL hz,UL dlen){
 WAVHDR*p=m;
 p->riff=dw("RIFF");
 p->len=dlen+44;
 p->wave=dw("WAVE");
 p->fmt=dw("fmt ");
 p->flen=0x10;
 p->one=1;
 p->chan=1;
 p->hz=hz;
 p->bpsec=hz;
 p->bpsmp=1;
 p->bitpsmp=16;	// 8
 p->dat=dw("data");
 p->dlen=dlen;
}

void make_and_write_freq(FILE *f, UL cycles_per_bit, UL baud, UL lfreq, UL hfreq, int8_t High)
{
	// write 1 bit, which will be several values from the sine wave table
	static uint16_t phase  = 0;
	uint16_t step;
	int i;

	if (High)
	{
		step = (512 * hfreq << 7) / (cycles_per_bit * baud);
		// printf("-");
	}
	else
	{
		step = (512 * lfreq << 7) / (cycles_per_bit * baud);
		// printf("_");
	}
	
	for (i=0; i<cycles_per_bit; i++)
	{
		// fwrite(&(_sine_table[(phase >> 7) & 0x1FF]), 1, 1, f);
		int16_t v = _sine_table[(phase >> 7) & 0x1FF] * 0x80 - 0x4000;
		fwrite(&v, 2, 1, f);
		phase += step;
	}
}

void make_and_write_bit(FILE *f, UL cycles_per_bit, UL baud, UL lfreq, UL hfreq, unsigned char Bit, int BitStuffing)
{
	static int8_t bc = 0;
	static int8_t High = 0;
			
	/*
	if (Bit)
	{
		// Stay with same frequency, but only for a max of 5 in a row
		bc++;
	}
	else
	{
		// 0 means swap frequency
		High = !High;
		bc = 0;
	}
	
	make_and_write_freq(f, cycles_per_bit, baud, lfreq, hfreq, High);

	if (BitStuffing)
	{
		if (bc >= 4)
		{	
			High = !High;
			make_and_write_freq(f, cycles_per_bit, baud, lfreq, hfreq, High);
			bc = 0;
		}
	}
	else
	{
		bc = 0;
	}
	*/
	if(BitStuffing)
	{
		if(bc >= 5)
		{
			High = !High;
			make_and_write_freq(f, cycles_per_bit, baud, lfreq, hfreq, High);
			bc = 0;
		}
	}
	else
	{
		bc = 0;
	}
	
	if (Bit)
	{
		// Stay with same frequency, but only for a max of 5 in a row
		bc++;
	}
	else
	{
		// 0 means swap frequency
		High = !High;
		bc = 0;
	}
	
	make_and_write_freq(f, cycles_per_bit, baud, lfreq, hfreq, High);	
}

 
void make_and_write_byte(FILE *f, UL cycles_per_bit, UL baud, UL lfreq, UL hfreq, unsigned char Character, int BitStuffing)
{
	int i;
	
	// printf("%02X ", Character);
		
	for (i=0; i<8; i++)
	{
		make_and_write_bit(f, cycles_per_bit, baud, lfreq, hfreq, Character & 1, BitStuffing);
		Character >>= 1;
	}
}

 
/* makes wav file */
void makeafsk(UL freq, UL baud, UL lfreq, UL hfreq, unsigned char *Message, int message_length)
{
	UL preamble_length, postamble_length, flags_before, flags_after, cycles_per_bit, cycles_per_byte, total_cycles;
	UC* m;
	FILE *f;
	int i;
	
	f = fopen("aprs.wav","wb");
	
	cycles_per_bit = freq / baud;
	printf ("cycles_per_bit=%d\n", cycles_per_bit);
	cycles_per_byte = cycles_per_bit * 8;
	printf ("cycles_per_byte=%d\n", cycles_per_byte);

	preamble_length = 128;
	postamble_length = 64;
	flags_before = 32;
	flags_after = 32;

	// Calculate size of file
	total_cycles = (cycles_per_byte * (flags_before + message_length + flags_after)) + ((preamble_length + postamble_length) * cycles_per_bit);

	// Make header
	m = malloc(44);
	wavhdr(m, freq, total_cycles * 2 + 10);		// * 2 + 10 is new
	
	// Write wav header
	fwrite(m, 1, 44, f);
	
	// Write preamble
	/*
	for (i=0; i< preamble_length; i++)
	{
		make_and_write_freq(f, cycles_per_bit, baud, lfreq, hfreq, 0);
	}
	*/
	
	for (i=0; i<flags_before; i++)
	{
		make_and_write_byte(f, cycles_per_bit, baud, lfreq, hfreq, 0x7E, 0);
	}
	
	// Create and write actual data
	for (i=0; i<message_length; i++)
	{
		make_and_write_byte(f, cycles_per_bit, baud, lfreq, hfreq, Message[i], 1);
	}

	for (i=0; i<flags_after; i++)
	{
		make_and_write_byte(f, cycles_per_bit, baud, lfreq, hfreq, 0x7E, 0);
	}

	// Write postamble
	for (i=0; i< postamble_length; i++)
	{
		make_and_write_freq(f, cycles_per_bit, baud, lfreq, hfreq, 0);
	}
	
	fclose(f);
}

	
void SendAPRS(struct TGPS *GPS)
{
	static Count=0;
	char stlm[9];
	char slat[5];
	char slng[5];
	char comment[3]={' ', ' ', '\0'};
	double aprs_lat, aprs_lon;
	int32_t aprs_alt;
	static uint16_t seq = 0;
	uint8_t *frame;
	int i, length;

	/*
	lat = 51.95023;
	lon = -2.54445;
	alt = 190;
	*/
	
	Count++;
	seq++;
	
	// Convert the min.dec coordinates to APRS compressed format
	aprs_lat = 900000000 - GPS->Latitude * 10000000;
	aprs_lat = aprs_lat / 26 - aprs_lat / 2710 + aprs_lat / 15384615;
	aprs_lon = 900000000 + GPS->Longitude * 10000000 / 2;
	aprs_lon = aprs_lon / 26 - aprs_lon / 2710 + aprs_lon / 15384615;
	aprs_alt = GPS->Altitude * 32808 / 10000;


	// Construct the compressed telemetry format
	ax25_base91enc(stlm + 0, 2, seq);
	
    frame = ax25_frame(&length,
    Config.APRS_Callsign,
	Config.APRS_ID,
    "APRS", 0,
    "WIDE1", 1, "WIDE2",1,
    "!/%s%sO   /A=%06ld|%s|%s/%s,%d'C,http://www.pi-in-the-sky.com",
    ax25_base91enc(slat, 4, aprs_lat),
    ax25_base91enc(slng, 4, aprs_lon),
    aprs_alt, stlm, comment, Config.APRS_Callsign, Count);	// , errorstatus,temperature1);

	printf("Length=%d\n\n", length);

	makeafsk(48000, 1200, 1200, 2200, frame, length);
}

void *APRSLoop(void *some_void_ptr)
{
	struct TGPS *GPS;

	GPS = (struct TGPS *)some_void_ptr;
	
    while (1)
	{
		if (GPS->Satellites > 3)
		{
			printf("Sending APRS message ...\n");
			
			SendAPRS(GPS);
			
			sleep(60 * Config.APRS_Period);
		}
		else
		{
			sleep(1);
		}
	}
}


// ADC

int AnalogRead (int chan)
{
  unsigned char spiData [3] ;
  unsigned char chanBits ;

  chanBits = 0xC0 | ((chan & 1) << 5);

  spiData[0] = chanBits ;
  spiData[1] = 0;
  spiData[2] = 0;

  wiringPiSPIDataRW (0, spiData, 3) ;

  return ((spiData [0] << 7) | (spiData [1] >> 1)) & 0x3FF ;
}

double ReadADC(int chan, double FullScale)
{
	int RawValue;
    double Result;

   	RawValue = AnalogRead(chan);
	Result = (double)RawValue * FullScale / 1024.0;
	
	return Result;
}


void *ADCLoop(void *some_void_ptr)
{
	double BatteryVoltage, BoardCurrent;
	FILE *fp;
	struct TGPS *GPS;

	GPS = (struct TGPS *)some_void_ptr;

    // printf("Opening SPI ...\n");

    if (gertboardSPISetup () < 0)
    {
        printf("Failed to setup SPI\n");
    }
		
	while (1)
	{
		BatteryVoltage = ReadADC(0, 6.67);
		GPS->BatteryVoltage = BatteryVoltage;
		// printf("BatteryVoltage = %lf\n", BatteryVoltage);

		if (NewBoard())
		{
			BoardCurrent = ReadADC(1, 14);
			GPS->BoardCurrent = (unsigned int)(fmax(0.0, BoardCurrent * 1000.0));
		}

		sleep(10);
	}

	return 0;
}