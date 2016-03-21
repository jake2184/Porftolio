#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include "gps.h"
#include "misc.h"

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
