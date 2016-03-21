// Supplied for access to external sensor object, Sensors, and the function
// to update the values, sensors_loop

#ifndef SENSORS_H
#define SENSORS_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>


typedef struct Sensors{
    short fd;
    unsigned short c[6];
    signed short data[6];               //raw acceleration data
    int tref; 
    int64_t st1;
	unsigned int ut;
	int up;
	int dT;
	int64_t off;
	int64_t sens;
	int htemp;
	
	int pressure;
	float humidity;
	float temp;
	
	float x;                          //x- axis acceleration 
	float y;                       
	float z;
	float infrared;
	float light;
}Sensors;

int initialise_sensors(void);
void update_sensors(void);

void *sensors_loop();

struct Sensors sensors;

short fd;

#endif