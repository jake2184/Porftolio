// Operates external sensors, updating an object's values with sensors_loop

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <errno.h>
#include <wiringPiSPI.h>
#include "sen.h"

#define MS5607_ADDRESS 0x77
#define HTU20D_ADDRESS 0x40
#define MS5607_RESET 0x1E
#define HTU20D_RESET 0xFE
#define MS5607_COEFF 0xA2
#define HTU20D_HUMID 0xF5
#define MS5607_PRESS 0x48
#define MS5607_TEMP 0x58
#define MS5607_READ 0x00
#define ADXL345_ADDRESS         0x53
#define ADXL345_POWER_CTL       0x2D
#define MEASURE                 0x08
#define AXES_DATA               0x32
#define TSL2561_ADDRESS         0x39
#define TSL2561_CONTROL         0x80
#define TSL2561_TIMING          0x81
#define TSL2561_MEASURE         0x03
#define LIGHT_DATA              0x8C
#define TSL2561_GAIN1           0x01

short Open_i2c(int address){
	//short fd;
	char i2c_dev[16];

	sprintf(i2c_dev, "/dev/i2c-%d", 1);

	if ((fd = open(i2c_dev, O_RDWR)) < 0)
	{                                        
		printf("Failed to open i2c port\n");
		return 0;
	}

	if (ioctl(fd, I2C_SLAVE, address) < 0)                                
	{
		printf("Unable to get bus access to talk to slave on address %02Xh\n", address);
		return 0;
	}
	return fd;
}

int initialise_sensors(void){
    int i;
	int r1 = 0, r2 = 0, r3 = 0, r4 = 0;
	unsigned char buf[10];
	unsigned char bytes[6];

	sensors.pressure = 0;
	sensors.humidity = 0;
	sensors.temp = 0;
	sensors.x = 0;
	sensors.y = 0;
	sensors.z = 0;
	sensors.infrared = 0;
	sensors.light = 0;
	
	
	if(sensors.fd = Open_i2c(MS5607_ADDRESS)){   
		r1 = 1;
        buf[0] = MS5607_RESET;
		
        if ((write(sensors.fd, buf, 1)) != 1) {							
            printf("Error writing to i2c slave\n");
        }
        
        usleep(4000);
        
        buf[0] = MS5607_COEFF;
        
            for(i=0; i<6; i++){
                buf[0] = MS5607_COEFF + 2*i;
                
                if ((write(sensors.fd, buf, 1)) != 1) {							
            		printf("Error writing to i2c slave\n");
            	}
            	
            	if (read(sensors.fd, buf, 2) != 2) {							
            		printf("Unable to read from slave\n");
            	}
            	else{
            	sensors.c[i] = buf[0]<<8 | buf[1];
            	}
            }
        close(sensors.fd);
    
    sensors.tref = (int)sensors.c[4]*256;
    sensors.st1 = sensors.c[0]*pow(2, 16);
	//sensors.st1 = sensors.c[0]*(2 << 16);
    }
	
    if(sensors.fd = Open_i2c(HTU20D_ADDRESS)){
        r2 = 1;
        buf[0] = HTU20D_RESET;
        
        if ((write(sensors.fd, buf, 1)) != 1) {							
            printf("Error writing to i2c slave\n");
        }
        
        close(sensors.fd);
    }
    
    if (sensors.fd = Open_i2c(ADXL345_ADDRESS)){
         r3 = 1;
         bytes[0] = ADXL345_POWER_CTL;
         bytes[1] = MEASURE;
                
         if ((write(fd, bytes, 2)) !=2 ) {
             printf("Error writing to POWER_CTL\n");
         }
         close(sensors.fd);    
    }
    
    if (sensors.fd = Open_i2c(TSL2561_ADDRESS)){
         r4 = 1;
         bytes[0] = TSL2561_CONTROL;
         bytes[1] = TSL2561_MEASURE;
                
         if ((write(fd, bytes, 2)) !=2 ) {
             printf("Error writing to CONTROL\n");
         }
         
         bytes[0] = TSL2561_TIMING;
         bytes[1] = TSL2561_GAIN1;
                
        if ((write(fd, bytes, 2)) !=2 ) {
            printf("Error writing to COMMAND\n");
        }
         close(sensors.fd);    
    }
    
    return (r1 + r2 + r3 + r4);
}

int sensor_r(unsigned char buf[10]){

    
	if ((write(sensors.fd, buf, 1)) != 1) {							
       	printf("Error writing to i2c slave\n");
    }
        
    usleep(10000);
        
		
    buf[0] = MS5607_READ;
        
    if ((write(sensors.fd, buf, 1)) != 1) {							
    	printf("Error writing to i2c slave\n");
    }
		
	if (read(sensors.fd, buf, 3) != 3) {							
        printf("Unable to read from slave\n");
    }
    return (buf[0]<<16 | buf[1]<<8 | buf[2]) ;
}

void readTemperature(void){
	unsigned char buf[10];
	
	buf[0] = MS5607_TEMP;
	sensors.ut = 0;
    sensors.ut = sensor_r(buf);
    sensors.dT = sensors.ut - sensors.tref;
////////  
  sensors.temp = 2000.0 + ((float)(sensors.dT*sensors.c[5])/8388608.0);
    sensors.temp /= 100.0;
}

void readPressure(void){
    unsigned char buf[10];
	
	buf[0] = MS5607_PRESS;
    
    sensors.up = sensor_r(buf);
    sensors.off = sensors.c[1]*pow(2,17) + (sensors.c[3]*sensors.dT)/pow(2,6);
    sensors.sens= sensors.st1 + (sensors.c[2]*sensors.dT)/pow(2, 7);
    sensors.pressure= ((sensors.up*sensors.sens)/pow(2, 21) - sensors.off)/pow(2, 15);
   
   
    //sensors.off = sensors.c[1]*(2 << 17) + (sensors.c[3]*sensors.dT)/(2 << 6);
    //sensors.sens= sensors.st1 + (sensors.c[2]*sensors.dT)/(2 << 7);
    //sensors.pressure= ((sensors.up*sensors.sens)/(2 << 21) - sensors.off)/( 2 << 15);
    
}

void readHumidity(void){

	unsigned char buf[10];
	
	if(sensors.fd = Open_i2c(HTU20D_ADDRESS)){
            
        buf[0] = HTU20D_HUMID;
            
        if ((write(sensors.fd, buf, 1)) != 1) {							
            printf("Error writing to i2c slave\n");
        }
        usleep(18000);
            
        if (read(sensors.fd, buf, 2) != 2) {							
            printf("Unable to read from HTU, %x%x\n", buf[0], buf[1]);
        }
		
        sensors.htemp = buf[0]<<8 | buf[1];
        sensors.humidity = sensors.htemp / pow(2, 16);
		//sensors.humidity = sensors.htemp / (2 << 16);
        sensors.humidity = (sensors.humidity * 125) - 6;
        close(sensors.fd);
    }
    else{
        printf("Couldn't connect to HTU"); 
    }
}

void readAccelerometer(void){
     unsigned char bytes[6];
     int i;
     int j = 0;
    if(sensors.fd = Open_i2c(ADXL345_ADDRESS)){
        
        bytes[0] = AXES_DATA;
        
        usleep(12000);
        
        if ((write(sensors.fd, bytes, 1)) != 1) {                                                   
            printf("Error writing to i2c slave\n");
        }

        if ((read(sensors.fd, bytes, 6)) != 6) {
            printf("Unable to read from slave\n");
        }
        else{
            for(i=0; i<3;i++){
                sensors.data[i] = bytes[j] |( bytes[j+1]<<8);
                j = j+2;
            }
            sensors.x = sensors.data[0]*0.004;
            sensors.y = sensors.data[1]*0.004;
            sensors.z = sensors.data[2]*0.004;
        }
        close(sensors.fd);
    }
    else{
        printf("Problem connecting ADXL345.\n");
    }
}

void readLight(void){
        unsigned char bytes[4];
        int i;
        int j = 0;
        int ratio;
       
    if(sensors.fd = Open_i2c(TSL2561_ADDRESS)){   
    
        bytes[0] = LIGHT_DATA;       
                    
        usleep(180000);
            
        if ((write(sensors.fd, bytes, 1)) != 1) {                                                   
            printf("Error writing to i2c slave\n");
        }

        if ((read(sensors.fd, bytes, 4)) != 4) {
            printf("Unable to read from slave\n");
        }
        
        else{
            for(i=0; i<2;i++){
                sensors.c[i] = (bytes[j] |( bytes[j+1]<<8))*16;
                j = j+2;
                //printf("raw_data_%d: %d\n", (i+1), sensors.c[i]);
            }
            
            ratio = (float)sensors.c[1]/sensors.c[0];            
                       
            if ((ratio >= 0) && (ratio <= 0.52))
                sensors.light = (0.0315 * sensors.c[0]) - (0.0593 * sensors.c[0] * pow(ratio, 1.4));
            else if (ratio <= 0.65)
                sensors.light = (0.0229 * sensors.c[0]) - (0.0291 * sensors.c[1]);
            else if (ratio <= 0.8)
                sensors.light = (0.0157 * sensors.c[0]) - (0.018 * sensors.c[1]);
            else if (ratio <= 1.3)
                sensors.light = (0.00338 * sensors.c[0]) - (0.0026 * sensors.c[1]);
            else if (ratio > 1.3)
                sensors.light = 0;  
                
                sensors.light = sensors.light * 3.9753;
        }
        
        close(sensors.fd);
    }
    else{
        printf("Problem connecting.\n");
    }

}

void update_sensors(void){
    
		if(sensors.fd = Open_i2c(MS5607_ADDRESS)){
			
			readTemperature();
			readPressure();
			
			close(sensors.fd);
		}		
		else{
			printf("Problem connecting MS5607.\n");
		}
		
		//readHumidity();
		
		readAccelerometer();
		
		readLight();
		
	
}

void *sensors_loop(){

	initialise_sensors();
	
	while(1){
		update_sensors();
		//delay(500);
	}

}