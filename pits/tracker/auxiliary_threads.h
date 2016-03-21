// Contains the thread functions from many of the other files. For streamlining

#ifndef AUXILIARY_H
#define AUXILIARY_H



void *CameraLoop(void *some_void_ptr);

void *LEDLoop(void *some_void_ptr);

void *DS18B20Loop(void *some_void_ptr);

void *BMP085Loop(void *some_void_ptr);

void *APRSLoop(void *some_void_ptr);

void *ADCLoop(void *some_void_ptr);

#endif