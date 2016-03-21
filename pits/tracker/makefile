all: tracker receiver

tracker: tracker.o gps.o misc.o auxiliary_threads.o lora.o sen.o
	cc -o tracker tracker.o gps.o misc.o auxiliary_threads.o lora.o sen.o -lm -lwiringPi -lwiringPiDev -lpthread -lncurses 

tracker_old: tracker.o gps.o misc.o DS18B20.o adc.o snapper.o led.o bmp085.o aprs.o lora.o sen.o
	cc -o tracker tracker.o gps.o misc.o DS18B20.o adc.o snapper.o led.o bmp085.o aprs.o lora.o sen.o -lm -lwiringPi -lwiringPiDev -lpthread -lncurses 

main: loramain.o misc.o lora.o sen.o
	cc -o main loramain.o lora.o sen.o -lm -lwiringPi -lwiringPiDev -lncurses 

receiver: receiver.o lora.o misc.o sen.o
	cc -o receiver/receiver receiver.o lora.o sen.o -lm -lwiringPi -lwiringPiDev -lpthread -lncurses 
	
tracker.o: tracker.c gps.h misc.h DS18B20.h adc.h bmp085.h lora.h
	gcc -c tracker.c

gps.o: gps.c gps.h misc.h
	gcc -c gps.c

aprs.o: aprs.c aprs.h misc.h sine_table.h
	gcc -c aprs.c

lora.o: lora.c lora.h misc.h
	gcc -c lora.c

DS18B20.o: DS18B20.c DS18B20.h misc.h gps.h
	gcc -c DS18B20.c

adc.o: adc.c adc.h
	gcc -c adc.c
	
snapper.o: snapper.c snapper.h misc.h
	gcc -c snapper.c

led.o: led.c led.h misc.h
	gcc -c led.c
	
misc.o: misc.c misc.h
	gcc -c misc.c

bmp085.o: bmp085.c bmp085.h
	gcc -c bmp085.c

sen.o: sensors/sen.c sensors/sen.h
	gcc -c sensors/sen.c 

receiver.o: receiver/receiver.c
	gcc -c receiver/receiver.c

auxiliary_treads.o: auxiliary_threads.c auxiliary_threads.h
	gcc -c auxiliary_threads.c

clean:
	rm *.o
