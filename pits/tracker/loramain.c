// Executable to test LoRa execution standalone

#include "gps.h"
#include "lora.h"
#include "misc.h"
#include <sys/time.h>

int main(void){
	
	lora_loop(&GPS);

	return 0;
}