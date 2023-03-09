#include "serialib.h"

int main(){
	serialib serial;
	if (serial.openDevice("/dev/ttyS0", 115200)!=1) return 1;

	for(int i=0; i<100; i++) {
		serial.writeString("Hello!");
		//usleep(100);
	}

	serial.closeDevice();
}