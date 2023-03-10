#include "serialib.h"
#include <chrono>
#include <string>

using namespace std;

char readSerial(serialib* serial){
	char byte = 0;
	if(serial->available() > 0){
		serial->readBytes(&byte, 1, 1);
	}
	return byte;
}

int main(){
	double serialSendDelay = 3; // seconds

	serialib serial;
	if (serial.openDevice("/dev/ttyS0", 115200)!=1) return 1;

	chrono::system_clock::time_point lastTime = chrono::system_clock::now();
	string message = "";
	while(true){
		chrono::system_clock::time_point currentTime = chrono::system_clock::now();
		chrono::duration<double> timeDiff = currentTime - lastTime;
		double elapsedTime = timeDiff.count(); // seconds

		if(elapsedTime > serialSendDelay){
			lastTime = currentTime;
			serial.writeString("Hello!");
		}

		char readByte = readSerial(&serial);
		if(readByte != 0){
			if(readByte != 10 && readByte != 13){
				message += readByte;
			}else if(readByte == 10){
				cout << "Received: " << message << endl;
				message = "";
			}
		}
	}

	serial.closeDevice();
}