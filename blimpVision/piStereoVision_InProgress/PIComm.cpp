// ============================== INCLUDES ==============================
#include "PiComm.h"

#include <wiringPi.h>
#include <wiringSerial.h>

#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>

using namespace std;

// ============================== CLASS ==============================

void PiComm::setBlimpID(int newBlimpID){
	blimpID = newBlimpID;
}


// ============================== SERIAL ==============================

void PiComm::initSerial(){
	clock_t lastAttempt;
	//Attempt to connect
	while ((serial = serialOpen ("/dev/ttyS0", 115200)) < 0 ) {
		cout << "Unable to open serial port." << endl;
		//Delay if not connected
		lastAttempt = clock()-SERIAL_ATTEMPT_DELAY;
		while(double(clock() - lastAttempt)/CLOCKS_PER_SEC < SERIAL_ATTEMPT_DELAY);
	}
}

void PiComm::sendSerial(string message){
	char* sendBuffer = &message[0];
	serialPuts(serial, sendBuffer);
	//cout << "Serial sending: " << message << endl << endl;
}

char PiComm::readSerial() {
	char byte = 0;

	int bytesReceived = serialDataAvail(serial);

	if (bytesReceived > 0) {
		//Read a pending byte into the buffer
			byte = (char)serialGetchar(serial);
	}

	//cout << (int)byte << "\t" << bytesReceived << endl;

	return byte;
}

// ============================== UDP ==============================


string groupAddress = UDP_IP;
char* group = &groupAddress[0];
int port = 	UDP_PORT;

int sockRec;
struct sockaddr_in addrRec;
int sockSend;
struct sockaddr_in addrSend;

void initUDPReceiver(){
	sockRec = socket(AF_INET, SOCK_DGRAM, 0);

	u_int yes = 1;
	setsockopt(sockRec, SOL_SOCKET, SO_REUSEADDR, (char*) &yes, sizeof(yes));

	memset(&addrRec, 0, sizeof(addrRec));
	addrRec.sin_family = AF_INET;
	addrRec.sin_addr.s_addr = htonl(INADDR_ANY); // differs from sender
	addrRec.sin_port = htons(port);
	bind(sockRec, (struct sockaddr*) &addrRec, sizeof(addrRec));

	struct ip_mreq mreq;
	mreq.imr_multiaddr.s_addr = inet_addr(group);
	mreq.imr_interface.s_addr = htonl(INADDR_ANY);
	setsockopt(sockRec, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*) &mreq, sizeof(mreq));

	fcntl(sockRec, F_SETFL, O_NONBLOCK);
}

void initUDPSender(){
	sockSend = socket(AF_INET, SOCK_DGRAM, 0);
	memset(&addrSend, 0, sizeof(addrSend));
	addrSend.sin_family = AF_INET;
	addrSend.sin_addr.s_addr = inet_addr(group);
	addrSend.sin_port = htons(port);
}

bool readUDP(string* retMessage, string* target, string* source, string* flag){
	int bufferSize = 1024;
	char buffer[bufferSize];
	memset(&buffer, 0, bufferSize);
	unsigned int addrLen = sizeof(addrRec);
	int numbytes = recvfrom(sockRec, &buffer, bufferSize, 0,(struct sockaddr *) &addrRec, &addrLen);
	if (numbytes > 0) {
		if(numbytes > bufferSize-1){
			buffer[bufferSize-1] = '\0';
		}else{
			buffer[numbytes] = '\0';
		}
		string message(buffer);
		if(message.substr(0,2) == ":)"){
			int comma = message.find(",");
			int firstColon = message.find(":",comma+1);
			int secondColon = message.find(":",firstColon+1);
			if(comma != string::npos && firstColon != string::npos && secondColon != string::npos){
				string targetString = message.substr(2,comma-2);
				string sourceString = message.substr(comma+1,firstColon-comma-1);
				string flagString = message.substr(firstColon+1,secondColon-firstColon-1);
				string messageString = message.substr(secondColon+1);

				if(retMessage != nullptr) *retMessage = messageString;
				if(target != nullptr) *target = targetString;
				if(source != nullptr) *source = sourceString;
				if(flag != nullptr) *flag = flagString;
				return true;
			}else{
				cout << "Comma or Colons not found: " << comma << ", " << firstColon << ", " << secondColon << endl;
			}
		}
	}
	return false;
}

void sendUDPRaw(string target, string source, string flag, string message){
	string combined = ":)" + target + "," + source + ":" + flag + ":" + message;
	char* messageBuff = &combined[0];
	int numbytes = sendto(sockSend, messageBuff, strlen(messageBuff), 0, (struct sockaddr*) &addrSend, sizeof(addrSend));
}

void sendUDP(string flag, string message){
	sendUDPRaw("0",to_string(blimpID),flag,message);
}

void sendUDP(string message){
	sendUDPRaw("0",to_string(blimpID),"D",message);
}