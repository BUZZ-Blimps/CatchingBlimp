// ============================== INCLUDES ==============================
#include "PiComm.h"
#include "serialib.h"

//#include <wiringPi.h>
//#include <wiringSerial.h>

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
#include <chrono>
#include <netdb.h>
#include <ifaddrs.h>
#include <linux/if_link.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/ximgproc.hpp>

using namespace std;
using namespace cv;

// ============================== CLASS ==============================

void PiComm::setBlimpID(string newBlimpID){
	blimpID = newBlimpID;
}

string PiComm::getIPAddress(){
	string IPAddress = "";
	string targetIfaName = "wlan0";
	int targetIfaFamily = AF_INET;
	string failureReturn = "";

	// Code from: https://man7.org/linux/man-pages/man3/getifaddrs.3.html
	struct ifaddrs *ifaddr;
	int s;
	char host[NI_MAXHOST];

	if (getifaddrs(&ifaddr) == -1) {
		cout << "Unable to get local IP addresses." << endl;
		return failureReturn;
	}

	/* Walk through linked list, maintaining head pointer so we
		can free list later. */

	for (struct ifaddrs *ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
		if (ifa->ifa_addr == NULL)
			continue;

		string ifaName = ifa->ifa_name;
		int ifaFamily = ifa->ifa_addr->sa_family;
		
		if(ifaName == targetIfaName && ifaFamily == targetIfaFamily){
			// getnameinfo ONLY IF: ifaFamily == AF_INET || ifaFamily == AF_INET6
			s = getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in), host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);
			if (s != 0) {
				cout << "getnameinfo() failed." << endl;
			}else{
				IPAddress = string(host);
				//cout << "Identified IP Address: " << IPAddress << endl;
			}
		}
	}

	freeifaddrs(ifaddr);
	return IPAddress;
}

// ============================== SERIAL ==============================

void PiComm::initSerial(){
	clock_t lastAttempt;
	//Attempt to connect
	while ((serial.openDevice("/dev/ttyS0", 115200)) < 0 ) {
		cout << "Unable to open serial port." << endl;
		//Delay if not connected
		lastAttempt = clock()-SERIAL_ATTEMPT_DELAY;
		while(double(clock() - lastAttempt)/CLOCKS_PER_SEC < SERIAL_ATTEMPT_DELAY);
	}
}

void PiComm::sendSerial(string message){
	serial.writeString(message.c_str());
}

char PiComm::readSerial() {
	char byte = 0;
	if(serial.available() > 0){
		serial.readBytes(&byte, 1);
	}
	return byte;
}

// ============================== UDP ==============================

void PiComm::initUDPReceiver(){
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

void PiComm::initUDPSender(){
	sockSend = socket(AF_INET, SOCK_DGRAM, 0);
	memset(&addrSend, 0, sizeof(addrSend));
	addrSend.sin_family = AF_INET;
	addrSend.sin_addr.s_addr = inet_addr(group);
	addrSend.sin_port = htons(port);
}

bool PiComm::readUDP(string* retMessage, string* target, string* source, string* flag){
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

void PiComm::sendUDPRaw(string target, string source, string flag, string message){
	string combined = ":)" + target + "," + source + ":" + flag + ":" + message;
	char* messageBuff = &combined[0];
	int numbytes = sendto(sockSend, messageBuff, strlen(messageBuff), 0, (struct sockaddr*) &addrSend, sizeof(addrSend));
}

void PiComm::sendUDP(string flag, string message){
	sendUDPRaw("0",blimpID,flag,message);
}

void PiComm::sendUDP(string message){
	sendUDPRaw("0",blimpID,"D",message);
}

bool PiComm::validTarget(string targetID){
	return targetID == blimpID;
}

//Expected format for message: baroData;targetGoal;targetEnemy
//Returns true if format matches, false if not
bool PiComm::parseAutonomousMessage(string message, bool* newBaroDataValid, float* newBaroData, goalType* newGoalColor){
	int firstSemiColon = message.find(";");
	int secondSemiColon = message.find(";",firstSemiColon+1);

	if(firstSemiColon == string::npos || secondSemiColon == string::npos){
		cout << "Invalid autonomous message format." << endl;
		return false;
	}
	
	//Barometer data
	bool baroDataValid = false;
	float baroData = 0;
	try {
		baroData = stof(message.substr(0,firstSemiColon));
		baroDataValid = true;
	}catch(std::invalid_argument& e) {
		cout << "Invalid barometer data received while autonomous." << endl;
	}
	*newBaroDataValid = baroDataValid;
	if(baroDataValid){
		*newBaroData = baroData;
	}
	
	//TargetGoal data
	string targetGoalData = message.substr(firstSemiColon+1,secondSemiColon-firstSemiColon-1);
	if(targetGoalData == "O"){
		*newGoalColor = orange;
	}else if(targetGoalData == "Y"){
		*newGoalColor = yellow;
	}

	return true;
}

//Expected format for message: #,#,#,#,#,#,;baroData;targetGoal;targetEnemy
//	where # = number motor data
//Returns true if format matches, false if not
bool PiComm::parseManualMessage(string message, vector<float>* motorCommands, bool* newBaroDataValid, float* newBaroData, goalType* newGoalColor){
	bool validFormatting = true;
	
	//Parse motor data
	int numMotorCommands = 6;
	float newMotorCommands[numMotorCommands];

	int previousComma = -1;
	for(int i=0; i<numMotorCommands; i++){
		int nextComma = message.find(",",previousComma+1);
		if(nextComma == string::npos){
			validFormatting = false;
			break;
		}

		string motorString = message.substr(previousComma+1,nextComma-previousComma-1);
		float motorData;
		try {
			motorData = stof(motorString);
		}catch(std::invalid_argument& e) {
			cout << "Invalid motor data received while manual." << endl;
			validFormatting = false;
			break;
		}
		previousComma = nextComma;

		newMotorCommands[i] = motorData;
	}
	
	int firstSemiColon = message.find(";");
	int secondSemiColon = message.find(";",firstSemiColon+1);
	int thirdSemiColon = message.find(";",secondSemiColon+1);
	if(firstSemiColon == string::npos || secondSemiColon == string::npos || thirdSemiColon == string::npos){
		validFormatting = false;
	}

	// Confirm valid formatting
	if(!validFormatting){
		cout << "Invalid manual message format." << endl;
		return false;
	}

	// Copy over motor data
	for(int i=0; i<numMotorCommands; i++){
		if(motorCommands->size() <= i) motorCommands->push_back(0);
		(*motorCommands)[i] = newMotorCommands[i];
	}

	//Barometer data
	bool baroDataValid = false;
	float baroData = 0;
	try {
		baroData = stof(message.substr(firstSemiColon+1,secondSemiColon-firstSemiColon-1));
		baroDataValid = true;
	}catch(std::invalid_argument& e) {
		cout << "Invalid barometer data received while autonomous." << endl;
	}
	*newBaroDataValid = baroDataValid;
	if(baroDataValid){
		*newBaroData = baroData;
	}
	
	//TargetGoal data
	string targetGoalData = message.substr(secondSemiColon+1,thirdSemiColon-secondSemiColon-1);
	if(targetGoalData == "O"){
		*newGoalColor = orange;
	}else if(targetGoalData == "Y"){
		*newGoalColor = yellow;
	}

	return true;
}

bool PiComm::parseBarometer(string message, float* newBaroData){
	//Barometer data
	bool baroDataValid = false;
	float baroData = 0;
	try {
		baroData = stof(message);
		baroDataValid = true;
	}catch(std::invalid_argument& e) {
		cout << "Invalid barometer data received while autonomous." << endl;
	}
	if(baroDataValid){
		*newBaroData = baroData;
	}
	return baroDataValid;
}

/*
void PiComm::establishBlimpID(){ // EXTREMELY DEPRECATED
	double delay = 0.25; // seconds
	chrono::system_clock::time_point lastTime = chrono::system_clock::now();
	while(blimpID == -1){
		chrono::system_clock::time_point currentTime = chrono::system_clock::now();
		chrono::duration<double> timeDiff = currentTime - lastTime;
		double elapsedTime = timeDiff.count(); // seconds

		if(elapsedTime > delay){
			lastTime = currentTime;
			sendUDPRaw("0","N","N","N");
			bool reading = true;
			while(reading){
				string message;
				string target;
				string source;
				string flag;
				bool success = readUDP(&message, &target, &source, &flag);
				if(!success){
					reading = false;
				}else{
					if(target == "N" && source == "0" && flag == "N"){
						try {
							blimpID = stoi(message);
						}
						catch (std::invalid_argument& e) {
							cout << "establish blimp ID->invalid blimp id: " << message << endl;
						}
					}
				}
			}
		}
	}
	cout << "Received blimp ID: " << blimpID << endl;
}
*/

// ============================== STREAMING ==============================
void PiComm::initStreamSocket(){
	//Create UDP socket
	stream_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (stream_socket_fd < 0) {
        perror("socket");
        exit(EXIT_FAILURE);
    }

    //Configure socket to be non-blocking
    fcntl(stream_socket_fd, F_SETFL, O_NONBLOCK);

    //Configure socket
    memset(&stream_server_addr, 0, sizeof(stream_server_addr));
    stream_server_addr.sin_family = AF_INET;
    stream_server_addr.sin_port = htons(stream_server_port);

    in_addr addr;
    inet_aton(stream_server_ip, &addr);
    stream_server_addr.sin_addr.s_addr = (in_addr_t)addr.s_addr;
}

void PiComm::setStreamFrame(Mat frame){
	// Copy image to shared memory
	pthread_mutex_lock(&mutex_frameToStream);
	frameToStream = frame;
	pthread_mutex_unlock(&mutex_frameToStream);
	pthread_mutex_lock(&mutex_newFrameNum);
	newFrameNum++;
	pthread_mutex_unlock(&mutex_newFrameNum);
}

BSFeedbackData PiComm::getBSFeedback(){
	BSFeedbackData BSFeedbackCopied;
	pthread_mutex_lock(&mutex_BSFeedback);
	BSFeedbackCopied = BSFeedback;
	pthread_mutex_unlock(&mutex_BSFeedback);
	return BSFeedbackCopied;
}

MLFeedbackData PiComm::getMLData(){
	MLFeedbackData MLFeedbackCopied;
	pthread_mutex_lock(&mutex_MLFeedback);
	MLFeedbackCopied = MLFeedback;
	pthread_mutex_unlock(&mutex_MLFeedback);
	return MLFeedbackCopied;
}

// Static functions to run thread member functions
// https://cplusplus.com/forum/unices/138864/
void* PiComm::staticStreamingThread_start(void* arg){
    static_cast<PiComm*>(arg)->streamingThread_loop();
}

void* PiComm::staticBSFeedbackThread_start(void* arg){
    static_cast<PiComm*>(arg)->BSFeedbackThread_loop();
}

void* PiComm::staticMLFeedbackThread_loop(void* arg){
    static_cast<PiComm*>(arg)->MLFeedbackThread_loop();
}

void PiComm::streamingThread_loop(){

}

//Helper function for video streaming thread
void PiComm::send_frame(cv::Mat image) {
	if(verboseMode) fprintf(stdout, "Starting to send video frame.\n");
    //Compress image into vector buffer
    std::vector<uchar> buf;
    cv::imencode(".jpg", image, buf);

    unsigned int size = (unsigned int)buf.size();
    unsigned int packet_count = ceil((double)size/(double)MAX_IMAGE_DGRAM);

    unsigned int array_pos_start = 0;
    while (packet_count > 0) {
		if(verboseMode) fprintf(stdout, "%d, ", packet_count);

    	//Grab the next subvector to populate the current data packet
    	int array_pos_end = std::min(size, array_pos_start + MAX_IMAGE_DGRAM);
    	std::vector<uchar>::const_iterator first = buf.begin() + array_pos_start;
    	std::vector<uchar>::const_iterator last = buf.begin() + array_pos_end;
    	std::vector<uchar> packet_buf(first, last);

    	//Convert the subvector buffer to an unsigned char array
    	unsigned int send_buf_len = packet_buf.size() + 1;
    	unsigned char send_buf[send_buf_len];
    	send_buf[0] = (unsigned char)packet_count; //First element is always the # of remaining packets
    	for (int i = 0; i < (int)packet_buf.size(); i++) {
    		send_buf[i+1] = packet_buf[i];
    	}

        sendto(stream_socket_fd, (const unsigned char *)send_buf, send_buf_len, MSG_CONFIRM, (const struct sockaddr *) &stream_server_addr, sizeof(stream_server_addr));

        array_pos_start = array_pos_end;
        packet_count--;
    }
	if(verboseMode) fprintf(stdout, "\nSent video frame.\n");
}

void PiComm::BSFeedbackThread_loop(){

}

void PiComm::MLFeedbackThread_loop(){

}