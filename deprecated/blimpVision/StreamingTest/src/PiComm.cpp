// ============================== INCLUDES ==============================
#include "PiComm.h"
#include "serialib.h"
#include "json.hpp"
using json = nlohmann::json;

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
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/ximgproc.hpp>

using namespace std;
using namespace cv;

// ============================== CLASS ==============================

void PiComm::init(ProgramData* programData){
	this->programData = programData;

	// Check that program is still running
	if(!programData->program_running){
		fprintf(stdout, "PiComm initialized with program_running=false. Stopping.\n");
		return;
	}

	// Init serial communication
	if(!programData->streamOnlyMode && !programData->disableSerialMode){
		initSerial();
	}

	// Determine blimp ID
	if(!programData->streamOnlyMode){
		if(programData->setBlimpID){
			blimpID = programData->customBlimpID;
		}else{
			blimpID = getIPAddress();
		}
	}

	// Init UDP connection to basestation
	if(!programData->streamOnlyMode){
		initUDPReceiver();
		initUDPSender();
	}

	// Init UDP connection for streaming
	initStreamSocket();

	send_buf = new unsigned char[MAX_DGRAM];

	// Start streaming thread
	if(!programData->disableStreamMode){
		if (pthread_create(&streaming_thread, NULL, PiComm::staticStreamingThread_start, this) < 0) {
			perror("PiComm failed to create thread: streaming_thread");
			programData->program_running = false;
			return;
		}
	}
	
	// Start machine learning feedback thread
	if(!programData->disableStreamMode){
		if (pthread_create(&MLFeedback_thread, NULL, PiComm::staticMLFeedbackThread_start, this) < 0) {
			perror("PiComm failed to create thread: MLFeedback_thread");
			programData->program_running = false;
			return;
		}
	}
}

void PiComm::end(){
    // If nullptr, never initialized
    if(programData == nullptr) return;
	
	fprintf(stdout, "Ending PiComm.\n");

	programData->program_running = false;

	// Close all threads
	pthread_join(streaming_thread, NULL);
	pthread_join(BSFeedback_thread, NULL);
	pthread_join(MLFeedback_thread, NULL);

	// Close socket
	if(stream_socket_fd){
		close(stream_socket_fd);
	}

	// Delete dynamic memory
	delete[] send_buf;

	map<string, Mat*>::iterator iter;
	for(iter = mapFrameNameToFrame.begin(); iter != mapFrameNameToFrame.end(); iter++){
		Mat* framePtr = iter->second;
		delete(framePtr);
	}
	mapFrameNameToFrame.clear(); // Just to be sure

    // Release programData
    programData = nullptr;
	//fprintf(stdout, "PiComm ended.\n");
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

float PiComm::chronoDiff(std::chrono::system_clock::time_point timeA, std::chrono::system_clock::time_point timeB){
	std::chrono::duration<double> chronoDiff = timeA - timeB;
	double timeDiff = chronoDiff.count(); // seconds
	return (float)timeDiff;
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
	//fprintf(stdout, "Sending serial message: %s\n", message.c_str());
}

char PiComm::readSerial() {
	char byte = 0;
	if(serial.available() > 0){
		serial.readBytes(&byte, 1);
	}
	return byte;
}

// ============================== UDP ==============================

bool PiComm::validTarget(string targetID){
	return targetID == blimpID;
}

//Expected format for message: baroData;targetGoal;targetEnemy
//Returns true if format matches, false if not
void PiComm::parseAutonomousMessage(string message, BSFeedbackData* BSFB){
	int firstSemiColon = message.find(";");
	int secondSemiColon = message.find(";",firstSemiColon+1);

	if(firstSemiColon == string::npos || secondSemiColon == string::npos){
		cout << "Invalid autonomous message format." << endl;
		return;
	}
	
	//Barometer data
	try {
		float baroData = stof(message.substr(0,firstSemiColon));
		BSFB->barometerData = baroData;
		BSFB->lastBaroMessageTime = (clock())/(float)CLOCKS_PER_SEC;
	}catch(std::invalid_argument& e) {
		cout << "Invalid barometer data received while autonomous." << endl;
	}
	
	//TargetGoal data
	string targetGoalData = message.substr(firstSemiColon+1,secondSemiColon-firstSemiColon-1);
	if(targetGoalData == "O"){
		BSFB->goalColor = orange;
	}else if(targetGoalData == "Y"){
		BSFB->goalColor = yellow;
	}
}

//Expected format for message: #,#,#,#,#,#,;baroData;targetGoal;targetEnemy
//	where # = number motor data
//Returns true if format matches, false if not
void PiComm::parseManualMessage(string message, BSFeedbackData* BSFB){
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
		return;
	}

	// Copy over motor data
	for(int i=0; i<numMotorCommands; i++){
		if(BSFB->recentMotorCommands.size() <= i){
			// expand vector if necessary
			BSFB->recentMotorCommands.push_back(newMotorCommands[i]);
		}else{
			BSFB->recentMotorCommands[i] = newMotorCommands[i];
		}
	}

	//Barometer data
	try {
		float baroData = stof(message.substr(firstSemiColon+1,secondSemiColon-firstSemiColon-1));
		BSFB->barometerData = baroData;
		BSFB->lastBaroMessageTime = (clock())/(float)CLOCKS_PER_SEC;
	}catch(std::invalid_argument& e) {
		cout << "Invalid barometer data received while autonomous." << endl;
	}
	
	//TargetGoal data
	string targetGoalData = message.substr(secondSemiColon+1,thirdSemiColon-secondSemiColon-1);
	if(targetGoalData == "O"){
		BSFB->goalColor = orange;
	}else if(targetGoalData == "Y"){
		BSFB->goalColor = yellow;
	}
}

void PiComm::parseBarometer(string message, BSFeedbackData* BSFB){
	//Barometer data
	try {
		float baroData = stof(message);
		BSFB->barometerData = baroData;
		BSFB->lastBaroMessageTime = (clock())/(float)CLOCKS_PER_SEC;
	}catch(std::invalid_argument& e) {
		cout << "Invalid barometer data received while autonomous." << endl;
	}
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

void PiComm::setStreamFrame(Mat frame, string frameNameRaw){
	if(frame.empty()) return;

	// Truncate name
	string frameName = frameNameRaw.substr(0,frameNameMaxSize);

	// Get frame pointer
	pthread_mutex_lock(&mutex_mapFrameNameToFrame);
	// Check if new frame needs to be dynamically allocated
	if(mapFrameNameToFrame.find(frameName) == mapFrameNameToFrame.end()){
		// New frame name, allocate!
		mapFrameNameToFrame[frameName] = new Mat();
	}
	Mat* framePtr = mapFrameNameToFrame[frameName];
	pthread_mutex_unlock(&mutex_mapFrameNameToFrame);
	
	// Copy frame
	*framePtr = frame;

	// Increment newFrameNum
	pthread_mutex_lock(&mutex_mapFrameNameToNewFrameNum);
	if(mapFrameNameToNewFrameNum.find(frameName) == mapFrameNameToNewFrameNum.end()){
		// New frame name, create new newFrameNum!
		mapFrameNameToNewFrameNum[frameName] = 0;
	}
	mapFrameNameToNewFrameNum[frameName] += 1;
	pthread_mutex_unlock(&mutex_mapFrameNameToNewFrameNum);
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
	return NULL;
}

void* PiComm::staticMLFeedbackThread_start(void* arg){
    static_cast<PiComm*>(arg)->MLFeedbackThread_loop();
	return NULL;
}

void PiComm::streamingThread_loop(){
	int t_id = (int)pthread_self();
	fprintf(stdout, "PiComm streaming thread (%d) successfully started.\n", t_id);
	
	while(programData->program_running){
		if(!programData->autonomous){
			continue;
		}
		// Get current time in clocks
		chrono::system_clock::time_point currentTime = chrono::system_clock::now();

		// Get number of unique frames streamed "recently" (within the last timeDelayRecentStream amount of time)
		int numFramesStreamedRecently = mapFrameNameToLastStreamTime.size();

		// Determine appropriate FPS throttle based on number of frames streamed recently
		float FPS; // Hz
		if(numFramesStreamedRecently <= 4){
			FPS = 5; //30
		}else{
			FPS = 5; //10
		}
		float delayFPS = 1.0/FPS; // seconds

		vector<NamedMatPtr> framesToStream;

		pthread_mutex_lock(&mutex_mapFrameNameToNewFrameNum);
		// Iterate through maps, find which frames are ready to be streamed
		map<string, unsigned int>::iterator iter;
		for(iter = mapFrameNameToNewFrameNum.begin(); iter != mapFrameNameToNewFrameNum.end(); iter++){
			string frameName = iter->first;
			unsigned int newFrameNum = iter->second;
			if(mapFrameNameToPrevFrameNum.find(frameName) != mapFrameNameToPrevFrameNum.end()){
				// Existing frame name
				if(newFrameNum <= mapFrameNameToPrevFrameNum[frameName]){
					// Frame already streamed for, skip...
					continue;
				}
			}
			// Frame is either new or not streamed yet
			
			// Check time last streamed
			if(mapFrameNameToLastStreamTime.find(frameName) != mapFrameNameToLastStreamTime.end()){
				// Existing frame name
				chrono::system_clock::time_point lastStreamTime = mapFrameNameToLastStreamTime[frameName];
				float lastStreamDelay = chronoDiff(currentTime, lastStreamTime);
				if(lastStreamDelay < delayFPS){
					// Frame streamed too recently, skip...
					continue;
				}else{
					//fprintf(stdout, "%s, last=%.2f, curr=%.2f\n", frameName.c_str(), lastStreamTime, currentTime);
				}
			}
			// Frame is either new or not streamed too recently

			// Frame is ready to stream!
			mapFrameNameToPrevFrameNum[frameName] = newFrameNum;
			mapFrameNameToLastStreamTime[frameName] = currentTime;

			// Get frame pointer
			pthread_mutex_lock(&mutex_mapFrameNameToFrame);
			Mat* framePtr = mapFrameNameToFrame[frameName];
			pthread_mutex_unlock(&mutex_mapFrameNameToFrame);

			// Push back to vector of frames to stream
			framesToStream.push_back(NamedMatPtr(framePtr, frameName));
		}
		pthread_mutex_unlock(&mutex_mapFrameNameToNewFrameNum);

		// Iterate through recent frames, remove frames NOT streamed recently
		map<string, chrono::system_clock::time_point>::iterator iterStreamTime = mapFrameNameToLastStreamTime.begin();
		while(iterStreamTime != mapFrameNameToLastStreamTime.end()){
			string frameName = iterStreamTime->first;
			chrono::system_clock::time_point lastStreamTime = iterStreamTime->second;
			iterStreamTime++;
			// Check if "recent"
			float lastStreamDelay = chronoDiff(currentTime, lastStreamTime);
			if(lastStreamDelay >= timeDelayRecentStream){
				mapFrameNameToLastStreamTime.erase(frameName);
			}
		}

		// Iterate through frames to stream and stream them!
		for(int i=0; i<framesToStream.size(); i++){
			send_frame(framesToStream[i]);
		}
	}

	fprintf(stdout, "PiComm streaming thread (%d) successfully stopped.\n", t_id);
	pthread_exit((void *) 0);
}

// STREAMING PROTOCOL:
// First uchar = N characters in frameName
// Next N uchars = frameName
// Remaining uchars = image
// Helper function for video streaming thread
void PiComm::send_frame(NamedMatPtr frameToStream) {
	//if(programData->verboseMode) fprintf(stdout, "Sending video frame (%s)... ", frameToStream.name.c_str());

	// Create header buffer
	vector<uchar> bufferHeader;
	// Push frameName size to header buffer
	int frameNameLen = frameToStream.name.size();
	bufferHeader.push_back(frameNameLen);
	// Push frameName to header buffer
	const char* frameName = frameToStream.name.c_str();
	for(int i=0; i<frameNameLen; i++){
		bufferHeader.push_back(frameName[i]); // Cast char to uchar?
	}

    // Compress image into vector buffer
    vector<uchar> bufferFrame;
	Mat* framePtr = frameToStream.mat;
	try{
    	cv::imencode(".jpg", *framePtr, bufferFrame);
	}catch(cv::Exception e){
		fprintf(stdout, "IMENCODE EXCEPTION.\n");
		return;
	}

	// Combine buffers
	vector<uchar> buffer;
	buffer.insert(buffer.end(), bufferHeader.begin(), bufferHeader.end());
	buffer.insert(buffer.end(), bufferFrame.begin(), bufferFrame.end());

    unsigned int size = (unsigned int)(buffer.size());
	//fprintf(stdout, "Frame %s (size = %d)\n", frameToStream.name.c_str(), size);
    unsigned int packet_count = ceil((double)size/(double)MAX_IMAGE_DGRAM);

	// Packet protocol to send DATA of size N
	// Max overhead size of M
	// Max packet size of P
	// Number of required packets Q = N / (P-M)
	// Each packet = overhead + DATA
	// Overhead = FrameName + FrameNum(mod 256?) + TotalNumberOfPackets + CurrentPacketNumber
    unsigned int array_pos_start = 0;
    while (packet_count > 0) {
		//fprintf(stdout, "%d, ", packet_count);

    	//Grab the next subvector to populate the current data packet
    	int array_pos_end = std::min(size, array_pos_start + MAX_IMAGE_DGRAM);
    	std::vector<uchar>::const_iterator first = buffer.begin() + array_pos_start;
    	std::vector<uchar>::const_iterator last = buffer.begin() + array_pos_end;
    	std::vector<uchar> packet_buf(first, last);
		packet_buf.insert(packet_buf.begin(), (unsigned char)packet_count);
		unsigned int send_buf_len = packet_buf.size();

		copy(packet_buf.begin(), packet_buf.end(), send_buf);

    	//Convert the subvector buffer to an unsigned char array
    	//unsigned int send_buf_len = 1 + packet_buf.size();
		//unsigned char* send_buf = new unsigned char[send_buf_len];
    	//unsigned char send_buf[send_buf_len];
    	//send_buf[0] = (unsigned char)packet_count; //First element is always the # of remaining packets
    	/*
		for (int i = 0; i < (int)packet_buf.size(); i++) {
    		send_buf[i+1] = packet_buf[i];
    	}*/

        sendto(stream_socket_fd, (const unsigned char *)send_buf, send_buf_len, MSG_CONFIRM, (const struct sockaddr *) &stream_server_addr, sizeof(stream_server_addr));
		//delete[] send_buf;

        array_pos_start = array_pos_end;
        packet_count--;
    }
	//fprintf(stdout, "\n");
	//if(programData->verboseMode) fprintf(stdout, "Done!\n");
}

void PiComm::MLFeedbackThread_loop(){
	int t_id = (int)pthread_self();
	fprintf(stdout, "PiComm machine learning feedback thread (%d) successfully started.\n", t_id);

	char buffer[MAXLINE];
	char sanitized_buffer[MAXLINE];
	unsigned int len = sizeof(stream_server_addr);
	while (programData->program_running) {
		int n = recvfrom(stream_socket_fd, (char *)buffer, MAXLINE, MSG_WAITALL, (struct sockaddr *) &stream_server_addr, &len);
		if (n > 0) {
			int m = 0;
			for (int i = 0; i < n; i++) {
				//Sanitize the JSON buffer
				if (buffer[i] != '\1' && buffer[i] != '\10') {
					sanitized_buffer[m++] = buffer[i];
				}
			}
			sanitized_buffer[m] = '\0';

			MLFeedbackData MLFeedbackTemp;
			pthread_mutex_lock(&mutex_MLFeedback);
			MLFeedbackTemp.lastReceivedTargets = json::parse(sanitized_buffer);
			MLFeedbackTemp.lastReceivedTargetsTime = clock();
			//cout << "Json: \"" << lastReceivedTargets << "\"" << endl;
			pthread_mutex_unlock(&mutex_MLFeedback);
		}
	}
	fprintf(stdout, "PiComm machine learning feedback thread (%d) successfully stopped.\n", t_id);
	pthread_exit((void *) 0);
}

void PiComm::setMode(autoState newMode){
	pthread_mutex_lock(&mutex_mode);
	mode = newMode;
	pthread_mutex_unlock(&mutex_mode);
}

autoState PiComm::getMode(){
	autoState modeCopied;
	pthread_mutex_lock(&mutex_mode);
	modeCopied = mode;
	pthread_mutex_unlock(&mutex_mode);
	return modeCopied;
}