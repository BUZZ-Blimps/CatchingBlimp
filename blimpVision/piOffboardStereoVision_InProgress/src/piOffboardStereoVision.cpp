//============================================================================
// Name        : piOffboardStereoVision.cpp
// Author      : Aidan Amstutz, Adam Pooley, Willie Warke
// Version     : 2.0
// Copyright   : Copyright 2022 SWAMP INCORPORATED
// Description : SWAMP Blimps Pi C++ Codebase
//============================================================================

//C++ includes
#include <algorithm>
#include <cmath>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <iostream>
#include <iterator>
#include <list>
#include <unordered_map>
#include <string>
#include <vector>

//C includes
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <libv4l2.h>
#include <linux/videodev2.h>
#include <math.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

//OpenCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>

//Libraries
#include <wiringPi.h>
#include <wiringSerial.h>
#include "Teleplot.h"

#include "piOffboardStereoVision.hpp"

using namespace std;
using namespace cv;

void stop_program(int signal) {
	fprintf(stdout, "\nInterrupt received - exiting program.\n");

	//Send all threads a termination flag
	program_running = false;

	//Wait for running threads to exit
	pthread_join(stream_thread, NULL);
	pthread_join(stream_fb_thread, NULL);

	//DESTROY the capture mutex
	sem_destroy(&cap_sem);

	//Close all open sockets
	if (stream_socket_fd) {
		close(stream_socket_fd);
	}

	if (recSocketFD) {
		close(recSocketFD);
	}

	if (sendSocketFD) {
		close(sendSocketFD);
	}

	//Exit successfully
	exit(EXIT_SUCCESS);
}

//Base station comms thread
void *bs_udp_comms(void *thread_id) {
	int t_id = (int)pthread_self();
	fprintf(stdout, "Base station communication thread (%d) successfully started.\n", t_id);

    clock_t lastCycle = 0;
    clock_t lastHeartbeat = 0;

	//Recording variables
	framesLeftToRecord = 9000;
	int moreFramesPerTrigger = 30 * 10;

	fprintf(stdout, "Connecting to base station. Blimp ID: %d.\n", blimpID);

	while (program_running) {
		clock_t currentTime = clock();
		double cycleTime = double(currentTime - lastCycle)/CLOCKS_PER_SEC;
		lastCycle = currentTime;
		//cout << "Cycle Time (s): " << cycleTime << endl;
		benchmarkFirst("Heart");

		//Send heartbeat
		if (double(currentTime - lastHeartbeat)/CLOCKS_PER_SEC > HEARTBEAT_PERIOD) {
			lastHeartbeat = currentTime;
			//sendUDP("H");
			sendUDP("S", to_string(mode));
		}

		//Receive UDP messages
		bool reading = true;
		while(reading) {
			string readIn;
			string target;
			string flag;
			bool messageReceived = readUDP(&readIn, &target, nullptr, &flag);
			if (!messageReceived) {
				reading = false;
			} else {
				int id = -1;
				try {
					id = stoi(target);
				}
				catch(std::invalid_argument& e) {
					cout << "invalid blimp id: \"" << target << "\", \"" << readIn << "\"" << endl;
				}
				if (id == blimpID){
					//cout << "Received: \"" << readIn << "\"" << endl;
					if (flag == "A") {
						autonomous = true;
						lastUDPReceived = clock();

						int firstSemiColon = readIn.find(";");
						int secondSemiColon = readIn.find(";",firstSemiColon+1);

						//Barometer data
						bool isBaroDataGood = false;
						try {
							barometerData = stof(readIn.substr(0,firstSemiColon));
							//cout << "Barodata: " << barometerData << endl;
							isBaroDataGood = true;
						}
						catch (std::invalid_argument& e) {
							cout << "Invalid barometer data received." << endl;
							isBaroDataGood = false;
						}

						if (isBaroDataGood) {
							//reset timer
							clock_t now = clock();
							lastBaroMessageTime = now/(float)CLOCKS_PER_SEC;
						}

						//TargetGoal data
						string targetGoalData = readIn.substr(firstSemiColon+1,secondSemiColon-firstSemiColon-1);
						if (targetGoalData == "O") {
							goalColor = orange;
						} else if (targetGoalData == "Y") {
							goalColor = yellow;
						}

						//Disregard the rest

					} else if (flag == "M") {
						//cout << "M" << endl;
						lastUDPReceived = clock();
						autonomous = false;

						//MotorInput data
						int startIndex = -1;
						for(int i=0; i<6; i++){
							int endIndex = readIn.find(",",startIndex+1);
							string motorString = readIn.substr(startIndex+1,endIndex-startIndex-1);
							float motorData = stof(motorString);
							startIndex = endIndex;

							if (recentMotorCommands.size() <= i) recentMotorCommands.push_back(0);
							recentMotorCommands[i] = motorData;
						}

						int firstSemiColon = readIn.find(";");
						int secondSemiColon = readIn.find(";",firstSemiColon+1);
						int thirdSemiColon = readIn.find(";",secondSemiColon+1);

						//Barometer data
						bool isBaroDataGood = false;
						try {
							barometerData = stof(readIn.substr(firstSemiColon+1,secondSemiColon-firstSemiColon-1));
							//cout << "Barodata: " << barometerData << endl;
							isBaroDataGood = true;
						}
						catch(std::invalid_argument& e) {
							cout << "Invalid barometer data received: \"" << readIn.substr(firstSemiColon+1,secondSemiColon-firstSemiColon-1) << "\"" << endl;
							isBaroDataGood = false;
						}
						if (isBaroDataGood) {
							//reset timer
							clock_t now = clock();
							lastBaroMessageTime = now/(float)CLOCKS_PER_SEC;
						}

						//TargetGoal data
						string targetGoalData = readIn.substr(secondSemiColon+1,thirdSemiColon-secondSemiColon-1);
						if (targetGoalData == "O") {
							goalColor = orange;
						} else if (targetGoalData == "Y") {
							goalColor = yellow;
						}

						//cout << "Manual Mode:" << endl;
						//cout << recentMotorCommands[0] << ", " << recentMotorCommands[1] << ", " << recentMotorCommands[2] << ", " << recentMotorCommands[3] << ", " << recentMotorCommands[4] << ", " << recentMotorCommands[5] << endl;
						//cout << "Barometer: " << barometerData << endl;
						//cout << "TargetGoal: " << goalColor << endl;

						//Disregard the rest

					} else if(flag == "B") {
						//Barometer reading
						bool isBaroDataGood = false;
						try {
							barometerData = stof(readIn);
							//cout << "Barodata: " << barometerData << endl;
							isBaroDataGood = true;
						}
						catch(std::invalid_argument& e) {
							cout << "Invalid barometer data received." << endl;
							isBaroDataGood = false;
						}

						if (isBaroDataGood) {
							//reset timer
							clock_t now = clock();
							lastBaroMessageTime = now/(float)CLOCKS_PER_SEC;
						}

					} else if (flag == "P") {
						char first = readIn.at(0);
						if(first == 'C'){
							cout << "Start recording." << endl;
							framesLeftToRecord += moreFramesPerTrigger;
						}
					} else if (flag == "K") {
						cout << "Received kill command. Killing..." << endl;
						stop_program(0);
						pthread_exit((void *) 0);
					} else if (flag == "TG") {
						if (readIn == "O") {
							goalColor = orange;
						} else if (readIn == "Y") {
							goalColor = yellow;
						}
					}
					//cout << "UDPReceived: \"" << readIn << "\" sent to \"" << target << "\"" << endl;
				}
			}
		} //end while (reading loop)

		if (clock()/((float)CLOCKS_PER_SEC) - lastBaroMessageTime > 10) {
			barometerData = -10000;

			if (verboseMode)
				cerr << "Baro data not current" << endl;
		}
	}

	fprintf(stdout, "Base station communication thread (%d) successfully stopped.\n", t_id);
	pthread_exit((void *) 0);
}

//Helper function for video streaming thread
void send_frame(cv::Mat image) {
    //Compress image into vector buffer
    std::vector<uchar> buf;
    cv::imencode(".jpg", image, buf);

    unsigned int size = (unsigned int)buf.size();
    unsigned int packet_count = ceil((double)size/(double)MAX_IMAGE_DGRAM);

    unsigned int array_pos_start = 0;
    while (packet_count > 0) {

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
}

//Video streaming thread
void *stream_video(void *thread_id) {
	int t_id = (int)pthread_self();
	fprintf(stdout, "Video streaming thread (%d) successfully started.\n", t_id);

	cv:: Mat image;

    //Set up video capture
	cv::VideoCapture cap;

	int cap_api_id = cv::CAP_ANY;
    cap.open(cap_device_id, cap_api_id);
    if (!cap.isOpened()) {
        std::cerr << "ERROR! Unable to open camera\n";
        return NULL;
    }

    //Set the stereo cam to full resolution
	cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

	while (program_running) {
		//Read image off stereo cam
		if (cap.grab()) {
			cap.retrieve(image);

		    //Crop the left and right images
		    cv::Rect left_roi(0, 0, image.cols/2, image.rows);
		    cv::Rect right_roi(image.cols/2, 0, image.cols/2, image.rows);
			cv::Mat lt_frame_maxres(image, left_roi);
			cv::Mat rt_frame_maxres(image, right_roi);

			//Reduce image size for rectification
			resize(lt_frame_maxres, lt_frame_lowres, Size(RECT_WIDTH, RECT_HEIGHT), INTER_LINEAR);
			resize(rt_frame_maxres, rt_frame_lowres, Size(RECT_WIDTH, RECT_HEIGHT), INTER_LINEAR);

		    //Signal to main thread that a new frame is available
		    sem_post(&cap_sem);

			//Stream left frame to offboard server
			if(!annotatedMode) send_frame(lt_frame_maxres);
		}

		if (annotatedMode && annotatedFrameReady){
			annotatedFrameReady = false;
			Mat annotatedFrameToSend;
			pthread_mutex_lock(&annotatedFrameMutex);
			annotatedFrame.copyTo(annotatedFrameToSend);
			pthread_mutex_unlock(&annotatedFrameMutex);
			send_frame(annotatedFrameToSend);
		}
	}

	fprintf(stdout, "Video streaming thread (%d) successfully stopped.\n", t_id);
	pthread_exit((void *) 0);
	return NULL;
}

void *stream_fb_check(void *thread_id) {
	int t_id = (int)pthread_self();
	fprintf(stdout, "Stream feedback thread (%d) successfully started.\n", t_id);

	char buffer[MAXLINE];
	char sanitized_buffer[MAXLINE];
	unsigned int len = sizeof(stream_server_addr);
	while (program_running) {
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

			pthread_mutex_lock(&receivedTargetsMutex);
			lastReceivedTargets = json::parse(sanitized_buffer);
			//cout << "Json: \"" << lastReceivedTargets << "\"" << endl;
			lastReceivedTargetsTime = clock();
			lastReceivedIsNew = true;
			pthread_mutex_unlock(&receivedTargetsMutex);

			//std::cout << "x: " << ex1["x"] << ", y: " << ex1["y"] << std::endl;
		}
	}
	fprintf(stdout, "Stream feedback thread (%d) successfully stopped.\n", t_id);
	pthread_exit((void *) 0);
	return NULL;
}

void init_udp_streamer() {
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

//==================== FUNCTION HEADERS ====================
//VideoCapture openCamera(int camIndex, int camWidth, int camHeight){
//	VideoCapture cap(camIndex);
//	if (!cap.isOpened()) {
//		CV_Assert("Cam open failed");
//	}
//
//	//cap.set(CAP_PROP_FPS, 120);
//	cap.set(CAP_PROP_FRAME_WIDTH, camWidth);
//	cap.set(CAP_PROP_FRAME_HEIGHT, camHeight);
//	return cap;
//}

//==================== COMMUNICATION FUNCTION HEADERS ====================
int serial;
void initSerial() {
	do {
		serial = serialOpen ("/dev/ttyS0", 115200);
		sleep(1);
	} while (serial < 0);
}

void sendSerial(string message){
	char* sendBuffer = &message[0];
	serialPuts(serial, sendBuffer);
	//cout << "Serial sending: " << message << endl << endl;
}

char readSerial() {
	char byte = 0;

	int bytesReceived = serialDataAvail(serial);

	if (bytesReceived > 0) {
		//Read a pending byte into the buffer
		byte = (char)serialGetchar(serial);
	}

	//cout << (int)byte << "\t" << bytesReceived << endl;

	return byte;
}

void initUDPReceiver(){
	recSocketFD = socket(AF_INET, SOCK_DGRAM, 0);

	u_int yes = 1;
	setsockopt(recSocketFD, SOL_SOCKET, SO_REUSEADDR, (char*) &yes, sizeof(yes));

	memset(&addrRec, 0, sizeof(addrRec));
	addrRec.sin_family = AF_INET;
	addrRec.sin_addr.s_addr = htonl(INADDR_ANY); // differs from sender
	addrRec.sin_port = htons(port);
	bind(recSocketFD, (struct sockaddr*) &addrRec, sizeof(addrRec));

	struct ip_mreq mreq;
	mreq.imr_multiaddr.s_addr = inet_addr(group);
	mreq.imr_interface.s_addr = htonl(INADDR_ANY);
	setsockopt(recSocketFD, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*) &mreq, sizeof(mreq));

	fcntl(recSocketFD, F_SETFL, O_NONBLOCK);
}

void initUDPSender(){
	sendSocketFD = socket(AF_INET, SOCK_DGRAM, 0);
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
	int numbytes = recvfrom(recSocketFD, &buffer, bufferSize, 0,(struct sockaddr *) &addrRec, &addrLen);
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
	int numbytes = sendto(sendSocketFD, messageBuff, strlen(messageBuff), 0, (struct sockaddr*) &addrSend, sizeof(addrSend));
}

void sendUDP(string flag, string message){
	sendUDPRaw("0",to_string(blimpID),flag,message);
}

void sendUDP(string message){
	sendUDPRaw("0",to_string(blimpID),"D",message);
}

void establishBlimpID(){
	while(blimpID == -1){
		sendUDPRaw("0","N","N","N");
		sleep(0.25);
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
	cout << "Received blimp ID: " << blimpID << endl;
}

void plotUDP(string varName, float varValue){
	string message = varName + "=" + to_string(varValue);
	sendUDP("T",message);
}

//==================== VISION HELPER FUNCTIONS ===================

vector<Point> scaleContour(vector<Point> contour, float scale) {
    Moments moment = moments(contour);
    double cx = moment.m10 / moment.m00; //int(M['m10']/M['m00'])
    double cy = moment.m01 / moment.m00; //int(M['m01']/M['m00'])

    //shift all points
    for (unsigned int i = 0; i < contour.size(); i++) {
        contour[i].x = ((contour[i].x-cx)*scale)+cx;
        contour[i].y = ((contour[i].y-cy)*scale)+cy;
    }

    return contour;
}

//==================== HELPER FUNCTIONS ====================
//Timing
float getFPS(){
	clock_t currentTP = clock();
	double deltaTime = double(currentTP - lastTP) / CLOCKS_PER_SEC;
	lastTP = currentTP;
	return 1.0/deltaTime;
}

void benchmarkFirst(string flag){
	pthread_mutex_lock(&benchmarkMutex);
	times.clear();
	flags.clear();
	times.push_back(clock());
	flags.push_back(flag);
	pthread_mutex_unlock(&benchmarkMutex);
}

void benchmark(string flag) {
	pthread_mutex_lock(&benchmarkMutex);
	times.push_back(clock());
	flags.push_back(flag);
	pthread_mutex_unlock(&benchmarkMutex);
}

void benchmarkPrint() {
	pthread_mutex_lock(&benchmarkMutex);

	if(times.size() < 2) return;
	double deltaTotal = double(times[times.size()-1] - times[0]) / CLOCKS_PER_SEC;
	for (int i=1; i<(int)times.size(); i++) {
		double deltaTime = double(times[i] - times[i-1]) / CLOCKS_PER_SEC;
		float percentTime = deltaTime / deltaTotal * 100;
		percentTime = round(percentTime * 100) / 100;
		string percentString = to_string(percentTime);
		if(percentString.at(1) == '.') percentString = " " + percentString;
		percentString = percentString.substr(0,5);

		cout << percentString << "%, " << flags[i-1] << "->" << flags[i] << ": " << deltaTime << endl;
	}
	cout << "Total: " << 1.0/deltaTotal << "Hz" << endl;

	pthread_mutex_unlock(&benchmarkMutex);
}

void delay(double delaySeconds){
	clock_t start = clock();
	while(double(clock() - start)/CLOCKS_PER_SEC < delaySeconds);
}

VideoWriter outputVideo;
void saveToVideo(Mat frame){
	while(!outputVideo.isOpened()){
		cout << "Opening output video." << endl;
		outputVideo.open(outputVideo_fileName, cv::VideoWriter::fourcc('M','J','P','G'), outputVideo_fps, frame.size(), true);
	}

	if(!frame.empty()){
		outputVideo.write(frame);
	} else{
		cout << "Attempted to write empty frame." << endl;
	}
}

//==================== MAIN THREAD ====================//
int main(int argc, char** argv) {

	bool validUsage = true;

	//Parse commandline args
	for (int i = 1; i < argc; i++) {
		//Blimp ID setting
		if ((strcmp(argv[i], "-i") == 0) || (strcmp(argv[i], "--id") == 0)) {
			//Make sure second argument was provided
			if (i+1 < argc) {
				try {
					blimpID = stoi(argv[i+1]);
				} catch (std::invalid_argument& e) {
					fprintf(stderr, "Invalid argument \"%s\"\n", argv[i+1]);
					validUsage = false;
					break;
				}

				fprintf(stdout, "I. Am. Blimp. %d.\n", blimpID);

				//Increment i because we already used the next argument
				i++;
			} else {
				validUsage = false;
				break;
			}
		} else if ((strcmp(argv[i], "-v") == 0) || (strcmp(argv[i], "--verbose") == 0)) {
			//Verbose logging mode setting
			verboseMode = true;
			std::cout << "Verbose logging mode enabled." << std::endl;
		} else if ((strcmp(argv[i], "-s") == 0) || (strcmp(argv[i], "--stream") == 0)) {
			streamOnlyMode = true;
			std::cout << "Stream-only mode enabled." << std::endl;
		} else if ((strcmp(argv[i], "-c") == 0) || (strcmp(argv[i], "--cap-id") == 0)) {
			//Change capture ID setting
			//Make sure second argument was provided
			if (i+1 < argc) {
				try {
					cap_device_id = stoi(argv[i+1]);
				} catch (std::invalid_argument& e) {
					fprintf(stderr, "Invalid argument \"%s\"\n", argv[i+1]);
					validUsage = false;
					break;
				}

				fprintf(stdout, "Capture device ID set to %d\n", cap_device_id);

				//Increment i because we already used the next argument
				i++;
			} else {
				validUsage = false;
				break;
			}
		} else if ((strcmp(argv[i], "-a") == 0) || (strcmp(argv[i], "--annotate") == 0)){
			annotatedMode = true;
			std::cout << "Annotated mode enabled." << std::endl;
		} else if ((strcmp(argv[i], "-ds") == 0) || (strcmp(argv[i], "--disable-serial") == 0)){
			disableSerialMode = true;
			std::cout << "Disable Serial mode enabled." << std::endl;
		}else if((strcmp(argv[i], "-j") == 0) || (strcmp(argv[i], "--json") == 0)){
			printJSONMode = true;
			std::cout << "Print JSON mode enabled." << std::endl;
		} else if ((strcmp(argv[i], "-h") == 0) || (strcmp(argv[i], "--help") == 0)) {
			//Print help info and return
			std::string helpText;
			helpText += "req -i --id {blimp_id}: Set Blimp ID Number\n";
			helpText += "opt -v, --verbose: Enable Verbose Mode\n";
			helpText += "opt -c, --cap-id {cap_dev_id}: Set Capture Device ID\n";
			helpText += "opt -s --stream: Enable Stream-Only Mode\n";
			helpText += "-a --annotate: Enable Annotated Mode\n";
			helpText += "-j --json: Enable JSON Print Mode\n";
			helpText += "-ds --disable-serial: Disable Serial Mode\n";
			fprintf(stderr, "%s\n", helpText.c_str());
			return 0;
		} else {
			fprintf(stderr, "Invalid argument given: \"%s\"\n", argv[i]);
			validUsage = false;
			break;
		}
	}

	//Enforce blimp ID requirement
	if (blimpID < 0) {
		fprintf(stderr, "Error: Blimp ID not provided.\n");
		validUsage = false;
	}

	const char *usageMsg = "Usage: piOffboardStereoVision [-i {blimp_id}] [opt -v] [opt -c {cap_dev_id}] [opt -s]";
	if (!validUsage) {
		fprintf(stderr, "%s\n", usageMsg);
		return EXIT_FAILURE;
	}

    //Assign interrupt signals
    signal(SIGINT, 	stop_program);
    signal(SIGABRT, stop_program);
    signal(SIGKILL, stop_program);
    signal(SIGTERM, stop_program);
    signal(SIGTSTP, stop_program);

	clock_t currentTime = clock();
	clock_t last = clock();

	if (!streamOnlyMode) {
		//Initialize serial and UDP comms
		if (!disableSerialMode) {
			initSerial();
		}

		initUDPReceiver();
		initUDPSender();
	}

	//UDP video streamer
	init_udp_streamer();

	//Initialize semaphore
	sem_init(&cap_sem, 0, 0);

    //Initialize video streaming threads
    if (pthread_create(&stream_thread, NULL, stream_video, (void *)stream_thread_id) < 0) {
        perror("Stream pthread_create");
        exit(EXIT_FAILURE);
    }

    if (pthread_create(&stream_fb_thread, NULL, stream_fb_check, (void *)stream_fb_thread_id) < 0) {
        perror("Stream feedback pthread_create");
        exit(EXIT_FAILURE);
    }

    if (!streamOnlyMode) {
        if (pthread_create(&bs_udp_thread, NULL, bs_udp_comms, (void *)bs_udp_thread_id) < 0) {
            perror("Base station UDP comms pthread_create");
            exit(EXIT_FAILURE);
        }
    }
//    sleep(5.0);
//    stop_program(0);
//    return 0;

	//Read stereo calibration file
	Mat Left_Stereo_Map1, Left_Stereo_Map2;
	Mat Right_Stereo_Map1, Right_Stereo_Map2;
	Mat Q;

	if (verboseMode) cout << "Reading Stereo Camera Parameters" << endl;
	FileStorage cv_file2 = FileStorage(stereo_cal_path, FileStorage::READ);

	if (!cv_file2.isOpened()) {
		fprintf(stderr, "Error: Failed to open stereo parameter file %s\n", stereo_cal_path);
		stop_program(0);
	}

	cv_file2["Left_Stereo_Map_x"] >> Left_Stereo_Map1;
	cv_file2["Left_Stereo_Map_y"] >> Left_Stereo_Map2;
	cv_file2["Right_Stereo_Map_x"] >> Right_Stereo_Map1;
	cv_file2["Right_Stereo_Map_y"] >> Right_Stereo_Map2;
	cv_file2["Q"] >> Q;
	cv_file2.release();
	if (verboseMode) cout << "Read Complete" << endl;

	Ptr<StereoBM> left_matcher = StereoBM::create(16, 13); //Num disp, block size
	left_matcher->setPreFilterType(1);
	left_matcher->setPreFilterSize(PRE_FILTER_SIZE);
	left_matcher->setPreFilterCap(PRE_FILTER_CAP);
	left_matcher->setUniquenessRatio(UNIQUENESS_RATIO);

	Ptr<ximgproc::DisparityWLSFilter> wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);
	Ptr<StereoMatcher> right_matcher = cv::ximgproc::createRightMatcher(left_matcher);

	wls_filter->setLambda(LAMBDA);
	wls_filter->setSigmaColor(SIGMA);

	//for serial in case teensy restarts
	clock_t lastMsgTime = clock();
	clock_t lastSerial = 0;

    while (program_running) {

		sem_wait(&cap_sem);

		//Copy L/R frames to avoid race conditions
		cv::Mat imgL, imgR;
		lt_frame_lowres.copyTo(imgL);
		rt_frame_lowres.copyTo(imgR);

		//remap images for stereo
		Mat left_correct, right_correct;

		benchmark("Remap");
		remap(imgL,
				left_correct,
				Left_Stereo_Map1,
				Left_Stereo_Map2,
				INTER_AREA,
				BORDER_CONSTANT,
				0);

		remap(imgR,
				right_correct,
				Right_Stereo_Map1,
				Right_Stereo_Map2,
				INTER_AREA,
				BORDER_CONSTANT,
				0);

		//shrink to disparity size
		Mat left_small_correct, right_small_correct;
		resize(left_correct, left_small_correct, Size(DISP_WIDTH, DISP_HEIGHT), INTER_LINEAR);
		resize(right_correct, right_small_correct, Size(DISP_WIDTH, DISP_HEIGHT), INTER_LINEAR);

		Mat left_correct_sg, right_correct_sg;
		cvtColor(left_small_correct, left_correct_sg, cv::COLOR_BGR2GRAY);
		cvtColor(right_small_correct, right_correct_sg, cv::COLOR_BGR2GRAY);

		//benchmark("Compute Disparity");
		//compute stereo and filter
		Mat left_disp, right_disp;
		left_matcher->compute(left_correct_sg, right_correct_sg, left_disp);
		//benchmark("Left Disp");
		right_matcher->compute(right_correct_sg, left_correct_sg, right_disp);
		//benchmark("Right Disp");

		Mat filtered_disp;
		wls_filter->filter(left_disp, left_correct_sg, filtered_disp, right_disp);
		benchmark("WLS");

		//benchmark("Compute Depth");
		Mat xyz;
		reprojectImageTo3D(filtered_disp, xyz, Q, true, -1);
		Mat XYZ[3];
		Mat x, y, z;
		split(xyz, XYZ);
		z = XYZ[2];

		resize(z, z, Size(RECT_WIDTH, RECT_HEIGHT), INTER_LINEAR);

		//benchmark("remove inf");
		//infinite value removal
		Mat close;
		inRange(z*0.15, Scalar(0), Scalar(200), close);

		//add object avoidence detection here
		Mat objectMask;
		inRange(z*0.15, Scalar(0), Scalar(AVOID_DIST),  objectMask);

		vector<vector<Point> > contoursA;
		vector<Vec4i> heirarchyA;

		findContours(objectMask, contoursA, heirarchyA, RETR_TREE, CHAIN_APPROX_SIMPLE);

		vector<Point> largestContour;
		float largestArea = 0;

		for (unsigned int i = 0; i < contoursA.size(); i++) {
			double area = contourArea(contoursA[i]);
			if (heirarchyA[i][3] == -1 && (area > AVOID_AREA)) {

				if (area > largestArea) {
					largestArea = area;
					largestContour = contoursA[i];
				}
			}
		}

		int quad = 10;

		if (largestArea > 0) {
			Moments moment = moments(largestContour);
			float cx = moment.m10/moment.m00;
			float cy = moment.m01/moment.m00;

			int xx = (int)((cx/((float)RECT_WIDTH))*3.0)+1;
			int yy = (int)((cy/((float)RECT_HEIGHT))*3.0)+1;

			quad = 3*(yy-1)+xx;
		}

		if (verboseMode) {
			//cout << "Area: " << largestArea << endl;
			cout << "Quad: " << quad << endl;
			cout << "Targeting Goal: " << goalColor << endl;
    	}

		//I disabled frame recording for now to avoid possible race conditions.
		//Offboard recording will be happening soon anyways, so DONWORRYBOUTIT.
		//--Willie
//		if(framesLeftToRecord > 0){
//			framesLeftToRecord--;
//			saveToVideo(left_correct);
//			cout << "Recording video." << endl;
//		}

		benchmark("find target");
		std::vector<vector<float> > balloons;
		std::vector<vector<float> > goals;

		if (mode == searching || mode == approach || mode == catching) {
			//perform colorvision
			Mat bMask;
			Mat imgLhsv;

			Mat balloonCorrected;
			Mat balloonCorrectM = Mat::zeros(left_correct.size(), left_correct.type());
			balloonCorrectM.setTo(B_CORRECTION);
			add(left_correct,balloonCorrectM, balloonCorrected);

			cvtColor(balloonCorrected, imgLhsv, cv::COLOR_BGR2HSV);

			inRange(imgLhsv, B_MIN, B_MAX, bMask);

			vector<vector<Point> > contours;
			vector<Vec4i> hierarchy;

			findContours(bMask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

			//cout << "Number of contours: " << contours.size() << endl;

			// iterate through all the top-level contours,
			// draw each connected component with its own random color
			for (unsigned int i = 0; i < contours.size(); i++) {
				//perfom initial area filter
				double area = contourArea(contours[i]);
				if (hierarchy[i][3] == -1 && (area > MIN_AREA) && (contours[i].size() > 5)) {
					RotatedRect objEllipse = fitEllipse(contours[i]);

					Mat temp = Mat::zeros(z.rows, z.cols, CV_8UC1);

					if (objEllipse.size.width/objEllipse.size.height > 1/(float)SIZE_RATIO && objEllipse.size.width/objEllipse.size.height < SIZE_RATIO) {
						//compute distance from depth map

						vector<Point> smallcnt = scaleContour(contours[i], 1.0);
						RotatedRect smallEllipse = fitEllipse(smallcnt);

						ellipse(temp, smallEllipse, Scalar(255), -1);
						bitwise_and(temp, close, temp);

						Mat mean, stddev;
						cv::meanStdDev(z*CONVERSION, mean, stddev, temp);

						float x = objEllipse.center.x;
						float y = objEllipse.center.y;
						float pz = mean.at<double>(0,0);
						//cout << "BallDistance: " << pz << endl;

						if (pz < 0) {
							pz = pz*-1;
						}

						if (pz == 0) {
							pz = 10000;
						}

						if (pz > 10000) {
							pz = 10000;
						}

						//add to vector
						std::vector<float> balloon;
						balloon.push_back(x);
						balloon.push_back(y);
						balloon.push_back(pz);
						balloon.push_back(area);
						balloons.push_back(balloon);
						//cout << "Depth of object: " << pz << "in\tstd: " << stddev.at<double>(0,0)*conversion << endl;
					}
				}
			}
		} else if (mode == goalSearch || mode == approachGoal || mode == scoringStart) {
			Mat bMask;
			Mat imgLhsv;

			Mat imgGhsv;
			Mat GoalCorrected;
			Mat GoalCorrectM = Mat::zeros(left_correct.size(), left_correct.type());

			if (goalColor == orange) {
				GoalCorrectM.setTo(ORANGE_G_CORRECTION);
			} else {
				GoalCorrectM.setTo(YELLOW_G_CORRECTION);
			}

			add(left_correct,GoalCorrectM, GoalCorrected);

			cvtColor(GoalCorrected, imgGhsv, cv::COLOR_BGR2HSV);

			if (goalColor == orange) {
				inRange(imgGhsv, ORANGE_G_MIN, ORANGE_G_MAX, bMask);
			} else {
				inRange(imgGhsv, YELLOW_G_MIN, YELLOW_G_MAX, bMask);
			}

			Mat erosionElem = getStructuringElement(MORPH_ELLIPSE, Size(2*E_SIZE+1,2*E_SIZE+1),Point(E_SIZE, E_SIZE));
			Mat dilationElem = getStructuringElement(MORPH_ELLIPSE, Size(2*D_SIZE+1,2*D_SIZE+1),Point(D_SIZE, D_SIZE));

			erode(bMask, bMask, erosionElem, Point(-1,-1), 0);
			dilate(bMask, bMask, dilationElem, Point(-1,-1), D_ITER+GOAL_DILATION_ADJUST);

			//imshow("Goal Mask", bMask);

			vector<vector<Point> > contours;
			vector<Vec4i> hierarchy;

			findContours(bMask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

			for (unsigned int i = 0; i < contours.size(); i++) {
				//perfom initial area filter
				double area = contourArea(contours[i]);
				if (hierarchy[i][3] == -1 && (area > 1000)) {
					//temp matrix for drawing
					Mat temp = Mat::zeros(bMask.rows, bMask.cols, CV_8UC3);
					Mat tempSingle;

					unsigned int indexOfInner = i;

					vector<Point> approx;
					vector<vector<Point>> outApprox;
					vector<vector<Point>> outScaled;

					if (true) {
						//do approx poly to close contours and smooth them
						double epsilon = G_POLY_APPROX_E*cv::arcLength(contours[i],true);
						cv::approxPolyDP(contours[i], approx, epsilon,true);
						outApprox.push_back(approx);
						//scale contour
						outScaled.push_back(scaleContour(approx, GOAL_INNER_CONTOUR_SCALE));

						Mat innerSingle, goalSingle;
						Mat innerSingleBit, goalSingleBit;
						Mat inner = Mat::zeros(bMask.rows, bMask.cols, CV_8UC3);
						Mat goal = Mat::zeros(bMask.rows, bMask.cols, CV_8UC3);

						drawContours(inner, outScaled, 0, Scalar(255, 255, 255), FILLED, 8);
						drawContours(goal, contours, i, Scalar(255, 255, 255), FILLED, 8, hierarchy);

						cvtColor(inner, innerSingle, COLOR_BGR2GRAY);
						cvtColor(goal, goalSingle, COLOR_BGR2GRAY);

						//get just the goal detected
						bitwise_xor(goalSingle, goalSingle, goalSingle, innerSingle);

						//set as optical flow mask
						tempSingle = goalSingle;
						//bitwise_or(debug, tempSingle, debug);

						//imshow("goal", goalSingle);

						Mat debug = Mat::zeros(bMask.rows, bMask.cols, CV_8UC3);

						for (unsigned int i = 0; i < contours.size(); i++) {
							if (hierarchy[i][3] == -1) {
								drawContours(debug, contours, i, Scalar(255,0,0), LINE_4, 8, hierarchy);

								//do approx poly to close contours and smooth them
								double epsilon = G_POLY_APPROX_E*cv::arcLength(contours[i],true);
								cv::approxPolyDP(contours[i], approx, epsilon,true);
								outApprox.push_back(approx);
								//scale contour
								outScaled.push_back(scaleContour(approx, GOAL_INNER_CONTOUR_SCALE));
							}
						}

						for (unsigned int i = 0; i < outApprox.size(); i++) {
							drawContours(debug, outApprox, i, Scalar(0,255,0), LINE_4, 8);
							drawContours(debug, outScaled, i, Scalar(0,0,255), LINE_4, 8);
						}

						//imshow("orange goal contours", debug);

						//get area of each goal mask
						//float areaI = contourArea(outScaled[0]);
						//float areaG = contourArea(approx)-areaI;

						double areaI = (double)countNonZero(innerSingle);
						double areaG = (double)countNonZero(tempSingle);

						//get the pixels detected in each region
						bitwise_and(innerSingle, bMask, innerSingleBit);
						bitwise_and(tempSingle, bMask, goalSingleBit);

						double innerDetect = ((double)countNonZero(innerSingleBit))/areaI;
						double goalDetect = ((double)countNonZero(goalSingleBit))/areaG;

						//cout << "inner total" << areaI << endl;
						//cout << "inner detected" << innerDetect << endl;
						//cout << "goal total" << areaG << endl;
						//cout << "goal detected" << goalDetect<< endl;temp

						Mat depthMask;

						bitwise_and(tempSingle, close, depthMask);

						float confidence = 1-innerDetect;
						if (confidence > 0.5) {
							cout << "condfidence: " << confidence << endl;
							std::vector<float> goal;

							Point2f center;
							float radius;
							minEnclosingCircle(contours[i], center, radius);
							//cout <<"Crash 2" << endl;
							Mat mean, stddev;
							cv::meanStdDev(z, mean, stddev, depthMask);

							//cout << "Crash 3" << endl;
							float pz = (float)mean.at<double>(0,0)*CONVERSION;

							if (countNonZero(depthMask) < GOAL_CONFIRM) {
								pz = 10000;
							}

							if (pz < 0) {
								pz = pz*-1;
							}

							if (pz == 0) {
								pz = 10000;
							}
							//cout << "Goal Distance: " << pz << endl;

							goal.push_back(center.x);
							goal.push_back(center.y);
							goal.push_back(pz);
							goal.push_back(area);
							goals.push_back(goal);
						}
					}
				}
			}
		}

		clock_t now = clock();
		float time = (float)(now-last)/(float)CLOCKS_PER_SEC;
		//cout << "Vision Compute Time: " << time << endl;
		//cout << "Vision Compute Rate: " << 1/time << endl;
		last = now;

		//get data from base and combine with color vision

		//waitKey(1);

		//Select target for blimp depending on state:
		std::vector<std::vector<float> > target;

		if (annotatedMode) {
			pthread_mutex_lock(&annotatedFrameMutex);
			left_correct.copyTo(annotatedFrame);
		}

		//get largest balloon or goal depending on state
		//mode = goalSearch;
		if (mode == searching || mode == approach || mode == catching) {
			//send back balloon data
			float area = 0;
			int index = -1;
			for (unsigned int i = 0; i < balloons.size(); i++) {
				if (area < balloons[i][3]) {
					area = balloons[i][3];
					index = i;
				}
			}

			if (index != -1) {
				if(printJSONMode) cout << "Balloon seen with computer vision." << endl;
				target.push_back(balloons[index]);
				if (annotatedMode) {
					float radius = sqrt(balloons[index][3]/3.14159f);
					circle(annotatedFrame, Point(balloons[index][0],balloons[index][1]), radius, Scalar(255,255,255));
					cv::putText(annotatedFrame,
							"THIS IS THE FUTURE",
							cv::Point(10.0, 20.0),
							cv::FONT_HERSHEY_COMPLEX,
							0.75,
							CV_RGB(118, 185, 0),
							2);

					cv::putText(annotatedFrame,
							"z: " + to_string(balloons[index][2]),
							Point(balloons[index][0], balloons[index][1]+radius+5.0),
							cv::FONT_HERSHEY_PLAIN,
							1.0,
							Scalar(255,255,255),
							1.0);
				}
			} else{
				//No balloons found
				if (annotatedMode) {
					cv::putText(annotatedFrame,
							":(",
							cv::Point(10.0, 50.0),
							cv::FONT_HERSHEY_COMPLEX,
							2,
							CV_RGB(118, 185, 0),
							2);
				}

				//Check machine learning
				if(lastReceivedIsNew){
					lastReceivedIsNew = false;
					pthread_mutex_lock(&receivedTargetsMutex);
					lastReceivedTargetsCopy = lastReceivedTargets;
					lastReceivedTargetsTimeCopy = lastReceivedTargetsTime;
					pthread_mutex_unlock(&receivedTargetsMutex);
				}

				if((clock() - lastReceivedTargetsTimeCopy)/CLOCKS_PER_SEC < receivedTargetTimeout){
					//parse through array, looking for best balloon
					//cout << "json balloon: \"" << lastReceivedTargetsCopy << "\"" << endl;

					double bestX;
					double bestY;
					double bestZ;
					double bestArea = -1;
					for (json::iterator it = lastReceivedTargetsCopy.begin(); it != lastReceivedTargetsCopy.end(); ++it) {
						json target = *it;
						float currentArea = target["Area"];
						bool correctTargetType = target["class"] == ML_CLASS_BALLOON;
						if(correctTargetType && bestArea < currentArea){
							bestX = target["xCenter"];
							bestY = target["yCenter"];
							bestZ = 10000;
							bestArea = currentArea;
						}
					}

					if(bestArea != -1){
						if(printJSONMode) cout << "Balloon seen with machine vision." << endl;
						//Use machine learning balloon info!
						vector<float> targetInfo;
						targetInfo.push_back(bestX); // x
						targetInfo.push_back(bestY); // y
						targetInfo.push_back(bestZ); // z
						targetInfo.push_back(bestArea); // area
						target.push_back(targetInfo);
					}else{
						if(printJSONMode) cout << "Balloon not seen." << endl;
					}
				}else{
					if(printJSONMode) cout << "Balloon not seen." << endl;
				}
			}

		} else if (mode == goalSearch || mode == approachGoal || mode == scoringStart) {
			//send back goal data
			float area = 0;
			int index = -1;
			for (unsigned int i = 0; i < goals.size(); i++) {
				if (area < goals[i][3]) {
					area = goals[i][3];
					index = i;
				}
			}

			if (index != -1) {
				if(printJSONMode) cout << "Goal seen with computer vision." << endl;
				target.push_back(goals[index]);
			}else{
				//No goals found
				//Check machine learning
				if(lastReceivedIsNew){
					//cout << "JSON copied." << endl;
					lastReceivedIsNew = false;
					pthread_mutex_lock(&receivedTargetsMutex);
					lastReceivedTargetsCopy = lastReceivedTargets;
					lastReceivedTargetsTimeCopy = lastReceivedTargetsTime;
					pthread_mutex_unlock(&receivedTargetsMutex);
				}else{
					//cout << "No new JSON to copy." << endl;
				}

				if((clock() - lastReceivedTargetsTimeCopy)/CLOCKS_PER_SEC < receivedTargetTimeout){
					//parse through array, looking for best balloon
					//cout << "json goal: \"" << lastReceivedTargetsCopy << "\"" << endl;

					double bestX;
					double bestY;
					double bestZ;
					double bestArea = -1;
					//cout << "JSON:" << endl;
					for (json::iterator it = lastReceivedTargetsCopy.begin(); it != lastReceivedTargetsCopy.end(); ++it) {
						json target = *it;
						//cout << "" << target << endl;
						float currentArea = target["Area"];
						bool correctTargetType = (goalColor == orange && target["class"] == ML_CLASS_ORANGEGOAL) || (goalColor == yellow && target["class"] == ML_CLASS_YELLOWGOAL);
						//cout << "Area: " << currentArea << ", CorrectTargetType: " << correctTargetType << ", Class: " << target["class"] << endl;
						if(correctTargetType && bestArea < currentArea){
							bestX = target["xCenter"];
							bestY = target["yCenter"];
							bestZ = 10000;
							bestArea = currentArea;
						}
					}

					if(bestArea != -1){
						if(printJSONMode) cout << "Goal seen with machine vision." << endl;
						//Use machine learning balloon info!
						vector<float> targetInfo;
						targetInfo.push_back(bestX); // x
						targetInfo.push_back(bestY); // y
						targetInfo.push_back(bestZ); // z
						targetInfo.push_back(bestArea); // area
						target.push_back(targetInfo);
					}else{
						if(printJSONMode) cout << "Goal not seen." << endl;
					}
				}else{
					if(printJSONMode) cout << "Goal not seen." << endl;
				}
			}
		}

		if (annotatedMode) {
			pthread_mutex_unlock(&annotatedFrameMutex);
			annotatedFrameReady = true;
		}

		//Debuging
		//Print target for debugging
		if (verboseMode) {
			for (int i = 0; i < target.size(); i++) {
				cout << "X: " << target[i][0] << endl;
				cout << "Y: " << target[i][1] << endl;
				cout << "Z: " << target[i][2] << endl;
				cout << "Area: " << target[i][3] << endl;
			}
		}

		//Print serial data-----------------------------------------------------------------------------------------------------
		if (double(currentTime - lastSerial)/CLOCKS_PER_SEC > 0.1) {
			//create message to send to teensy
			String message;

			//build message
			if (autonomous) {
				message = "A&";
				message += to_string(quad) + "&" + to_string(barometerData) + "&";
				if (target.size() > 0) {
					for (unsigned int i = 0; i < 4; i++) {
						message += to_string(target[0][i]);
						message += ":";
					}
					message += "&";
				}
				message += "#\n";
			} else {
				message = "M";
				message += "&" + to_string(quad) + "&" + to_string(barometerData);
				for (int i=0; i < (int)recentMotorCommands.size(); i++) {
					message += "&" + to_string(recentMotorCommands[i]).substr(0,4);
					if (i == (int)recentMotorCommands.size()-1) {
						message += "&";
					}
				}
				message += "#\n";
			}

			//Implement lost state
			if ((clock()-lastUDPReceived)/CLOCKS_PER_SEC > UDPTimeout) {
				//UDP Timed-out
				message = "L&#\n";
			}

			//cout << message << endl << endl;

			//benchmark("Send message");
			//send message
			if (!streamOnlyMode && !disableSerialMode) {
				sendSerial(message);
			}

			//benchmark("Output2");

			//ignores timeouts for serial communication
			if (debugMode || disableSerialMode) continue;

			//benchmark("Listen from teensy");
			//read state data from teensy
			char c = readSerial();
			String modeString = "";
			String blimpString = "";
			String goalString = "";
			int counter = 0;

			std::vector<String> teensyKeys;
			std::vector<String> teensyValues;

			String tempKey = "";
			String tempValue = "";

			if (c != 0) {
				//byte read was valid
				bool reading = true;
				while (reading) {
					if (c == '#') {
						//update mode
						try {

							for (int i = 0; i < (int)msgTemp.length(); i++) {

								if (msgTemp[i] == ':') {
									if (counter % 2 == 1 && counter > 2) {

										teensyKeys.push_back(tempKey);
										tempKey = "";
									} else if (counter > 2) {

										teensyValues.push_back(tempValue);
										tempValue = "";
									}
									counter++;
									continue;
								}

								if (counter <= 2) {
									if (msgTemp[i] >= 48 && msgTemp[i] <= 57) {
										if (counter == 0) {
											modeString.push_back(msgTemp[i]);
										} else if (counter == 1) {
											blimpString.push_back(msgTemp[i]);
										} else if (counter == 2) {
											goalString.push_back(msgTemp[i]);
										}
									}
								}

								if (counter > 2) {
									if (counter % 2 == 1) {
										//cout << msgTemp[i] << endl;
										tempKey.push_back(msgTemp[i]);
									} else {
										//cout << msgTemp[i] << endl;
										tempValue.push_back(msgTemp[i]);
									}
								}
							}

							mode = stoi(modeString);
							blimpColor = stoi(blimpString);
							//goalColor = stoi(goalString);
						}
						catch(std::invalid_argument& e) {
							cout << "Invalid mode sent from Teensy!" << endl;
							cout << msgTemp << endl;
						}

						msgTemp = "";

					} else if (c != 0) {
						msgTemp += String(1, c);
					} else {
						reading = false;
						break;
					}

					c = readSerial();
				}
			}

			if (verboseMode) {
				if (teensyKeys.size() == teensyValues.size()) {
					for (int i = 0; i < (int)teensyKeys.size(); i++) {
						cout << teensyKeys[i] << ": " << teensyValues[i] << endl;
					}
				}
				cout << endl;
			}

			/*
			//print mode
			if (autonomous) {
				switch (mode) {
				case searching:
					cout << "Mode: Searching" << endl;
					break;
				case approach:
					cout << "Mode: Approach Game Ball" << endl;
					break;
				case catching:
					cout << "Mode: Catching Game Ball" << endl;
					break;
				case caught:
					cout << "Mode: Ball Caught, Resetting Search" << endl;
					break;
				case goalSearch:
					cout << "Mode: Goal Search" << endl;
					break;
				case approachGoal:
					cout << "Mode: Approach Goal" << endl;
					break;
				case scoringStart:
					cout << "Mode: Positioning For Scoring" << endl;
					break;
				case shooting:
					cout << "Mode: Shooting" << endl;
					break;
				case scored:
					cout << "Mode: Scored, Reseting Search" << endl;
					break;
				default:
					cout << "Mode: Invalid" << endl;
					break;
				}
			} else {
				cout << "Mode: Manual" << endl;
			}
			*/

			//benchmarkPrint();
		}

		benchmark("Last");

    } //end while (main program loop)

    stop_program(0); //never called
    return 0;
}
