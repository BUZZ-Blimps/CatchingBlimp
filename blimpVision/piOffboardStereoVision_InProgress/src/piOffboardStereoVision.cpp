//============================================================================
// Name        : piOffboardStereoVision.cpp
// Author      : Adam Pooley, Aidan Amstutz, Willie Warke
// Version     : 3.0
// Copyright   : Copyright 2023 SWAMP INCORPORATED
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

	fprintf(stdout, "Connecting to base station. Blimp ID: %s.\n", blimpID.c_str());

	while (program_running) {
		clock_t currentTime = clock();
		double cycleTime = double(currentTime - lastCycle)/CLOCKS_PER_SEC;
		lastCycle = currentTime;
		//cout << "Cycle Time (s): " << cycleTime << endl;
		//benchmarkFirst("Heart");

		//Send heartbeat
		if (double(currentTime - lastHeartbeat)/CLOCKS_PER_SEC > HEARTBEAT_PERIOD) {
			lastHeartbeat = currentTime;
			//sendUDP("H");
			piComm.sendUDP("S", to_string(mode));
		}

		//Receive UDP messages
		bool reading = true;
		while(reading) {
			string readIn;
			string target;
			string flag;
			bool success = piComm.readUDP(&readIn, &target, nullptr, &flag);
			
			if(!success){
				// No more messages, stop reading for now
				reading = false;

			}else{
				// If we are not the target for this message, skip message
				if(!piComm.validTarget(target)) continue;

				lastUDPReceived = clock();
				//cout << "Received: \"" << readIn << "\"" << endl;
				
				if(flag == FLAG_AUTONOMOUS){
					autonomous = true;

					bool newBaroDataValid;
					bool parseSuccess = piComm.parseAutonomousMessage(readIn, &newBaroDataValid, &barometerData, &goalColor);
					
					if(parseSuccess && newBaroDataValid){
						//reset timer
						clock_t now = clock();
						lastBaroMessageTime = now/(float)CLOCKS_PER_SEC;
					}
					
				}else if(flag == FLAG_MANUAL){
					autonomous = false;
					
					bool newBaroDataValid;
					bool parseSuccess = piComm.parseManualMessage(readIn, &recentMotorCommands, &newBaroDataValid, &barometerData, &goalColor);

					if(parseSuccess && newBaroDataValid){
						//reset timer
						clock_t now = clock();
						lastBaroMessageTime = now/(float)CLOCKS_PER_SEC;
					}
					
				}else if(flag == FLAG_BAROMETER){
					bool newBaroDataValid = piComm.parseBarometer(readIn, &barometerData);
					
					if (newBaroDataValid) {
						//reset timer
						clock_t now = clock();
						lastBaroMessageTime = now/(float)CLOCKS_PER_SEC;
					}
					
				}else if(flag == FLAG_PARAMETER){
					char first = readIn.at(0);
					if(first == 'C'){
						cout << "Start recording." << endl;
						framesLeftToRecord += moreFramesPerTrigger;
					}

				}else if(flag == FLAG_KILL){
					cout << "Received kill command. Killing..." << endl;
					return 0;

				}else if(flag == FLAG_TARGETGOAL){
					if(readIn == "O"){
						goalColor = orange;
					}else if(readIn == "Y"){
						goalColor = yellow;
					}
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

//Video streaming thread
void *stream_video(void *thread_id) {
	int t_id = (int)pthread_self();
	fprintf(stdout, "Video streaming thread (%d) successfully started.\n", t_id);

	cv:: Mat image;

    //Set up video capture
	cv::VideoCapture cap;

	int cap_api_id = cv::CAP_V4L; //cv::CAP_ANY;
	if(cap_device_id == -1){
		cap_device_id = CAMERA_INDEX;
	}
	bool cameraOpened = false;
    cap.open(cap_device_id, cap_api_id);
	if(!cap.isOpened()){
		cap.open(CAMERA_INDEX_BACKUP, cap_api_id);
		if(cap.isOpened()){
			cap_device_id = CAMERA_INDEX_BACKUP;
		}else{
			std::cerr << "ERROR! Unable to open camera\n";
			return NULL;
		}
	}
	fprintf(stdout, "Successfully opened camera (index=%d).\n");

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

//==================== COMMUNICATION FUNCTION HEADERS ====================

void plotUDP(string varName, float varValue){
	string message = varName + "=" + to_string(varValue);
	piComm.sendUDP("T",message);
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

void printMode(bool autonomous, autoState mode){
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
}

bool parseCommandLineArgs(int argc, char** argv, ProgramData* programData){
	//Parse commandline args
	for (int i = 1; i < argc; i++) {
		//Blimp ID setting
		if ((strcmp(argv[i], "-i") == 0) || (strcmp(argv[i], "--id") == 0)) {
			//Make sure second argument was provided
			if (i+1 < argc) {
				programData->setBlimpID = true;
				programData->customBlimpID = argv[i+1];
				fprintf(stdout, "I. Am. Blimp. %s.\n", blimpID);

				//Increment i because we already used the next argument
				i++;
			} else {
				return false;
			}
		} else if ((strcmp(argv[i], "-v") == 0) || (strcmp(argv[i], "--verbose") == 0)) {
			//Verbose logging mode setting
			programData->verboseMode = true;
			std::cout << "Verbose logging mode enabled." << std::endl;
		} else if ((strcmp(argv[i], "-s") == 0) || (strcmp(argv[i], "--stream") == 0)) {
			programData->streamOnlyMode = true;
			std::cout << "Stream-only mode enabled." << std::endl;
		} else if ((strcmp(argv[i], "-c") == 0) || (strcmp(argv[i], "--cap-id") == 0)) {
			//Change capture ID setting
			//Make sure second argument was provided
			if (i+1 < argc) {
				int customCaptureIDTemp;
				try {
					customCaptureIDTemp = stoi(argv[i+1]);
				} catch (std::invalid_argument& e) {
					fprintf(stderr, "Invalid argument \"%s\"\n", argv[i+1]);
					return false;
				}

				programData->setCaptureID = true;
				programData->customCaptureID = customCaptureIDTemp;
				fprintf(stdout, "Capture device ID set to %d\n", cap_device_id);

				//Increment i because we already used the next argument
				i++;
			} else {
				return false;
			}
		} else if ((strcmp(argv[i], "-a") == 0) || (strcmp(argv[i], "--annotate") == 0)){
			programData->annotatedMode = true;
			std::cout << "Annotated mode enabled." << std::endl;
		} else if ((strcmp(argv[i], "-ds") == 0) || (strcmp(argv[i], "--disable-serial") == 0)){
			programData->disableSerialMode = true;
			std::cout << "Disable Serial mode enabled." << std::endl;
		}else if((strcmp(argv[i], "-j") == 0) || (strcmp(argv[i], "--json") == 0)){
			programData->printJSONMode = true;
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
			return false;
		} else {
			fprintf(stderr, "Invalid argument given: \"%s\"\n", argv[i]);
			return false;
		}
	}
}

//==================== MAIN THREAD ====================//
int main(int argc, char** argv) {
	// Create struct to hold parameters set as command line arguments
	ProgramData programData;
	programData.program_running = true;

	// Parse command line input
	bool validUsage = parseCommandLineArgs(argc, argv, &programData);
	
	// Confirm valid usage of command line arguments
	if (!validUsage) {
		const char *usageMsg = "Usage: piOffboardStereoVision [-i {blimp_id}] [opt -v] [opt -c {cap_dev_id}] [opt -s]";
		fprintf(stderr, "%s\n", usageMsg);
		return EXIT_FAILURE;
	}

    //Assign interrupt signals
    signal(SIGINT, 	stop_program);
    signal(SIGABRT, stop_program);
    signal(SIGKILL, stop_program);
    signal(SIGTERM, stop_program);
    signal(SIGTSTP, stop_program);


	// START COMMUNICATION
	piComm.init(&programData);

	// INIT CAMERA
	cameraHandler.init(&piComm, &programData);

	// INIT COMPUTER VISION
	computerVision.init(&programData);

	clock_t currentTime = clock();
	clock_t last = clock();

	//for serial in case teensy restarts
	clock_t lastMsgTime = clock();
	clock_t lastSerial = 0;

    while (program_running) {
		// Get recent frames
		cameraHandler.getRecentFrames(&lt_frame_lowres, &rt_frame_lowres);

		computerVision.update(lt_frame_lowres, rt_frame_lowres, mode, goalColor, verboseMode);
		int quad = computerVision.getQuad();

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
			computerVision.left_correct.copyTo(annotatedFrame);
		}

		//get largest balloon or goal depending on state
		//mode = goalSearch;
		if (mode == searching || mode == approach || mode == catching) {
			target = computerVision.getTargetBalloon();

			if(target.size() > 0){
				vector<float> balloon = target[0];
				if(printJSONMode) cout << "Balloon seen with computer vision." << endl;
				if (annotatedMode) {
					float radius = sqrt(balloon[3]/3.14159f);
					circle(annotatedFrame, Point(balloon[0],balloon[1]), radius, Scalar(255,255,255));
					cv::putText(annotatedFrame,
							"THIS IS THE FUTURE",
							cv::Point(10.0, 20.0),
							cv::FONT_HERSHEY_COMPLEX,
							0.75,
							CV_RGB(118, 185, 0),
							2);

					cv::putText(annotatedFrame,
							"z: " + to_string(balloon[2]),
							Point(balloon[0], balloon[1]+radius+5.0),
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
			target = computerVision.getTargetGoal();

			if (target.size() > 0) {
				if(printJSONMode) cout << "Goal seen with computer vision." << endl;
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
				piComm.sendSerial(message);
			}

			//benchmark("Output2");

			//ignores timeouts for serial communication
			if (debugMode || disableSerialMode) continue;

			//benchmark("Listen from teensy");
			//read state data from teensy
			char c = piComm.readSerial();
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

							mode = static_cast<autoState>(stoi(modeString));
							blimpColor = static_cast<blimpType>(stoi(blimpString));
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

					c = piComm.readSerial();
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

			//printMode(autonomous, mode);

			//benchmarkPrint();
		}

		//benchmark("Last");

    } //end while (main program loop)

    stop_program(0); //never called
    return 0;
}
