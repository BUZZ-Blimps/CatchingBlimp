//C++ includes
//#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <iostream>
#include <list>
#include <vector>
#include <unordered_map>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/ximgproc.hpp>
//#include <vector>

//C includes
#include <errno.h>
#include <fcntl.h>
#include <libv4l2.h>
#include <linux/videodev2.h>
#include <stdio.h>

#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <thread>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>
#include <stdlib.h>

#include "Teleplot.h"
#include "EnumUtil.h"
#include "ComputerVision.h"
#include "PiComm.h"

using namespace std;
using namespace cv;
using namespace chrono;


//==================== FUNCTION HEADERS ====================
VideoCapture openCamera(int camIndex, int camWidth, int camHeight);
vector<vector<float> > getObjects(int type, Mat mask);
//bool initPython();
//string runYolo(Mat& frame);


//==================== COMMUNICATION FUNCTION HEADERS ====================
void establishBlimpID();

//==================== HELPER FUNCTION HEADERS ====================
float getFPS();
void benchmarkFirst(string flag);
void benchmark(string flag);
void benchmarkPrint();
void delay(double delaySeconds);
void saveToVideo(Mat frame);
void printMode(bool autonomous, autoState mode);

//==================== GLOBAL VARIABLES ====================
string blimpID = "";
int teensyState;
bool autonomous = false;

bool scoreInOrange = false;
bool selfIsBlue = false;

blimpType blimpColor = blue;
goalType goalColor = orange;

int framesLeftToRecord;

bool debugMode = false;

String msgTemp = "";
autoState mode = searching;

float barometerData = 0;

string outputVideo_fileName = "outputVideo.avi";
double outputVideo_fps = 30;

float lastBaroMessageTime = 0.0;

Teleplot teleplot("127.0.0.1");

PiComm piComm;
ComputerVision computerVision;

//==================== MAIN ====================

int main(int argc, char** argv) {

	// Parse arguments
	for(int i=0; i<argc; i++) cout << "Arg[" << i << "] = \"" << argv[i] << "\"" << endl;

	// Determine blimp ID
	if(argc == 2){
		// Use argument to override blimpID
		blimpID = string(argv[1]);
	}else{
		// DEFAULT, Use IP Address as blimpID
		blimpID = piComm.getIPAddress();
	}
	cout << "I. Am. Blimp. " << blimpID << "." << endl;
	piComm.setBlimpID(blimpID);

	clock_t currentTime = clock();

	piComm.initSerial();

	computerVision.init();

	framesLeftToRecord = 9000;
	int moreFramesPerTrigger = 30 * 10;

	piComm.initUDPReceiver();
	piComm.initUDPSender();
	//establishBlimpID();

    vector<float> recentMotorCommands;

    clock_t lastCycle = 0;
    clock_t lastHeartbeat = 0;
    clock_t lastSerial = 0;
    clock_t lastUDPReceived = 0;

	clock_t last = clock();

	//for serial in case teensy restarts
	clock_t lastMsgTime = clock();

	//read stereo calibration file
	computerVision.readCalibrationFiles();
	
	while(true) {
		cout << "Blimp ID: " << blimpID << endl;
		clock_t currentTime = clock();
		double cycleTime = double(currentTime - lastCycle)/CLOCKS_PER_SEC;
		lastCycle = currentTime;
		//cout << "Cycle Time (s): " << cycleTime << endl;


		//Send heartbeat
		if(double(currentTime - lastHeartbeat)/CLOCKS_PER_SEC > 0.1){
			lastHeartbeat = currentTime;
			//sendUDP("H");
			piComm.sendUDP("S",to_string(mode));
		}

		//Receive UDP messages
		bool reading = true;
		while(reading){
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
			
		}
		
		if (clock()/((float)CLOCKS_PER_SEC) - lastBaroMessageTime > 10) {
			barometerData = -10000;
			cout << "Baro data not current" << endl;
		}
		
		computerVision.update(mode, goalColor);
		int quad = computerVision.getQuad();

		clock_t now = clock();
		float time = (float)(now-last)/(float)CLOCKS_PER_SEC;
		//cout << "Vision Compute Time: " << time << endl;
		//cout << "Vision Compute Rate: " << 1/time << endl;
		last = now;

		//Select target for blimp depending on state:
		std::vector<std::vector<float> > target;
		//get largest balloon or goal depending on state
		if (mode == searching || mode == approach || mode == catching) {
			target = computerVision.getTargetBalloon();
		} else if (mode == goalSearch || mode == approachGoal || mode == scoringStart) {
			target = computerVision.getTargetGoal();
		}

		//Debugging
		//print target
		for (int i = 0; i < target.size(); i++) {
			cout << "X: " << target[i][0] << endl;
			cout << "Y: " << target[i][1] << endl;
			cout << "Z: " << target[i][2] << endl;
			cout << "Area: " << target[i][3] << endl;
		}

		//Print serial data-----------------------------------------------------------------------------------------------------
		if(double(currentTime - lastSerial)/CLOCKS_PER_SEC > 0.1){
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
				for(int i=0; i<recentMotorCommands.size(); i++){
					message += "&" + to_string(recentMotorCommands[i]).substr(0,4);
					if(i == recentMotorCommands.size()-1){
						message += "&";
					}
				}
				message += "#\n";
			}

			//imlement lost state
			if((clock()-lastUDPReceived)/CLOCKS_PER_SEC > UDPTimeout){
				//UDP Timed-out
				message = "L&#\n";
			}

			cout << message << endl << endl;

			//send message
			piComm.sendSerial(message);



			//ignores timeouts for serial communication
			if(debugMode) continue;

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

							for (int i = 0; i < msgTemp.length(); i++) {

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
			
			cout << endl;
			if (teensyKeys.size() == teensyValues.size()) {
				for (int i = 0; i < teensyKeys.size(); i++) {
					cout << teensyKeys[i] << ": " << teensyValues[i] << endl;
				}
			}
			cout << endl;

			//printMode(autonomous, mode);
		}
	}

	cout << "DONE" << endl;

	return 0;
}

//==================== HELPER FUNCTION HEADERS ====================
clock_t lastTP = 0;
float getFPS(){
	clock_t currentTP = clock();
	double deltaTime = double(currentTP - lastTP) / CLOCKS_PER_SEC;
	lastTP = currentTP;
	return 1.0/deltaTime;
}

vector<clock_t> times;
vector<string> flags;
void benchmarkFirst(string flag){
	times.clear();
	flags.clear();
	times.push_back(clock());
	flags.push_back(flag);
}

void benchmark(string flag){
	times.push_back(clock());
	flags.push_back(flag);
}

void benchmarkPrint(){
	if(times.size() < 2) return;
	double deltaTotal = double(times[times.size()-1] - times[0]) / CLOCKS_PER_SEC;
	for(int i=1; i<times.size(); i++){
		double deltaTime = double(times[i] - times[i-1]) / CLOCKS_PER_SEC;
		float percentTime = deltaTime / deltaTotal * 100;
		percentTime = round(percentTime * 100) / 100;
		string percentString = to_string(percentTime);
		if(percentString.at(1) == '.') percentString = " " + percentString;
		percentString = percentString.substr(0,5);

		cout << percentString << "%, " << flags[i-1] << "->" << flags[i] << ": " << deltaTime << endl;
	}
	cout << "Total: " << 1.0/deltaTotal << "Hz" << endl;
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
	}else{
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