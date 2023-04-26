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

#include "piOffboardStereoVision.h"

using namespace std;
using namespace cv;

//==================== HELPER FUNCTIONS ====================//

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
				fprintf(stdout, "I. Am. Blimp. %s.\n", programData->customBlimpID.c_str());

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
				fprintf(stdout, "Capture device ID set to %d\n", programData->customCaptureID);

				//Increment i because we already used the next argument
				i++;
			} else {
				return false;
			}
		} else if((strcmp(argv[i], "-a") == 0) || (strcmp(argv[i], "--annotate") == 0)){
			programData->annotatedMode = true;
			std::cout << "Annotated mode enabled." << std::endl;
		}else if((strcmp(argv[i], "-ds") == 0) || (strcmp(argv[i], "--disable-serial") == 0)){
			programData->disableSerialMode = true;
			std::cout << "Disable Serial mode enabled." << std::endl;
		}else if(strcmp(argv[i], "--disable-stream") == 0){
			programData->disableStreamMode = true;
			std::cout << "Disable Stream mode enabled." << std::endl;
		}else if((strcmp(argv[i], "-j") == 0) || (strcmp(argv[i], "--json") == 0)){
			programData->printJSONMode = true;
			std::cout << "Print JSON mode enabled." << std::endl;
		}else if((strcmp(argv[i], "-b") == 0) || (strcmp(argv[i], "--barometer") == 0)){
			programData->printBaroMode = true;
			std::cout << "Print Barometer mode enabled." << std::endl;
		}else if((strcmp(argv[i], "-f") == 0) || (strcmp(argv[i], "--force") == 0)){
			programData->forceStreamMode = true;
			std::cout << "Force Stream mode enabled." << std::endl;
		} else if ((strcmp(argv[i], "-h") == 0) || (strcmp(argv[i], "--help") == 0)) {
			//Print help info and return
			std::string helpText;
			helpText += "req -i --id {blimp_id}: Set Blimp ID Number\n";
			helpText += "opt -v, --verbose: Enable Verbose Mode\n";
			helpText += "opt -c, --cap-id {cap_dev_id}: Set Capture Device ID\n";
			helpText += "opt -s --stream: Enable Stream-Only Mode\n";
			helpText += "opt -a --annotate: Enable Annotated Mode\n";
			helpText += "opt -j --json: Enable JSON Print Mode\n";
			helpText += "opt -ds --disable-serial: Disable Serial Mode\n";
			helpText += "opt -b --barometer: Enable Barometer Print Mode\n";
			helpText += "opt --disable-stream: Disable Stream Mode\n";
			helpText += "opt -f --force: Enable Force Stream Mode\n";
			fprintf(stderr, "%s\n", helpText.c_str());
			return false;
		} else {
			fprintf(stderr, "Invalid argument given: \"%s\"\n", argv[i]);
			return false;
		}
	}
	return true;
}

string findSourceDir(char** argv){
	// Find current working directory
	int cwdBufSize = 200;
    char cwdBuf[cwdBufSize];
	getcwd(cwdBuf, cwdBufSize);
	string cwd(cwdBuf);

	// Find executable path
	string exePath(argv[0]);
	int lastSlash = exePath.find_last_of('/');
	exePath = exePath.substr(1, lastSlash-1); // Only want everything between the leading period and the final slash ("/")

	// Find total path
	string srcDirPath = cwd + exePath + "/..";
	return srcDirPath;
}

void stop_program(int signal){
	programData.program_running = false;
	piComm.end();
	cameraHandler.end();
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

	// Assign interrupt signals
	signal(SIGINT, stop_program);
	signal(SIGABRT, stop_program);
	signal(SIGKILL, stop_program);
	signal(SIGTERM, stop_program);
	signal(SIGTSTP, stop_program);

	// START COMMUNICATION
	piComm.init(&programData);

	// INIT CAMERA
	cameraHandler.init(&piComm, &programData);

	// INIT COMPUTER VISION
	string srcDir = findSourceDir(argv); // Required for reading calibration files
	computerVision.init(&programData, srcDir, &piComm);

	clock_t last = 0;

	//for serial in case teensy restarts
	clock_t lastMsgTime = 0;
	clock_t lastSerial = 0;

    while (programData.program_running) {
		clock_t currentTime = clock();

		// Get feedback from basestation
		BSFeedbackData BSFeedback = piComm.getBSFeedback();
		goalType goalColor = BSFeedback.goalColor;
		programData.autonomous = BSFeedback.autonomous;

		// Get recent frames
		Mat frame_raw, frame_L, frame_R;
		bool recentFrames = cameraHandler.getRecentFrames(&frame_raw, &frame_L, &frame_R);
		if(!recentFrames) continue;
		videoSaver.writeFrame(frame_raw);

		// Do computer vision
		computerVision.update(frame_L, frame_R, piComm.getMode(), goalColor);
		int quad = computerVision.getQuad();

		// Get feedback from machine learning
		MLFeedbackData MLFeedback = piComm.getMLData();

		float time = (float)(currentTime-last)/(float)CLOCKS_PER_SEC;
		//cout << "Vision Compute Time: " << time << endl;
		//cout << "Vision Compute Rate: " << 1/time << endl;
		last = currentTime;

		computerVision.left_correct.copyTo(annotatedFrame);

		//Select largest target for blimp depending on state:
		std::vector<std::vector<float> > target;

		autoState currMode = piComm.getMode();
		if (currMode == searching || currMode == approach || currMode == catching) {
			target = computerVision.getTargetBalloon();

			if(target.size() > 0){
				vector<float> balloon = target[0];
				if(programData.printJSONMode) fprintf(stdout, "Balloon seen with computer vision.\n");
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
			} else{
				//No balloons found
				cv::putText(annotatedFrame,
						":(",
						cv::Point(10.0, 50.0),
						cv::FONT_HERSHEY_COMPLEX,
						2,
						CV_RGB(118, 185, 0),
						2);

				if((clock() - MLFeedback.lastReceivedTargetsTime)/CLOCKS_PER_SEC < receivedTargetTimeout){
					//parse through array, looking for best balloon
					double bestX;
					double bestY;
					double bestZ;
					double bestArea = -1;
					for (json::iterator it = MLFeedback.lastReceivedTargets.begin(); it != MLFeedback.lastReceivedTargets.end(); ++it) {
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
						if(programData.printJSONMode) fprintf(stdout, "Balloon seen with machine vision.\n");
						//Use machine learning balloon info!
						vector<float> targetInfo;
						targetInfo.push_back(bestX); // x
						targetInfo.push_back(bestY); // y
						targetInfo.push_back(bestZ); // z
						targetInfo.push_back(bestArea); // area
						target.push_back(targetInfo);
					}else{
						if(programData.printJSONMode) cout << "Balloon not seen." << endl;
					}
				}else{
					if(programData.printJSONMode) cout << "Balloon not seen." << endl;
				}
			}

		} else if (currMode == goalSearch || currMode == approachGoal || currMode == scoringStart) {
			target = computerVision.getTargetGoal();

			if (target.size() > 0) {
				if(programData.printJSONMode) cout << "Goal seen with computer vision." << endl;
			}else{
				//No goals found

				if((clock() - MLFeedback.lastReceivedTargetsTime)/CLOCKS_PER_SEC < receivedTargetTimeout){
					//parse through array, looking for best balloon

					double bestX;
					double bestY;
					double bestZ;
					double bestArea = -1;
					//cout << "JSON:" << endl;
					for (json::iterator it = MLFeedback.lastReceivedTargets.begin(); it != MLFeedback.lastReceivedTargets.end(); ++it) {
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
						if(programData.printJSONMode) cout << "Goal seen with machine vision." << endl;
						//Use machine learning balloon info!
						vector<float> targetInfo;
						targetInfo.push_back(bestX); // x
						targetInfo.push_back(bestY); // y
						targetInfo.push_back(bestZ); // z
						targetInfo.push_back(bestArea); // area
						target.push_back(targetInfo);
					}else{
						if(programData.printJSONMode) cout << "Goal not seen." << endl;
					}
				}else{
					if(programData.printJSONMode) cout << "Goal not seen." << endl;
				}
			}
		}

		// If annotated mode, stream the annotated frame
		if (programData.annotatedMode || true) {
			//piComm.setStreamFrame(annotatedFrame, "Annotated");
		}

		//Debuging
		//Print target for debugging
		if (programData.verboseMode) {
			for (int i = 0; i < target.size(); i++) {
				cout << "X: " << target[i][0] << endl;
				cout << "Y: " << target[i][1] << endl;
				cout << "Z: " << target[i][2] << endl;
				cout << "Area: " << target[i][3] << endl;
			}
		}

		//Print serial data-----------------------------------------------------------------------------------------------------
		if (double(currentTime - lastSerial)/CLOCKS_PER_SEC > 0.1) {
			lastSerial = currentTime;
			//create message to send to teensy
			String message;

			//build message
			if (BSFeedback.autonomous) {
				message = "A&";
				message += to_string(quad) + "&" + to_string(BSFeedback.barometerData) + "&";
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
				message += "&" + to_string(quad) + "&" + to_string(BSFeedback.barometerData);
				for (int i=0; i < (int)BSFeedback.recentMotorCommands.size(); i++) {
					message += "&" + to_string(BSFeedback.recentMotorCommands[i]).substr(0,4);
					if (i == (int)BSFeedback.recentMotorCommands.size()-1) {
						message += "&";
					}
				}
				message += "#\n";
			}

			//Implement lost state
			if ((clock()-BSFeedback.lastUDPReceived)/CLOCKS_PER_SEC > UDPTimeout) {
				//UDP Timed-out
				message = "L&#\n";
			}

			//Send message over serial
			if (!programData.streamOnlyMode && !programData.disableSerialMode) {
				piComm.sendSerial(message);
			}

			//Ignores timeouts for serial communication
			if (programData.disableSerialMode) continue;

			//benchmark("Listen from teensy");
			//read state data from teensy
			char c = piComm.readSerial();
			//String modeString = "";
			//String blimpString = "";
			//String goalString = "";
			//int counter = 0;

			//std::vector<String> teensyKeys;
			//std::vector<String> teensyValues;

			//String tempKey = "";
			//String tempValue = "";

			if (c != 0) {
				//byte read was valid
				bool reading = true;
				while (reading) {
					if (c == '#') {
						//fprintf(stdout, "Read from teensy: %s\n", msgTemp.c_str());
						//update mode
						/*
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

							piComm.setMode(static_cast<autoState>(stoi(modeString)));
							fprintf(stdout, "ModeString=%s, mode=%d\n", modeString.c_str(), piComm.getMode());
							//blimpColor = static_cast<blimpType>(stoi(blimpString));
							//goalColor = stoi(goalString);
						}
						catch(std::invalid_argument& e) {
							cout << "Invalid mode sent from Teensy!" << endl;
							cout << msgTemp << endl;
						}
						*/
						

						// Parse through feedback (delimited by :)
						vector<string> feedbackStrings;
						char delim = ':';
						int prevDelim = -1;
						while(true){
							int nextDelim = msgTemp.find_first_of(delim, prevDelim+1);
							if(nextDelim == string::npos){
								break;
							}else{
								string currentFeedback = msgTemp.substr(prevDelim+1, nextDelim-prevDelim-1);
								feedbackStrings.push_back(currentFeedback);
							}
							prevDelim = nextDelim;
						}

						// Use feedback
						//cout << "Feedbacksize: " << feedbackStrings.size() << endl;

						if(false){
							for(int i=0; i<feedbackStrings.size(); i++){
								cout << feedbackStrings[i] << ", ";
							}
							cout << endl;
						}

						string modeString = feedbackStrings[0];
						string blimpString = feedbackStrings[1];
						string goalString = feedbackStrings[2];
						string baroDataString = feedbackStrings[3];

						try{
							piComm.setMode(static_cast<autoState>(stoi(modeString)));
							baroData = stof(baroDataString);
							if(programData.printBaroMode){
								cout << "BaroData: " << baroData << endl;
							}
						}catch(invalid_argument& e){	
							cout << "Invalid mode sent from Teensy!" << endl;
							cout << msgTemp << endl;
						}

						msgTemp = "";
					} else if (c != 0) {
						//msgTemp += String(1, c);
						msgTemp += c;
					} else {
						reading = false;
						break;
					}

					c = piComm.readSerial();
					//cout << "Read " << c << endl;
				}
			}

			/*
			if (programData.verboseMode) {
				if (teensyKeys.size() == teensyValues.size()) {
					for (int i = 0; i < (int)teensyKeys.size(); i++) {
						cout << teensyKeys[i] << ": " << teensyValues[i] << endl;
					}
					if(teensyKeys.size() > 0) cout << endl;
				}
			}
			*/
		}

    } //end while (main program loop)

	stop_program(0);
    return 0;
}
