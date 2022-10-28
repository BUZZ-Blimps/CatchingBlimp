//C++ includes
//#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <iostream>
#include <list>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
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

#include <wiringPi.h>
#include <wiringSerial.h>

//Python includes
//#include <wchar.h>
//#include <Python.h>


//==================== CONSTANTS ====================
//Communication
#define UDP_IP				"239.255.255.250"
#define UDP_PORT			1900
#define UDPTimeout			5

//Yolo Python
#define YOLO_FILE_PATH		"/home/corelab-laptop2/opencv-workspace/adam_test1"
#define YOLO_FILE_NAME		"RunYolo"
#define YOLO_FUNC_NAME		"runYoloOnImage"
#define YOLO_IMAGE_NAME		"TestImage.jpg"

//Camera
#define CAMERA_WIDTH        640
#define CAMERA_HEIGHT       480
#define OPTICAL_FLOW_SCALE  0.25
#define MASK_SCALE			0.4
#define CAMERA_INDEX        1
#define WAIT                1


//FILTERING
#define MIN_AREA                 100
#define B_MIN_CONFIDENCE         0.3
#define G_MIN_CONFIDENCE         0.5
#define G_POLY_APPROX_E          0.001
#define GOAL_INNER_CONTOUR_SCALE 0.15

//COLORS
#define COLOR_SPACE COLOR_BGR2HSV

//balloon
#define B_MIN           Scalar(35,0,0)
#define B_MAX           Scalar(98,255,255)
#define B_Correction    Scalar(40,0,3)

//yellow goal
#define GY_MIN          Scalar(15,0,87)
#define GY_MAX          Scalar(76,255,255)
#define GY_Correction   Scalar(20,3,4)

//orange goal
#define GO_MIN          Scalar(0,0,0)
#define GO_MAX          Scalar(18,255,255)
#define GO_Correction   Scalar(0,0,0)


//blue blimp
#define BB_MIN          Scalar(255,255,255)
#define BB_MAX          Scalar(255,255,255)
#define BB_Correction   Scalar(0,0,0)


//red blimp
#define BR_MIN          Scalar(255,255,255)
#define BR_MAX          Scalar(255,255,255)
#define BR_Correction   Scalar(0,0,0)


//EROSION AND DILATION
#define E_ITER 1
#define D_ITER 6
#define E_SIZE 1
#define D_SIZE 1

#define GE_ITER	1
#define GD_ITER 6


//OPTICAL FLOW
#define PYRAMID_SCALE   0.5
#define LEVELS          6
#define WIN_SIZE        40
#define ITERATIONS      2
#define POLY_N          5
#define POLY_SIGMA      1.1

//GAUSIAN BLUR
#define KERNEL_SIZE     9

//BALLOON CONFIDENCE
#define MIN_GAP_DISTANCE 10


using namespace std;
using namespace cv;
using namespace chrono;

enum object {
    balloon,
    blimpB,
    blimpR,
    goalO,
    goalY,
    first = balloon,
    last = goalY
};

enum autoState{
	searching,
	approach,
	catching,
	caught,
	goalSearch,
	approachGoal,
	scoring,
	scored
};


//==================== FUNCTION HEADERS ====================
VideoCapture openCamera(int camIndex, int camWidth, int camHeight);
bool parseArguments(int argc, char** argv);
vector<Point> scaleContour(vector<Point> contour, float scale);
vector<vector<float> > getObjects(int type, Mat mask);
//bool initPython();
//string runYolo(Mat& frame);


//==================== COMMUNICATION FUNCTION HEADERS ====================
void initSerial();
void sendSerial(string message);
char readSerial();
void initUDPReceiver();
void initUDPSender();
bool readUDP(string* retMessage = nullptr, string* target = nullptr, string* source = nullptr, string* flag = nullptr);
void sendUDPRaw(string target, string source, string flag, string message);
void sendUDP(string message);
void establishBlimpID();


//==================== HELPER FUNCTION HEADERS ====================
float getFPS();
void benchmarkFirst(string flag);
void benchmark(string flag);
void benchmarkPrint();
void delay(double delaySeconds);


//==================== GLOBAL VARIABLES ====================
int blimpID = -1;
int teensyState;
bool autonomous = false;

bool scoreInOrange = false;
bool selfIsBlue = false;

bool recording = false;
int framesLeftToRecord = 0;

bool debugMode = false;

//==================== MAIN ====================

int main(int argc, char** argv) {
	VideoCapture cap = openCamera(0, CAMERA_WIDTH, CAMERA_HEIGHT);

	VideoWriter outputVideo;

	bool successfulArgs = parseArguments(argc, argv);
	if(!successfulArgs) return 0;

	//bool successfulPythonInit = initPython();
	//if(!successfulPythonInit) return 0;

	if(!debugMode){
		initSerial();
		sendSerial("Hello World");

		//clear serial buffer
		while (true) {
			char c = readSerial();
			if (c == 0) {
				break;
			}
		}

		//handshake
		while (true) {
			char c = readSerial();
			//cout << c << endl;
			if (c == '?') {
				cout << "Teensy handshake recieved" << endl;
				sendSerial("#");
				break;
			}
			cout << "Waiting for Teensy" << endl;
		}

		cout << "Handshake complete" << endl;
	}

	initUDPReceiver();
	initUDPSender();
	establishBlimpID();

	Mat img;
	Mat imgBlur;
	Mat imgTarget;
	Mat imgHSV;
	Mat imgMask;
	Mat imgContours;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

    vector<float> recentMotorCommands;

    clock_t lastCycle = 0;
    clock_t lastHeartbeat = 0;
    clock_t lastSerial = 0;
    clock_t lastUDPReceived = 0;

    //image processing--------------------------------------------------------------------------------
    //init optical flow
    Mat frame1, prvs, small1;
    cap >> frame1;

	// let's downscale the image using new  width and height
	int down_width = CAMERA_WIDTH*OPTICAL_FLOW_SCALE;
	int down_height = CAMERA_HEIGHT*OPTICAL_FLOW_SCALE;
	//resize down
	resize(frame1, small1, Size(down_width, down_height), INTER_LINEAR);

	cvtColor(small1, prvs, COLOR_BGR2GRAY);

	//for serial in case teensy restarts
	clock_t lastMsgTime = clock();

	while(true) {
		clock_t currentTime = clock();
		double cycleTime = double(currentTime - lastCycle)/CLOCKS_PER_SEC;
		lastCycle = currentTime;
		//cout << "Cycle Time (s): " << cycleTime << endl;
		benchmarkFirst("Heart");


		//Send heartbeat
		if(double(currentTime - lastHeartbeat)/CLOCKS_PER_SEC > 0.1){
			lastHeartbeat = currentTime;
			sendUDP("H");
		}
		benchmark("UDP");
		//Receive UDP messages
		bool reading = true;
		while(reading){
			string readIn;
			string target;
			string flag;
			bool success = readUDP(&readIn, &target, nullptr, &flag);
			if(!success){
				reading = false;
			}else if(stoi(target) == blimpID){
				//cout << "Received: \"" << readIn << "\"" << endl;
				if(flag == "A"){
					autonomous = true;
				}else if(flag == "M"){
					char first = readIn.at(0);
					if(isdigit(first)){
						autonomous = false;

						int equalIndex = readIn.find("=");
						int numMotors = stoi(readIn.substr(0,equalIndex));
						int startIndex = readIn.find("=");
						for(int i=0; i<numMotors; i++){
							int endIndex = readIn.find(",",startIndex+1);
							string motorString = readIn.substr(startIndex+1,endIndex-startIndex-1);
							float motorData = stof(motorString);
							startIndex = endIndex;

							if(recentMotorCommands.size() <= i) recentMotorCommands.push_back(0);
							recentMotorCommands[i] = motorData;
						}
					}
				}else if(flag == "P"){
					char first = readIn.at(0);
					if(first == 'C'){
						string recordTimeStr = readIn.substr(1);
						int recordTime = stoi(recordTimeStr);
						int roughFPS = 30;
						framesLeftToRecord = roughFPS*recordTime;
						cout << "Start recording for " << recordTime << " seconds = " << framesLeftToRecord << " frames" << endl;
						outputVideo.open("recording.avi", cv::VideoWriter::fourcc('M','J','P','G'), roughFPS, Size(CAMERA_WIDTH,CAMERA_HEIGHT), true);
						recording = true;
					}
				}else if(flag == "K"){
					cout << "Received kill command. Killing..." << endl;
					return 0;
				}
				lastUDPReceived = clock();
				//cout << "UDPReceived: \"" << readIn << "\" sent to \"" << target << "\"" << endl;
			}
		}


		benchmark("GetFrame");
		//image processing-----------------------------------------------------------------------------------------------------
		Mat LAB, mask, frame1, gray;
		Mat balloonMask, goalOMask, goalYMask, blimpBMask, blimpRMask;



		//get frame
		Mat frame2, frame3, next, small2;
		cap >> frame2;

		vector<vector<float> > balloons;
		vector<vector<float> > goalYs;
		vector<vector<float> > goalOs;
		vector<vector<float> > blimpRs;
		vector<vector<float> > blimpBs;

		if (frame2.empty())
			break;
		if(framesLeftToRecord > 0){
			framesLeftToRecord--;
			outputVideo << frame2;
			cout << "Recording... " << framesLeftToRecord << " frames left." << endl;
			waitKey(1);
		}else if(recording){
			//Finish recording
			recording = false;
		}else{
			//outputVideo.write(frame2);

			benchmark("Resize");
			// let's downscale the image using new  width and height
			int down_width = CAMERA_WIDTH*OPTICAL_FLOW_SCALE;
			int down_height = CAMERA_HEIGHT*OPTICAL_FLOW_SCALE;
			//resize down
			resize(frame2, small2, Size(down_width, down_height), INTER_LINEAR);

			benchmark("Blur");
			GaussianBlur(frame2, frame2, Size(KERNEL_SIZE, KERNEL_SIZE), 0);

			benchmark("Color Correction1");
			Mat balloonCorrected;
			Mat balloonCorrectM = Mat::zeros(frame2.size(), frame2.type());
			balloonCorrectM.setTo(B_Correction);
			add(frame2,balloonCorrectM, balloonCorrected);

			benchmark("Color Correction2");
			Mat goalYCorrected;
			Mat goalYCorrectM = Mat::zeros(frame2.size(), frame2.type());
			goalYCorrectM.setTo(GY_Correction);
			add(frame2,goalYCorrectM, goalYCorrected);

			benchmark("Color Correction3");
			Mat goalOCorrected;
			Mat goalOCorrectM = Mat::zeros(frame2.size(), frame2.type());
			goalOCorrectM.setTo(GO_Correction);
			add(frame2,goalOCorrectM, goalOCorrected);
			/*
			 Mat BBCorrected;
			Mat BBCorrectM = Mat::zeros(frame2.size(), frame2.type());
			BBCorrectM.setTo(BB_Correction);
			add(frame2,BBCorrectM, BBCorrected);

			 Mat BRCorrected;
			Mat BRCorrectM = Mat::zeros(frame2.size(), frame2.type());
			BRCorrectM.setTo(BR_Correction);
			add(frame2,goalYCorrectM, BRCorrected);
			*/

			benchmark("Conversion and Threshold");
			//get labs and greyscale for detection
			Mat B_HSV, GY_HSV, GO_HSV, BB_HSV, BR_HSV;
			cvtColor(small2, next, COLOR_BGR2GRAY);
			cvtColor(balloonCorrected, B_HSV, COLOR_SPACE);
			cvtColor(goalYCorrected, GY_HSV, COLOR_SPACE);
			cvtColor(goalOCorrected, GO_HSV, COLOR_SPACE);
			//cvtColor(BBCorrected, BB_HSV, COLOR_SPACE);
			//cvtColor(BRCorrected, BR_HSV, COLOR_SPACE);

			///fixed thresholding
			cv::inRange(B_HSV, B_MIN, B_MAX, balloonMask);
			cv::inRange(GO_HSV, GO_MIN, GO_MAX, goalOMask);
			cv::inRange(GY_HSV, GY_MIN, GY_MAX, goalYMask);
			//cv::inRange(BB_HSV, BB_MIN, BB_MAX, blimpBMask);
			//cv::inRange(BR_HSV, BR_MIN, BR_MAX, blimpRMask);

			benchmark("Morphology");
			//imshow("Mask", mask);
			Mat erosionElem = getStructuringElement(MORPH_ELLIPSE, Size(2*E_SIZE+1,2*E_SIZE+1),Point(E_SIZE, E_SIZE));
			Mat dilationElem = getStructuringElement(MORPH_ELLIPSE, Size(2*D_SIZE+1,2*D_SIZE+1),Point(D_SIZE, D_SIZE));

			erode(balloonMask, balloonMask, erosionElem, Point(-1,-1), E_ITER);
			dilate(balloonMask, balloonMask, dilationElem, Point(-1,-1), D_ITER);

			erode(goalYMask, goalYMask, erosionElem, Point(-1,-1), GE_ITER);
			dilate(goalYMask, goalYMask, dilationElem, Point(-1,-1), GD_ITER);

			erode(goalOMask, goalOMask, erosionElem, Point(-1,-1), GE_ITER);
			dilate(goalOMask, goalOMask, dilationElem, Point(-1,-1), GD_ITER);

			/*
			erode(blimpBMask, blimpBMask, erosionElem, Point(-1,-1), E_ITER);
			dilate(blimpBMask, blimpBMask, dilationElem, Point(-1,-1), D_ITER);

			erode(blimpRMask, blimpRMask, erosionElem, Point(-1,-1), E_ITER);
			dilate(blimpRMask, blimpRMask, dilationElem, Point(-1,-1), D_ITER);
			*/

			benchmark("Resize");
			Mat balloonMasks, goalYMasks, goalOMasks;
			resize(balloonMask, balloonMasks, Size(CAMERA_WIDTH*MASK_SCALE, CAMERA_HEIGHT*MASK_SCALE), INTER_LINEAR);
			resize(goalYMask, goalYMasks, Size(CAMERA_WIDTH*MASK_SCALE, CAMERA_HEIGHT*MASK_SCALE), INTER_LINEAR);
			resize(goalOMask, goalOMasks, Size(CAMERA_WIDTH*MASK_SCALE, CAMERA_HEIGHT*MASK_SCALE), INTER_LINEAR);


			benchmark("Objects");
			//get data on object

			benchmark("Balloons");
			balloons = getObjects(balloon, balloonMasks);
			benchmark("Goals");
			goalYs = getObjects(goalY, goalYMasks);
			benchmark("OrangeGoals");
			//Around here
			goalOs = getObjects(goalO, goalOMasks);
			//blimpRs = getObjects(blimpR, blimpRMask, flow);
			//blimpBs = getObjects(blimpB, blimpBMask, flow);

			if (goalOs.size() > 0 && goalYs.size() > 0) {
				goalYs.clear();
			}


			benchmark("Output");
			int numBalloons = balloons.size();
			int numYGoals = goalYs.size();
			int numOGoals = goalOs.size();
			cout << "balloons: " << "\t" << numBalloons << endl;
			cout << "yellow goals: " << "\t" << numYGoals << endl;
			cout << "orange goals: " << "\t" << numOGoals << endl << endl;
		}

		//waitKey(1);


		//Print serial data-----------------------------------------------------------------------------------------------------
		if(double(currentTime - lastSerial)/CLOCKS_PER_SEC > 0.1){
			lastSerial = currentTime;

			string message;
			if(autonomous){
				//A,firstBalloon,secondBalloon,etc...,
				//Each balloon: centerX;centerY;radius;area;confidence
				message = "A&";
				message += "CV&";
				for(int i=0; i<5; i++){
					switch (i) {
					case 0:
						for (unsigned int j= 0; j < balloons.size(); j++) {
							message += "B:";
							for (unsigned int k = 0; k < 7; k++) {
								message += to_string(balloons[j][k]);
								message += ":";
							}
							message += "&";
						}
						break;
					case 1:
						for (unsigned int j= 0; j < goalYs.size(); j++) {
							if (scoreInOrange) {
								message += "GD:";
							} else {
								message += "GA:";
							}
							//score in orange/ defend yellow
							for (unsigned int k = 0; k < 7; k++) {
								message += to_string(goalYs[j][k]);
								message += ":";
							}
							message += "&";
						}
						break;
					case 2:
						for (unsigned int j= 0; j < goalOs.size(); j++) {
							if (!scoreInOrange) {
								message += "GD:";
							} else {
								message += "GA:";
							}
							//score in yellow / defend orange
							for (unsigned int k = 0; k < 7; k++) {
								message += to_string(goalOs[j][k]);
								message += ":";
							}
							message += "&";
						}
						break;
					case 3:
						for (unsigned int j= 0; j < blimpBs.size(); j++) {
							if (selfIsBlue) {
								message += "BF:";
							} else {
								message += "BE:";
							}
							//team blue
							for (unsigned int k = 0; k < 7; k++) {
								message += to_string(blimpBs[j][k]);
								message += ":";
							}
							message += "&";
						}
						break;
					case 4:
						for (unsigned int j= 0; j < blimpRs.size(); j++) {
							if (!selfIsBlue) {
								message += "BF:";
							} else {
								message += "BE:";
							}
							//team red
							for (unsigned int k = 0; k < 7; k++) {
								message += to_string(blimpRs[j][k]);
								message += ":";
							}
							message += "&";
						}
						break;
					default:
						break;
					}

				}
				message += "\n";
			}else{
				message = "M";
				message += "&CV";
				for(int i=0; i<recentMotorCommands.size(); i++){
					message += "&" + to_string(recentMotorCommands[i]).substr(0,4);
					if(i == recentMotorCommands.size()-1){
						message += "&";
					}
				}
				message += "\n";
			}
			//sendSerial(message);
			cout << message;
			benchmark("Output1");

			if((clock()-lastUDPReceived)/CLOCKS_PER_SEC > UDPTimeout){
				//UDP Timed-out
				message = "L&\n";
			}

			benchmark("Output2");

			if(debugMode) continue;

			//cout << "Sending \"" << message << endl;

			vector<String> splitMessage;
			//split final message into an array of strings to be sent
			for (unsigned int i = 0; i < message.length(); i = i + 8) {
				//get next eight characters in message
				splitMessage.push_back(message.substr(i, 8));
			}

			benchmark("Output3");

			/*
			for (unsigned int i = 0; i < splitMessage.size(); i++) {
				cout << splitMessage[i] << endl;
			}
			cout << endl;
			*/

			//send first string of data to teensy and wait for response
			if (splitMessage.size() > 0) {
				sendSerial(splitMessage[0]);
				sendSerial("@");
				//wait for response from teensy
			}


			benchmark("Output4");

			String returnedMessage = "";
			bool canEndSend = false;
			int index = 1;
			while (!canEndSend) {
				while (true) {

					char c = readSerial();

					if (c == '?') {
						//teensy waiting for handshake
						canEndSend = true;
						break;
					}

					if (c == '@' || c == '*') {
						lastMsgTime = clock();
						//send next part of message
						if (index < splitMessage.size()) {
							sendSerial(splitMessage[index]);
							sendSerial("@");
							index += 1;
						} else {
							if (c == '@') {
								sendSerial("*");
							} else {
								sendSerial("#");
								canEndSend = true;
							}
						}
						break;
					} else if (c != 0) {
						returnedMessage = returnedMessage + c;
					}

					if ((clock() - lastMsgTime)/CLOCKS_PER_SEC > 1) {
						canEndSend = true;
						break;
					}
				}
			}

			cout << returnedMessage << endl;


			benchmark("Output5");

			if ((clock()-lastMsgTime)/CLOCKS_PER_SEC > 1) {
				//redo handshake
				while (true) {
					char c = readSerial();
					//cout << c << endl;
					if (c == '?') {
						cout << "Teensy handshake recieved" << endl;
						sendSerial("#");
						lastMsgTime = clock();
						//clear serial buffer
						while (true) {
							char c = readSerial();
							if (c == 0) {
								break;
							}
						}
						break;
					}
					cout << "Waiting for Teensy" << endl;
				}

				cout << "Handshake complete" << endl;
			}

			//benchmarkPrint();
		}

		benchmark("Last");
		//benchmarkPrint();
	}

	cout << "DONE" << endl;

	return 0;
}


//==================== FUNCTION HEADERS ====================
VideoCapture openCamera(int camIndex, int camWidth, int camHeight){
	string video_str = "/dev/video2";
	video_str.at(video_str.length()-1) = to_string(camIndex).at(0);
	int device;
	device = v4l2_open(video_str.c_str(), O_RDWR | O_NONBLOCK);
	struct v4l2_format fmt;
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	int forcedWidth = camWidth;
	int forcedHeight = camHeight;
	int forcedFormat = V4L2_PIX_FMT_MJPEG;
	int forcedField = V4L2_FIELD_ANY;

	fmt.fmt.pix.width = forcedWidth;
	fmt.fmt.pix.height = forcedHeight;
	fmt.fmt.pix.pixelformat = forcedFormat;
	fmt.fmt.pix.field = forcedField;
	v4l2_ioctl(device, VIDIOC_S_FMT, &fmt);

	struct v4l2_control c_exposure_auto;
	c_exposure_auto.id = V4L2_CID_EXPOSURE_AUTO;
	c_exposure_auto.value = 1; // 1=manual, 3=auto; V4L2_EXPOSURE_AUTO fails...
	if (v4l2_ioctl(device, VIDIOC_S_CTRL, &c_exposure_auto) != 0) {
		cout << "Failed to set... " << strerror(errno) << endl;
	}

	struct v4l2_control c_exposure_auto_priority;
	c_exposure_auto_priority.id = V4L2_CID_EXPOSURE_AUTO_PRIORITY;
	c_exposure_auto_priority.value = 0;
	if (v4l2_ioctl(device, VIDIOC_S_CTRL, &c_exposure_auto_priority) != 0) {
		cout << "Failed to set... " << strerror(errno) << endl;
	}

	struct v4l2_control c_exposure_auto_priority_absolute;
	c_exposure_auto_priority_absolute.id = V4L2_CID_EXPOSURE_ABSOLUTE;
	c_exposure_auto_priority_absolute.value = 47;
	if (v4l2_ioctl(device, VIDIOC_S_CTRL, &c_exposure_auto_priority_absolute) != 0) {
		cout << "Failed to set... " << strerror(errno) << endl;
	}

	if(false){
		struct v4l2_control c_whitebalance_auto;
		c_whitebalance_auto.id = V4L2_CID_AUTO_WHITE_BALANCE;
		c_whitebalance_auto.value = 0;
		v4l2_ioctl(device, VIDIOC_S_CTRL, &c_whitebalance_auto);

		struct v4l2_control c_whitebalance_temp;
		c_whitebalance_temp.id = V4L2_CID_WHITE_BALANCE_TEMPERATURE;
		c_whitebalance_temp.value = 3750;
		v4l2_ioctl(device, VIDIOC_S_CTRL, &c_whitebalance_temp);

	}else{
		struct v4l2_control c_whitebalance_auto;
		c_whitebalance_auto.id = V4L2_CID_AUTO_WHITE_BALANCE;
		c_whitebalance_auto.value = 1;
		v4l2_ioctl(device, VIDIOC_S_CTRL, &c_whitebalance_auto);
	}
	v4l2_close(device);

	//string src = "v4l2-ctl -d 2 --set-fmt-video=width=1920,height=1080,pixelformat=MJPG";
	VideoCapture cap(camIndex, CAP_V4L);
	if (!cap.isOpened()) {
		CV_Assert("Cam open failed");
	}

	//640x480x120fps
	//1280x720x60fps
	//1920x1080x30fps
	//4000x3000x10fps

	int fourcc2 = VideoWriter::fourcc('M', 'J', 'P', 'G');
	cap.set(CAP_PROP_FOURCC, fourcc2);
	cap.set(CAP_PROP_BUFFERSIZE, 3);

	//cap.set(CAP_PROP_FPS, 120);
	cap.set(CAP_PROP_FRAME_WIDTH, camWidth);
	cap.set(CAP_PROP_FRAME_HEIGHT, camHeight);
	return cap;
}

//Returns true on success, false on failure
bool parseArguments(int argc, char** argv){
	//Parse arguments for blimp/goal type
	cout << argc << endl;

	if (argc < 3 || argc > 3) {
		cout << "Error: Incorrect argument size" << endl;
		return false;
	}

	if (argv[1][0] == 'b') {
		cout << "Blimp is blue" << endl;
		selfIsBlue = true;
	} else if (argv[1][0] == 'r') {
		cout << "Blimp is red" << endl;
		selfIsBlue = false;
	} else {
		cout << "Invalid Argument, Exiting Program" << endl;
		return false;
	}

	if (argv[2][0] == 'o') {
		cout << "Blimp should score in orange goals" << endl;
		scoreInOrange = true;
	} else if (argv[2][0] == 'y') {
		cout << "Blimp should score in yellow goals" << endl;
		scoreInOrange = false;
	} else {
		cout << "Invalid Argument, Exiting Program" << endl;
		return false;
	}
	return true;
}

vector<Point> scaleContour(vector<Point> contour, float scale) {
    Moments moment = moments(contour);
    float cx = moment.m10 / moment.m00; //int(M['m10']/M['m00'])
    float cy = moment.m01 / moment.m00; //int(M['m01']/M['m00'])

    //shift all points
    for (unsigned int i = 0; i < contour.size(); i++) {
        contour[i].x = ((contour[i].x-cx)*scale)+cx;
        contour[i].y = ((contour[i].y-cy)*scale)+cy;
    }

    return contour;

}

vector<vector<float> > getObjects(int type, Mat mask) {

    vector<float> object;
    vector<vector<float> > set;

    //check if inputs are in the correct range
    if (!mask.empty()) {
        //get object data

        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;

        findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        Mat debug = Mat::zeros(mask.rows, mask.cols, CV_8UC3);

        // iterate through all the top-level contours,
        // draw each connected component with its own random color
        for (unsigned int i = 0; i < contours.size(); i++) {
            //perfom initial area filter
            float area = contourArea(contours[i]);
            if (hierarchy[i][3] == -1 && (area > MIN_AREA || type == goalO || type == goalY)) {

                //<-------------Optical Flow----------------->

                //temp matrix for drawing
                Mat temp = Mat::zeros(mask.rows, mask.cols, CV_8UC3);
                Scalar color(255, 255, 255);
                Mat tempSingle;

                unsigned int indexOfInner = i;

                vector<Point> approx;
                vector<vector<Point> > outApprox;
                vector<vector<Point> > outScaled;

                if (type == goalO || type == goalY) {
                    //do approx poly to close contours and smooth them
                    float epsilon = 0.0001*cv::arcLength(contours[i],true);
                    cv::approxPolyDP(contours[i], approx, epsilon,true);
                    outApprox.push_back(approx);
                    //scale contour
                    outScaled.push_back(scaleContour(approx, 0.7));


                    //cout << "goal found" << endl;
                    //get goal mask
                    //get confidence interval
                    Mat innerSingle, goalSingle;
                    Mat innerSingleBit, goalSingleBit;
                    Mat inner = Mat::zeros(mask.rows, mask.cols, CV_8UC3);
                    Mat goal = Mat::zeros(mask.rows, mask.cols, CV_8UC3);

                    drawContours(inner, outScaled, 0, color, FILLED, 8);
                    drawContours(goal, contours, i, color, FILLED, 8, hierarchy);

                    cvtColor(inner, innerSingle, COLOR_BGR2GRAY);
                    cvtColor(goal, goalSingle, COLOR_BGR2GRAY);

                    //get just the goal detected
                    bitwise_xor(goalSingle, goalSingle, goalSingle, innerSingle);

                    //set as optical flow mask
                    tempSingle = goalSingle;
                    //bitwise_or(debug, tempSingle, debug);


                } else {
                    drawContours(temp, contours, i, color, FILLED, 8, hierarchy);
                    //get grayscale of specified filled coutour for optical flow mask
                    cvtColor(temp, tempSingle, COLOR_BGR2GRAY);
                }


                //<------------Confidence---------------->
                float confidence = 0;

                if (type == goalO || type == goalY) {
                    //get confidence interval of goal
                    Mat innerSingle;
                    Mat innerSingleBit, goalSingleBit;
                    Mat inner = Mat::zeros(mask.rows, mask.cols, CV_8UC3);

                    drawContours(inner, outScaled, 0, color, FILLED, 8);

                    cvtColor(inner, innerSingle, COLOR_BGR2GRAY);


                    float areaI = (float)countNonZero(innerSingle);
                    float areaG = (float)countNonZero(tempSingle);

                    //get the pixels detected in each region
                    bitwise_and(innerSingle, mask, innerSingleBit);
                    bitwise_and(tempSingle, mask, goalSingleBit);

                    float innerDetect = ((float)countNonZero(innerSingleBit))/areaI;
                    float goalDetect = ((float)countNonZero(goalSingleBit))/areaG;

                    //cout << "inner total" << areaI << endl;
                    //cout << "inner detected" << innerDetect << endl;

                    confidence = goalDetect*(1-innerDetect);

                    //cout << "condfidence:" << confidence << endl;
                } else if (type == balloon) {
                    //get confidence of balloon (from adams code)
                    Point2f center;
					float radius;
					minEnclosingCircle(contours[i],center,radius);

					//Establish confidence
					Mat innerCircleMask = Mat::zeros(mask.size(),CV_8U);
					Mat gapRingMask = Mat::zeros(mask.size(),CV_8U);

					circle(innerCircleMask, center, radius, 255, -1);
					circle(gapRingMask, center, radius + MIN_GAP_DISTANCE, 255, -1);
					subtract(gapRingMask, innerCircleMask, gapRingMask);

					float innerCircleArea = 3.1415*pow(radius,2);
					float gapRingArea = 3.1415*(pow(radius+MIN_GAP_DISTANCE,2)-pow(radius,2));

					Mat positiveDetection;
					Mat negativeDetection;
					bitwise_and(innerCircleMask, mask, positiveDetection);
					bitwise_and(gapRingMask, mask, negativeDetection);

					float innerCirclePixels = countNonZero(positiveDetection);
					float gapRingPixels = countNonZero(negativeDetection);

					float good = innerCirclePixels/innerCircleArea;
					float bad = gapRingPixels/gapRingArea;

					confidence = good * (1-bad);
                } else {
                    //is blimp
                    confidence = 1;
                }

                //<------------x, y coordinates---------------->
                Point2f center;
			    float radius;
				minEnclosingCircle(contours[i],center,radius);

                float x = (float)center.x;
                float y = (float)center.y;


                double confidenceCheck = 0;

				if (type == goalO || type == goalY) {
					confidenceCheck = G_MIN_CONFIDENCE;
				} else if (type == balloon) {
					confidenceCheck = B_MIN_CONFIDENCE;
				}

				//add parameters to object and push to output
				if (confidence >= confidenceCheck) {
					//cout << confidence << endl;
					object.push_back(x/MASK_SCALE);
					object.push_back(y/MASK_SCALE);
					object.push_back((double)radius/MASK_SCALE);
					object.push_back(area/(MASK_SCALE*MASK_SCALE));
					object.push_back(confidence);
					set.push_back(object);
					object.clear();
				}
            }
        }
        //imshow("Mask of goals", mask);
    }

    return set;
}
/*
PyObject *pYoloModuleName, *pYoloModule, *pYoloFunc, *pYoloArg, *pYoloArgVal, *pYoloReturn;
string pythonFilePath = YOLO_FILE_PATH;
string yoloFileName = YOLO_FILE_NAME;
string yoloFuncName = YOLO_FUNC_NAME;
string yoloImageName = YOLO_IMAGE_NAME;

bool initPython(){
	//Load python
	Py_Initialize();
	PyRun_SimpleString("import sys");
	string commandAddPath = "sys.path.append(\"" + pythonFilePath + "\")";
	PyRun_SimpleString(commandAddPath.c_str());

	pYoloModuleName = PyUnicode_FromString(yoloFileName.c_str());
	pYoloModule = PyImport_Import(pYoloModuleName);
	if(pYoloModule == NULL){
		cout << "Could not find yolo module :(" << endl;
		return false;
	}

	pYoloFunc = PyObject_GetAttrString(pYoloModule, yoloFuncName.c_str());
	if(pYoloFunc == NULL){
		cout << "Could not find yolo function :(" << endl;
		return false;
	}

	pYoloArg = PyTuple_New(1);
	pYoloArgVal = PyBytes_FromString(yoloImageName.c_str());
	PyTuple_SetItem(pYoloArg,0,pYoloArgVal);
	return true;
}

string runYolo(Mat& frame){
	//cout << "Saving image." << endl;
	imwrite(yoloImageName,frame);
	time_point<system_clock> startTP = system_clock::now();
	pYoloReturn = PyObject_CallObject(pYoloFunc,pYoloArg);
	if(pYoloReturn == NULL){
		cout << "Yolo returned null :(" << endl;
		if (PyErr_Occurred()) {
			cout << "Python Error Exception:" << endl;
			PyErr_PrintEx(0);
			PyErr_Clear();
		}
		return "";
	}

	string returned = PyBytes_AsString(pYoloReturn); //CAN THROW LOGIC ERROR; have to return bytes in python (if type is str, just call str.encode(<your str object here>))
	time_point<system_clock> endTP = system_clock::now();
	duration<float> deltaTP = endTP-startTP;
	float deltaTime = deltaTP.count();
	cout << "Elapsed Yolo Time: " << deltaTime << " sec" << endl;
	return returned;
}
*/

//==================== COMMUNICATION FUNCTION HEADERS ====================
int serial;

void initSerial(){
	while ((serial = serialOpen ("/dev/ttyS0", 115200)) < 0) {
		cout << "Unable to open serial port." << endl;
		delay(1.0);
	}
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

void sendUDP(string message){
	sendUDPRaw("0",to_string(blimpID),"D",message);
}

void establishBlimpID(){
	cout << "Waiting for Blimp ID..." << endl;
	while(blimpID == -1){
		sendUDPRaw("0","N","N","N");
		delay(0.25);
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
					blimpID = stoi(message);
				}
			}
		}
	}
	cout << "Received blimp ID: " << blimpID << endl;
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
	cout << "Total: " << deltaTotal << endl;
}

void delay(double delaySeconds){
	clock_t start = clock();
	while(double(clock() - start)/CLOCKS_PER_SEC < delaySeconds);
}
