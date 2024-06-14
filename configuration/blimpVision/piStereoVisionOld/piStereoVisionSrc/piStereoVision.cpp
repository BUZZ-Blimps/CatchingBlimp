//C++ includes
//#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <iostream>
#include <list>
#include <unordered_map>
#include <opencv2/opencv.hpp>
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

//#include <wiringPi.h>
//#include <wiringSerial.h>

#include "serialib.h"

#include "Teleplot.h"



//==================== CONSTANTS ====================
//Communication
#define UDP_IP				"239.255.255.250"
#define UDP_PORT			1900
#define UDPTimeout			5


//Camera
#define CAMERA_ID		0
#define CAMERA_WIDTH	320
#define CAMERA_HEIGHT	240

#define CV_WIDTH		320
#define CV_HEIGHT		240

#define RECT_WIDTH		320
#define RECT_HEIGHT		240

//Stereo
#define DISP_WIDTH		160
#define DISP_HEIGHT		120

#define CONVERSION		0.15

#define PRE_FILTER_SIZE	7
#define PRE_FILTER_CAP	2
#define UNIQUENESS_RATIO	5


#define LAMBDA			17.8
#define SIGMA			5.0


//Colors
#define B_CORRECTION	Scalar(29,7,15)
#define B_MIN			Scalar(46,0,0)
#define B_MAX			Scalar(96,74,213)

#define ORANGE_G_CORRECTION    Scalar(0,47,0)
#define ORANGE_G_MIN           Scalar(13,0,0)
#define ORANGE_G_MAX           Scalar(24,255,255)

#define YELLOW_G_CORRECTION	    Scalar(48,0,0)
#define YELLOW_G_MIN			Scalar(25,0,0)
#define YELLOW_G_MAX			Scalar(63, 255, 255)

#define G_POLY_APPROX_E          0.01
#define GOAL_INNER_CONTOUR_SCALE 0.7
#define GOAL_CONFIRM			 6000

#define E_ITER 1
#define D_ITER 1
#define E_SIZE 1
#define D_SIZE 1

#define GOAL_DILATION_ADJUST 4

#define MIN_AREA		50
#define SIZE_RATIO		3

#define AVOID_DIST		70
#define AVOID_AREA		6000


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
	scoringStart,
	shooting,
	scored
};

enum blimpType{
	blue,
	red
};

enum goalType{
	orange,
	yellow
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
void sendUDP(string flag, string message);
void sendUDP(string message);
void establishBlimpID();
void plotUDP(string varName, float varValue);


//==================== HELPER FUNCTION HEADERS ====================
float getFPS();
void benchmarkFirst(string flag);
void benchmark(string flag);
void benchmarkPrint();
void delay(double delaySeconds);
void saveToVideo(Mat frame);

//==================== GLOBAL VARIABLES ====================
int blimpID = -1;
int teensyState;
bool autonomous = false;

bool scoreInOrange = false;
bool selfIsBlue = false;

int blimpColor = blue;
int goalColor = orange;

int framesLeftToRecord;

bool debugMode = false;

String msgTemp = "";
int mode = searching;

float barometerData = 0;

string outputVideo_fileName = "outputVideo.avi";
double outputVideo_fps = 30;

float lastBaroMessageTime = 0.0;

Teleplot teleplot("127.0.0.1");

serialib serial;

//==================== MAIN ====================

int main(int argc, char** argv) {

	for(int i=0; i<argc; i++) cout << "Arg[" << i << "] = \"" << argv[i] << "\"" << endl;

	if (argc != 2) {
		fprintf(stderr, "Error: ID not provided.\n");
		return -1;
	}

	blimpID = atoi(argv[1]);
	cout << "I. Am. Blimp. " << blimpID << "." << endl;

	clock_t currentTime = clock();

	initSerial();

	VideoCapture cap = openCamera(CAMERA_ID, CAMERA_WIDTH, CAMERA_HEIGHT);

	framesLeftToRecord = 9000;
	int moreFramesPerTrigger = 30 * 10;

	//bool successfulPythonInit = initPython();
	//if(!successfulPythonInit) return 0;

	initUDPReceiver();
	initUDPSender();
	//establishBlimpID();

    vector<float> recentMotorCommands;

    clock_t lastCycle = 0;
    clock_t lastHeartbeat = 0;
    clock_t lastSerial = 0;
    clock_t lastUDPReceived = 0;


	clock_t last = clock();

	//image processing--------------------------------------------------------------------------------


	//for serial in case teensy restarts
	clock_t lastMsgTime = clock();

	//read stereo calibration file
	Mat Left_Stereo_Map1, Left_Stereo_Map2;
	Mat Right_Stereo_Map1, Right_Stereo_Map2;
	Mat Q;

	cout << "Read Stereo Camera Parameters" << endl;
	FileStorage cv_file2 = FileStorage("/home/pi/piStereoVisionSrc/stereo_rectify_maps240p.xml",FileStorage::READ);
	cv_file2["Left_Stereo_Map_x"] >> Left_Stereo_Map1;
	cv_file2["Left_Stereo_Map_y"] >> Left_Stereo_Map2;
	cv_file2["Right_Stereo_Map_x"] >> Right_Stereo_Map1;
	cv_file2["Right_Stereo_Map_y"] >> Right_Stereo_Map2;
	cv_file2["Q"] >> Q;
	cv_file2.release();
	cout << "Read Complete" << endl;

	Ptr< StereoBM > left_matcher = StereoBM::create(16, 13); //Num disp, block size
	left_matcher->setPreFilterType(1);
	left_matcher->setPreFilterSize(PRE_FILTER_SIZE);
	left_matcher->setPreFilterCap(PRE_FILTER_CAP);
	left_matcher->setUniquenessRatio(UNIQUENESS_RATIO);

	Ptr<ximgproc::DisparityWLSFilter> wls_filter = ximgproc::createDisparityWLSFilter(left_matcher);
	Ptr<StereoMatcher> right_matcher = ximgproc::createRightMatcher(left_matcher);

	wls_filter->setLambda(LAMBDA);
	wls_filter->setSigmaColor(SIGMA);

	while(true) {
		cout << "Blimp ID: " << blimpID << endl;
		clock_t currentTime = clock();
		double cycleTime = double(currentTime - lastCycle)/CLOCKS_PER_SEC;
		lastCycle = currentTime;
		//cout << "Cycle Time (s): " << cycleTime << endl;
		benchmarkFirst("Heart");


		//Send heartbeat
		if(double(currentTime - lastHeartbeat)/CLOCKS_PER_SEC > 0.1){
			lastHeartbeat = currentTime;
			//sendUDP("H");
			sendUDP("S",to_string(mode));
		}
		//benchmark("UDP");
		//Receive UDP messages
		bool reading = true;
		while(reading){
			string readIn;
			string target;
			string flag;
			bool success = readUDP(&readIn, &target, nullptr, &flag);
			
			if(!success){
				reading = false;
			}else{
				int id = -1;
				try {
					id = stoi(target);
				}
				catch(std::invalid_argument& e) {
					cout << "invalid blimp id: \"" << target << "\", \"" << readIn << "\"" << endl;
				}
				if(id == blimpID){
					//cout << "Received: \"" << readIn << "\"" << endl;
					if(flag == "A"){
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
						catch(std::invalid_argument& e) {
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
						if(targetGoalData == "O"){
							goalColor = orange;
						}else if(targetGoalData == "Y"){
							goalColor = yellow;
						}
						
						//Disregard the rest
						
					}else if(flag == "M"){
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

							if(recentMotorCommands.size() <= i) recentMotorCommands.push_back(0);
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
						if(targetGoalData == "O"){
							goalColor = orange;
						}else if(targetGoalData == "Y"){
							goalColor = yellow;
						}
						
						//cout << "Manual Mode:" << endl;
						//cout << recentMotorCommands[0] << ", " << recentMotorCommands[1] << ", " << recentMotorCommands[2] << ", " << recentMotorCommands[3] << ", " << recentMotorCommands[4] << ", " << recentMotorCommands[5] << endl;
						//cout << "Barometer: " << barometerData << endl;
						//cout << "TargetGoal: " << goalColor << endl;
						
						//Disregard the rest
						
					}else if(flag == "B"){
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
						
					}else if(flag == "P"){
						char first = readIn.at(0);
						if(first == 'C'){
							cout << "Start recording." << endl;
							framesLeftToRecord += moreFramesPerTrigger;
						}
					}else if(flag == "K"){
						cout << "Received kill command. Killing..." << endl;
						return 0;
					}else if(flag == "TG"){
						if(readIn == "O"){
							goalColor = orange;
						}else if(readIn == "Y"){
							goalColor = yellow;
						}
					}
					
					//cout << "UDPReceived: \"" << readIn << "\" sent to \"" << target << "\"" << endl;
				}
			}
			
		}

		//benchmark("GetFrame");
		
		if (clock()/((float)CLOCKS_PER_SEC) - lastBaroMessageTime > 10) {
			barometerData = -10000;
			cout << "Baro data not current" << endl;
		}
		

		//image processing-----------------------------------------------------------------------------------------------------

		//get frame
		Mat frame2;
		cap >> frame2;

		//split frame into two images, L and R
		Mat imgL, imgR;

		Rect left_roi(0, 0, frame2.cols/2, frame2.rows);
		Rect right_roi (frame2.cols/2, 0, frame2.cols/2, frame2.rows);

		Mat crop_left(frame2, left_roi);
		Mat crop_right (frame2, right_roi);

		crop_left.copyTo(imgL);
		crop_right.copyTo(imgR);

		//send left image to base
		//TO BE COMPLETED

		//reduce image size for rectification
		Mat imgL_rect, imgR_rect;
		resize(imgL, imgL_rect, Size(RECT_WIDTH, RECT_HEIGHT), INTER_LINEAR);
		resize(imgR, imgR_rect, Size(RECT_WIDTH, RECT_HEIGHT), INTER_LINEAR);

		//remap images for stereo
		Mat left_correct, right_correct;

		benchmark("Remap");
		remap(imgL_rect,
				left_correct,
				Left_Stereo_Map1,
				Left_Stereo_Map2,
				INTER_AREA,
				BORDER_CONSTANT,
				0);

		remap(imgR_rect,
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

		//cout << "Area: " << largestArea << endl;
		cout << "Quad: " << quad << endl;
		cout << "Targeting Goal: " << goalColor << endl;
		//plotUDP("Quad",quad);
		
		/*
		//recording frames-----------------------------------------
		Mat tempRecord;
		cvtColor(temp, tempRecord, COLOR_GRAY2BGR);
		//cout << "TempNewType: " << tempRecord.type() << ", NewSize: " << tempRecord.size() << endl;
		cvtColor(bMask, tempRecord, COLOR_GRAY2BGR);
		//cout << "BMaskNewType: " << tempRecord.type() << ", NewSize: " << tempRecord.size() << endl;
		//drawContours(tempRecord, contours, i, Scalar(125), 4, 8, hierarchy);
		//cout << "GoodType: " << imgL_rect.type() << ", GoodSize: " << imgL_rect.size() << endl;
		//cout << "NewType: " << tempRecord.type() << ", NewSize: " << tempRecord.size() << endl;

		
		tempRecord = 0.5*tempRecord + 0.5*left_correct;
		*/
		
		if(framesLeftToRecord > 0){
			framesLeftToRecord--;
			saveToVideo(left_correct);
			cout << "Recording video." << endl;
		}


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
											cout << "condfidence:" << confidence << endl;
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

		/*
		for (unsigned int i = 0; i < balloons.size(); i++) {
			if (balloons[i].size() == 4) {
				cout << "Balloon x: " << balloons[i][0]-320/2 << "\ty: " << balloons[i][1]-240/2 << "\tz:" << balloons[i][2] << endl;
			}
		}

		for (unsigned int i = 0; i < goals.size(); i++) {
			if (goals[i].size() == 4) {
				cout << "Goals x: " << goals[i][0] -320/2 << "\ty: " << goals[i][1] -240/2 << "\tarea: " << goals[i][3] << endl;
			}
		}
		*/

		clock_t now = clock();
		float time = (float)(now-last)/(float)CLOCKS_PER_SEC;
		//cout << "Vision Compute Time: " << time << endl;
		//cout << "Vision Compute Rate: " << 1/time << endl;
		last = now;

		//get data from base and combine with color vision

		//waitKey(1);

		//Select target for blimp depending on state:
		std::vector<std::vector<float> > target;

		//get largest balloon or goal depending on state
		if (mode == searching || mode == approach || mode == catching) {
			//send back balloon data
			float area = 0;
			int index = -1;
			for (int i = 0; i < balloons.size(); i++) {
				if (area < balloons[i][3]) {
					area = balloons[i][3];
					index = i;
				}
			}

			if (index != -1) {
				target.push_back(balloons[index]);
			}
		} else if (mode == goalSearch || mode == approachGoal || mode == scoringStart) {
			//send back goal data
			float area = 0;
			int index = -1;
			for (int i = 0; i < goals.size(); i++) {
				if (area < goals[i][3]) {
					area = goals[i][3];
					index = i;
				}
			}

			if (index != -1) {
				target.push_back(goals[index]);
			}
		}

		//Debuging
		//print target
		
		for (int i = 0; i < target.size(); i++) {
			cout << "X: " << target[i][0] << endl;
			cout << "Y: " << target[i][1] << endl;
			cout << "Z: " << target[i][2] << endl;
			cout << "Area: " << target[i][3] << endl;
		}
		
		//benchmark("Create message");

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

			//benchmark("Send message");
			//send message
			sendSerial(message);

			//benchmark("Output2");


			//ignores timeouts for serial communication
			if(debugMode) continue;

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
			
			cout << endl;
			if (teensyKeys.size() == teensyValues.size()) {
				for (int i = 0; i < teensyKeys.size(); i++) {
					cout << teensyKeys[i] << ": " << teensyValues[i] << endl;
				}
			}
			cout << endl;


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
		//benchmarkPrint();
	}

	cout << "DONE" << endl;

	return 0;
}


//==================== FUNCTION HEADERS ====================
VideoCapture openCamera(int camIndex, int camWidth, int camHeight){
	VideoCapture cap(camIndex);
	if (!cap.isOpened()) {
		CV_Assert("Cam open failed");
	}

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

//==================== COMMUNICATION FUNCTION HEADERS ====================
//int serial;

void initSerial(){
	if (serial.openDevice("/dev/ttyS0", 115200)!=1){
		cout << "Unable to open serial port." << endl;
		delay(1.0);
	}
	/*
	while ((serial = serialOpen ("/dev/ttyS0", 115200)) < 0) {
		cout << "Unable to open serial port." << endl;
		delay(1.0);
	}*/
}

void sendSerial(string message){
	serial.writeString(message.c_str());
	//char* sendBuffer = &message[0];
	//serialPuts(serial, sendBuffer);
	//cout << "Serial sending: " << message << endl << endl;
}

char readSerial() {
	char byte = 0;

	if(serial.available() > 0){
		serial.readBytes(&byte, 1);
	}

	/*
	int bytesReceived = serialDataAvail(serial);

	if (bytesReceived > 0) {
		//Read a pending byte into the buffer
			byte = (char)serialGetchar(serial);
	}*/

	//cout << (int)byte << "\t" << bytesReceived << endl;

	return byte;
}
//cout << "establish blimp ID->i";
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
	//cout << "establish blimp ID->i";
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
			//cout << "establish blimp ID->i";
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

void establishBlimpID(){
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
