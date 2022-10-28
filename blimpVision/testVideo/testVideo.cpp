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

//Camera
#define CAMERA_WIDTH	1280
#define CAMERA_HEIGHT	960

using namespace std;
using namespace cv;
using namespace chrono;

//==================== FUNCTION HEADERS ====================
VideoCapture openCamera(int camIndex, int camWidth, int camHeight);
void initVideo(string videoName, int fps, Size size);
void saveToVideo(Mat frame);

int framesSaved = 0;
//==================== MAIN ====================

int main(int argc, char** argv) {
	clock_t startTime = clock();
	VideoCapture cap = openCamera(0,CAMERA_WIDTH*2,CAMERA_HEIGHT);
	Mat frame;
	cap >> frame;

	VideoWriter outputVideo;
	outputVideo.open("outputVideo.avi",cv::VideoWriter::fourcc('M','J','P','G'),30.0,Size(CAMERA_WIDTH,CAMERA_HEIGHT),true);
	if (!outputVideo.isOpened()){
		cout  << "Could not open the output video for write" << endl;
	}

	//imshow("Frame",frame);

	while(true){
		clock_t currentTime = clock();
		double elapsedTime = double(currentTime - startTime)/CLOCKS_PER_SEC;

		cout << "ET: " << elapsedTime << endl;
		cap >> frame;
		cout << "Size: " << frame.size() << endl;

		Mat imgL;
		Rect left_roi(0, 0, frame.cols/2, frame.rows);
		Mat crop_left(frame, left_roi);
		crop_left.copyTo(imgL);

		//imshow("Frame",frame);
		outputVideo.write(imgL);

		//waitKey(1);
		if(elapsedTime > 1) break; //Quit
		continue;



		Mat randMat(Size(CAMERA_WIDTH, CAMERA_HEIGHT), CV_8UC1);
		randMat.setTo(Scalar(0,0,0));

		if(!randMat.empty()){
			outputVideo.write(randMat);
			framesSaved++;
			if(framesSaved/100 == framesSaved/100.0) cout << "Saved " << framesSaved << " frames." << endl;
		}

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

/*
VideoWriter outputVideo;
void initVideo(string videoName, int fps, Size size){
	outputVideo.open(videoName, cv::VideoWriter::fourcc('M','J','P','G'), fps, size, true);
	if (!outputVideo.isOpened()){
		cout  << "Could not open the output video for write" << endl;
	}
}

void saveToVideo(Mat frame){
	if(!frame.empty()){
		outputVideo.write(frame);
		framesSaved++;
		if(framesSaved/100 == framesSaved/100.0) cout << "Saved " << framesSaved << " frames." << endl;
	}
}
*/

/*
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

//Camera
#define CAMERA_WIDTH	1280
#define CAMERA_HEIGHT	720

using namespace std;
using namespace cv;
using namespace chrono;

//==================== FUNCTION HEADERS ====================
VideoCapture openCamera(int camIndex, int camWidth, int camHeight);
void initVideo(string videoName, int fps, Size size);
void saveToVideo(Mat frame);

int framesSaved = 0;
//==================== MAIN ====================

int main(int argc, char** argv) {
	clock_t startTime = clock();
	VideoCapture cap(0);
	Mat frame;
	cap >> frame;

	VideoWriter outputVideo;
	string fileName = "outputVideo.avi";
	int codec = cv::VideoWriter::fourcc('M','J','P','G');
	double fps = 30;
	Size recordingSize(CAMERA_WIDTH,CAMERA_HEIGHT);
	outputVideo.open(fileName,codec,fps,frame.size(),true);
	if (!outputVideo.isOpened()){
		cout  << "Could not open the output video for write" << endl;
	}

	imshow("Frame",frame);

	while(true){
		clock_t currentTime = clock();
		double elapsedTime = double(currentTime - startTime)/CLOCKS_PER_SEC;

		cout << "ET: " << elapsedTime << endl;
		cap >> frame;
		imshow("Frame",frame);
		outputVideo.write(frame);

		waitKey(1);
		if(elapsedTime > 1) break; //Quit
		continue;


		Mat imgL;
		Rect left_roi(0, 0, frame.cols/2, frame.rows);
		Mat crop_left(frame, left_roi);
		crop_left.copyTo(imgL);

		Mat randMat(Size(CAMERA_WIDTH, CAMERA_HEIGHT), CV_8UC1);
		randMat.setTo(Scalar(0,0,0));

		if(!randMat.empty()){
			outputVideo.write(randMat);
			framesSaved++;
			if(framesSaved/100 == framesSaved/100.0) cout << "Saved " << framesSaved << " frames." << endl;
		}

	}

	cout << outputVideo.get(CAP_PROP_POS_MSEC) << endl;
	cout << outputVideo.get(CAP_PROP_FRAME_WIDTH) << endl;
	cout << outputVideo.get(CAP_PROP_FRAME_HEIGHT) << endl;
	cout << outputVideo.get(CAP_PROP_FPS) << endl;
	cout << outputVideo.get(CAP_PROP_FOURCC) << endl;
	cout << outputVideo.get(CAP_PROP_FORMAT) << endl;

	cout << "DONE; " << framesSaved << " frames." << endl;
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

/*
VideoWriter outputVideo;
void initVideo(string videoName, int fps, Size size){
	outputVideo.open(videoName, cv::VideoWriter::fourcc('M','J','P','G'), fps, size, true);
	if (!outputVideo.isOpened()){
		cout  << "Could not open the output video for write" << endl;
	}
}

void saveToVideo(Mat frame){
	if(!frame.empty()){
		outputVideo.write(frame);
		framesSaved++;
		if(framesSaved/100 == framesSaved/100.0) cout << "Saved " << framesSaved << " frames." << endl;
	}
}
*/
