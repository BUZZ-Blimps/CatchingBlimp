#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>

using namespace std;
using namespace cv;
using namespace chrono;

#define CAMERA_INDEX	       	2
#define CAMERA_WIDTH            2560   //px
#define CAMERA_HEIGHT           960    //px

VideoCapture openCamera(int camIndex, int camWidth, int camHeight);
void delay(double delaySeconds);

int main(int argc, char** argv) {
	//cout << "Trying to open camera..." << endl;
	VideoCapture cap = openCamera(CAMERA_INDEX, CAMERA_WIDTH, CAMERA_HEIGHT);
	//cout << "Camera opened?" << endl;
	//cout << "Waiting 2 seconds..." << endl;
	//delay(2);

	while(true) {
		Mat frame;
		cap >> frame;
		imshow("Camera Input",frame);
		waitKey(1);
	}

	return 0;
}

VideoCapture openCamera(int camIndex, int camWidth, int camHeight){
	VideoCapture cap(camIndex, CAP_V4L2);
	if (!cap.isOpened()) {
		CV_Assert("Cam open failed");
	}

	//cap.set(CAP_PROP_FPS, 120);
	cap.set(CAP_PROP_FRAME_WIDTH, camWidth);
	cap.set(CAP_PROP_FRAME_HEIGHT, camHeight);
	return cap;
}

void delay(double delaySeconds){
	clock_t start = clock();
	while(double(clock() - start)/CLOCKS_PER_SEC < delaySeconds);
}