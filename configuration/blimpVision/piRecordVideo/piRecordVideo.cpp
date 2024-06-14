#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace std;
using namespace cv;

//Display camera feed
int main(int, char**) {
	VideoCapture inputVideo(2);

	//Make sure camera is connected
	if (!inputVideo.isOpened()) {
		CV_Assert("Cam open failed");
	}

	inputVideo.set(CAP_PROP_FRAME_WIDTH, 2560);
	inputVideo.set(CAP_PROP_FRAME_HEIGHT, 960);

	int fps = inputVideo.get(CAP_PROP_FPS);

	int secondsToRecord = 120;
	int numFrames = secondsToRecord*fps;

	Size S = Size((int) inputVideo.get(CAP_PROP_FRAME_WIDTH)/2,    // Acquire input size
	              (int) inputVideo.get(CAP_PROP_FRAME_HEIGHT));

	VideoWriter outputVideo;

	outputVideo.open("recording.avi", cv::VideoWriter::fourcc('M','J','P','G'), fps, S, true);
    if (!outputVideo.isOpened())
    {
	cout  << "Could not open the output video for write" << endl;
	return -1;
    }

	Mat img, imgResize;
	Mat frame;
	for (int i = 0; i < numFrames; i++) {

		inputVideo >> frame;

		//for only one lens feedback
        // Crop the left and right images
        Rect left_roi(0, 0, frame.cols/2, frame.rows);
        Rect right_roi(frame.cols/2, 0, frame.cols/2, frame.rows);
        Mat leftFrame_maxres(frame, left_roi);
        Mat rightFrame_maxres(frame, right_roi);

		if (frame.empty()) {
			break;
		}

		outputVideo.write(leftFrame_maxres);
		imshow("CameraOutput", leftFrame_maxres);

		waitKey(1);
	}

	// When everything done, release the video capture object
	inputVideo.release();

	return 0;
}

