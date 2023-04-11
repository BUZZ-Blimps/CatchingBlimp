// ============================== INCLUDES ==============================
#include <libv4l2.h>
#include <iostream>

#include "CameraHandler.h"
#include "PiComm.h"

using namespace std;
using namespace cv;

// ============================== CLASS ==============================

// Store pointer to piComm, open camera
void CameraHandler::init(PiComm* piComm, bool streamFrames, bool* program_running, int cap_device_id){
    this->piComm = piComm;
    this->streamFrames = streamFrames;
    this->program_running = program_running;

    // Set up video capture
	cv::VideoCapture cap;

	if(cap_device_id == -1){
		cap_device_id = CAMERA_INDEX;
	}
    cap.open(cap_device_id, CAMERA_API);
	if(!cap.isOpened()){
		cap.open(CAMERA_INDEX_BACKUP, CAMERA_API);
		if(cap.isOpened()){
			cap_device_id = CAMERA_INDEX_BACKUP;
		}else{
			fprintf(stderr, "ERROR! Unable to open camera\n");
            exit(EXIT_FAILURE);
		}
	}
	fprintf(stdout, "Successfully opened camera (index=%d).\n");

    // Set the stereo cam to full resolution
	cap.set(cv::CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);

    // Init class data
    newFrameNum = 0;
    prevFrameNum = 0;

    // Start thread
    if (pthread_create(&capture_thread, NULL, CameraHandler::staticCaptureThread_start, this) < 0) {
        perror("pthread_create failure in CameraHandler");
        exit(EXIT_FAILURE);
    }
}

// Static function to run thread member function
// https://cplusplus.com/forum/unices/138864/
void* CameraHandler::staticCaptureThread_start(void* arg){
    static_cast<CameraHandler*>(arg)->captureThread_loop();
}

void CameraHandler::captureThread_loop(){
    while(*program_running){
        // If no frame is available, continue
        if(!cap.grab()) continue;
        
        Mat rawFrame;
        cap.retrieve(rawFrame);

        // Crop the left and right images
        Rect left_roi(0, 0, rawFrame.cols/2, rawFrame.rows);
        Rect right_roi(rawFrame.cols/2, 0, rawFrame.cols/2, rawFrame.rows);
        Mat leftFrame_maxres(rawFrame, left_roi);
        Mat rightFrame_maxres(rawFrame, right_roi);

        // Reduce image size for rectification
        Mat leftFrame_lowres, rightFrame_lowres;
        resize(leftFrame_maxres, leftFrame_lowres, Size(RECT_WIDTH, RECT_HEIGHT), INTER_LINEAR);
        resize(rightFrame_maxres, rightFrame_lowres, Size(RECT_WIDTH, RECT_HEIGHT), INTER_LINEAR);

        // Copy images to shared memory
        pthread_mutex_lock(&mutex_newFrame);
        leftFrameCopied = leftFrame_lowres;
        rightFrameCopied = rightFrame_lowres;
        pthread_mutex_unlock(&mutex_newFrame);
        pthread_mutex_lock(&mutex_newFrameNum);
        newFrameNum++;
        pthread_mutex_unlock(&mutex_newFrameNum);

        if(streamFrames){
            piComm->setStreamFrame(leftFrame_lowres);
        }
    }
}

bool CameraHandler::getRecentFrames(Mat* leftFrame, Mat* rightFrame){
    // Really fast mutex
    unsigned int tempFrameNum;
    pthread_mutex_lock(&mutex_newFrameNum);
    tempFrameNum = newFrameNum;
    pthread_mutex_unlock(&mutex_newFrameNum);

    if(prevFrameNum > tempFrameNum){
        // Current frame has not been acquired, get and return it
        prevFrameNum = tempFrameNum;
        Mat leftFrameTemp, rightFrameTemp;
        pthread_mutex_lock(&mutex_newFrame);
        leftFrameTemp = leftFrameCopied;
        rightFrameTemp = rightFrameCopied;
        pthread_mutex_unlock(&mutex_newFrame);
        // Return frames and true
        *leftFrame = leftFrameTemp;
        *rightFrame = rightFrameTemp;
        return true;
    }else{
        // Current frame has already been got, do not return it
        return false;
    }
}