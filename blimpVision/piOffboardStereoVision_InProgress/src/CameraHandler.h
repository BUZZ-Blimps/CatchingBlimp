#pragma once

// ============================== INCLUDES ==============================
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/ximgproc.hpp>

#include "PiComm.h"
#include "Util.h"

using namespace std;
using namespace cv;

// ============================== DEFINES ==============================
#define CAMERA_INDEX       0
#define CAMERA_INDEX_BACKUP 1
#define CAMERA_API      cv::CAP_V4L

#define CAMERA_WIDTH		1280 //320
#define CAMERA_HEIGHT		720  //240

#define RECT_WIDTH		320
#define RECT_HEIGHT		240

// ============================== CLASS ==============================

class CameraHandler {
    private:
        VideoCapture cap;
        Mat leftFrameCopied, rightFrameCopied;
        unsigned int newFrameNum;
        unsigned int prevFrameNum;

        PiComm* piComm;

        ProgramData* programData;

        pthread_mutex_t mutex_newFrame;
        pthread_mutex_t mutex_newFrameNum;

        pthread_t capture_thread;
        static void* staticCaptureThread_start(void* arg);
        void captureThread_loop();

    public:
        void init(PiComm* piComm, ProgramData* programData);
        bool getRecentFrames(Mat* leftFrame, Mat* rightFrame);

};
