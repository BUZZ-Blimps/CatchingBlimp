#pragma once

// ============================== INCLUDES ==============================
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/ximgproc.hpp>
#include <vector>

#include "EnumUtil.h"

using namespace std;
using namespace cv;

// ============================== DEFINES ==============================
#define CAMERA_INDEX       0


// ============================== CLASS ==============================
class CompVisNew {
    private:
        VideoCapture cam;

    public:
        void init();


};