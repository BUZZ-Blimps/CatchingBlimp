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
#define CAMERA_INDEX       2
#define CAMERA_WIDTH	320
#define CAMERA_HEIGHT	240

#define CV_WIDTH		320
#define CV_HEIGHT		240

#define RECT_WIDTH		320
#define RECT_HEIGHT		240

#define DISP_WIDTH		160
#define DISP_HEIGHT		120

#define STEREO_CAL_PATH     "/home/corelab-laptop2/Documents/testCams/piTestCompileSrc/stereo_rectify_maps240p.xml"

#define PRE_FILTER_SIZE	7
#define PRE_FILTER_CAP	2
#define UNIQUENESS_RATIO	5

#define LAMBDA			17.8
#define SIGMA			5.0

#define AVOID_DIST		70
#define AVOID_AREA		6000

#define MIN_AREA		50
#define SIZE_RATIO		3

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

#define CONVERSION		0.15

#define E_ITER 1
#define D_ITER 1
#define E_SIZE 1
#define D_SIZE 1

#define GOAL_DILATION_ADJUST 4

#define G_POLY_APPROX_E          0.01
#define GOAL_INNER_CONTOUR_SCALE 0.7
#define GOAL_CONFIRM			 6000

// ============================== CLASS ==============================

class ComputerVision {
    private:
        VideoCapture cap;

        // Stereo calibration
        Mat Left_Stereo_Map1;
        Mat Left_Stereo_Map2;
        Mat Right_Stereo_Map1;
        Mat Right_Stereo_Map2;
        Mat Q;

        // Stereo matcher
        Ptr< StereoBM > left_matcher;
        Ptr<ximgproc::DisparityWLSFilter> wls_filter;
        Ptr<StereoMatcher> right_matcher;

        // Identified objects
        std::vector<vector<float> > balloons;
        std::vector<vector<float> > goals;
        // Object avoidance quadrant
        int quad;

        vector<Point> scaleContour(vector<Point> contour, float scale);

    public:
        void init();
        void readCalibrationFiles();
        void update(autoState mode, goalType goalColor); // Big image processing function
        vector<vector<float>> getTargetBalloon();
        vector<vector<float>> getTargetGoal();
        int getQuad();

        

};
