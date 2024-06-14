#pragma once

// ============================== INCLUDES ==============================
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/ximgproc.hpp>
#include <vector>

#include "EnumUtil.h"

using namespace std;
using namespace cv;

// ============================== DEFINES ==============================
#define USE_VIDEO       false
#define VIDEO_FPS       30

#define CAMERA_INDEX    0

#define CAMERA_WIDTH	320
#define CAMERA_HEIGHT	240

#define RECT_WIDTH		320
#define RECT_HEIGHT		240

#define F               420
#define BASELINE        0.062 //0.20341207

#define PRE_FILTER_SIZE	7
#define PRE_FILTER_CAP	2
#define UNIQUENESS_RATIO	5

#define LAMBDA			17.8
#define SIGMA			5.0

#define AVOID_AREA		6000
#define AVOID_DIST		70

#define DISP_WIDTH		160
#define DISP_HEIGHT		120

#define B_CORRECTION	Scalar(0,19,0)
#define B_MIN			Scalar(105,0,19)
#define B_MAX			Scalar(171,255,255)

#define ORANGE_G_CORRECTION    Scalar(56,68, 0)
#define ORANGE_G_MIN           Scalar(0,0,185) //orange min changed to 10
#define ORANGE_G_MAX           Scalar(61,255,255) //orange min changed to 35


#define STEREO_CAL_FILENAME     "stereo_rectify_maps240p.xml"

//#define STEREO_CAL_PATH     "/home/corelab-laptop2/Documents/testCams/piTestCompileSrc/stereo_rectify_maps240p.xml"



// ============================== CLASS ========#define AVOID_AREA		6000======================
class ComputerVision {
    private:
        VideoCapture cap;
        bool capturing = true;
        Mat lastCaptured;

        // Stereo Calibration
        Mat Left_Stereo_Map1;
        Mat Left_Stereo_Map2;
        Mat Right_Stereo_Map1;
        Mat Right_Stereo_Map2;
        Mat Q;

        // Stereo matcher
        Ptr< StereoBM > stereo;
        Ptr< StereoBM > left_matcher;
        Ptr<ximgproc::DisparityWLSFilter> wls_filter;
        Ptr<StereoMatcher> right_matcher;

        // Identified objects
        std::vector<vector<float> > balloons;
        std::vector<vector<float> > goals;

        // Goal detection
        double pixelDensityL = 0.2;
        double pixelDensityR = 0.2;
        Mat output,output_norm,output_norm_scaled;
        int quad;

        /*
        int correction1 = 0;
        int correction2 = 0;
        int correction3 = 0;
        int min1 = 117;
        int min2 = 90;
        int min3 = 40;
        int max1 = 255;
        int max2 = 234;
        int max3 = 255;
        */

        int correction1 = 0;
        int correction2 = 19;
        int correction3 = 0;
        int min1 = 105;
        int min2 = 0;
        int min3 = 19;
        int max1 = 171;
        int max2 = 255;
        int max3 = 255;

        /*
        int correction1 = 0;
        int correction2 = 0;
        int correction3 = 0;
        int min1 = 117;
        int min2 = 90;
        int min3 = 40;
        int max1 = 255;
        int max2 = 234;
        int max3 = 255;
        */

        // Masks purple and red
        /*
        int correction1 = 0;
        int correction2 = 11;
        int correction3 = 33;
        int min1 = 100;
        int min2 = 0;
        int min3 = 19;
        int max1 = 123;
        int max3 = 58;
        */

       int targetH = 127;
       int targetS = 60;
       int targetV = 160;
       int minH = 0;
       int minS = 0;
       int minV = 0;
       int maxH = 0;
       int maxS = 0;
       int maxV = 0;
       int maxDiff = 0;

    public:
        void init();
        void setQ();
        void readCalibrationFiles(string srcDir);
        void update(autoState mode, goalType goalColor);
        void getFrames(Mat &imgL, Mat &imgR);
        bool getBall(float &X, float &Y, float &Z, float &area, Mat imgL, Mat imgR);
        bool tuneBall(float &X, float &Y, float &Z, float &area, Mat imgL, Mat imgR);
        bool tuneBall_Lawson(float &X, float &Y, float &Z, float &area, Mat imgL, Mat imgR);
        void getGoal(float &X, float &Y, float &Z, float &area, float &angle, Mat imgL, Mat imgR);
        void tuneGoal(float &X, float &Y, float &Z, float &area, float &angle, Mat imgL, Mat imgR);
        int getAvoidance(Mat imgL, Mat imgR);
        float get_avg_dist_FM(Mat imgL, Mat imgR, String index);
        float get_avg_dist_DM(Mat imgL, Mat imgR, String index);
        float get_avg_dist_CA(Mat imgL, Mat imgR, String index);
        vector<vector<float>> getTargetBalloon();
        vector<vector<float>> getTargetGoal();
        int getQuad();
};


