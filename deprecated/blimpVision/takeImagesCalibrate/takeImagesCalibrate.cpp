#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/ximgproc.hpp"
#include <opencv2/viz.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <ctime>
#include <chrono>

//<-----------Constants------------->
#define CAMERA_INDEX	        2
#define CAMERA_WIDTH            2560   //px
#define CAMERA_HEIGHT           960    //px
#define CAMERA_API              CAP_V4L2

#define CAMERA_READ_WIDTH       640
#define CAMERA_READ_HEIGHT      480

#define AVOIDENCE_SIZE          30

#define GOAL_CONFIRM            50

#define CV_WINDOW_AUTOSIZE      cv::WINDOW_NORMAL

#define RECT_WIDTH              640
#define RECT_HEIGHT             480

#define DISPLAY_H		240
#define DISPLAY_W		320

#define DISP_WIDTH              160
#define DISP_HEIGHT             120

#define LAB_MAP_ADDRESS         "stereo_rectify_maps.xml"
#define HOME_MAP_ADDRESS        "stereo_rectify_maps.xml"

#define INTERPOLATION           INTER_AREA

#define CAMERA_FLAGS            cv::CALIB_USE_INTRINSIC_GUESS | cv::CALIB_RATIONAL_MODEL | cv::CALIB_THIN_PRISM_MODEL
#define STEREO_FLAGS            cv::CALIB_USE_INTRINSIC_GUESS | cv::CALIB_RATIONAL_MODEL | cv::CALIB_THIN_PRISM_MODEL
#define RECTIFY_FLAGS           cv::CALIB_ZERO_DISPARITY

#define ALPHA_CAMERA            0.0
#define ALPHA_STEREO            0.0

#define WAIT                    10 //250

#define BASELINE                50.0      //mm
#define FOCAL                   650.0     //mm

#define LAMBDA                  0.5
#define SIGMA                   0.5
#define VIS_MULT                255.0

//Size of squares in mm (I think?)
#define CHECKERBOARD_SIZE_WIDTH         30.0*0.9
#define CHECKERBOARD_SIZE_HEIGHT        30.0*0.9

//Number of interior corners
//Small squares = 13x18, Big squares = 9x13
#define CHECKERBOARD_NUM_HORIZONTAL             18
#define CHECKERBOARD_NUM_VERTICAL               13

//#define PATHL                   "takeImagesCalibrate/tempImages/imgL"
//#define PATHR                   "takeImagesCalibrate/tempImages/imgR"
#define PATHL                   "tempImages/imgL"
#define PATHR                   "tempImages/imgR"
#define IMGEXTENSION            ".jpg"

#define CORRECT_COLOR           false

#define B_CORRECTION            Scalar(29,7,15)
#define R_Correction            Scalar(0,0,0)
#define G_CORRECTION            Scalar(0,0,0) //Orange          Yellow: Scalar(48,0,0)

#define B_MIN                   Scalar(46,0,0)
#define B_MAX                   Scalar(96,74,213)

#define R_MIN                   Scalar(0,0,81)
#define R_MAX                   Scalar(87,78,255)

#define G_MIN                   Scalar(0,61,155) //Orange       Yellow: Scalar(25, 0, 0)
#define G_MAX                   Scalar(20,196,255) //Orange     Yellow: Scalar(63, 255, 255)

#define G_POLY_APPROX_E          0.01
#define GOAL_INNER_CONTOUR_SCALE 0.6

#define GOAL_CONFIDENCE         0.40
#define MIN_AREA                50.0
#define SIZE_RATIO              1.5

#define E_ITER 1
#define D_ITER 1
#define E_SIZE 1
#define D_SIZE 1

#define GOAL_DILATION_ADJUST 3

//Camera
using namespace cv;
using namespace std;

vector<Point> scaleContour(vector<Point> contour, double scale);
string type2str(int type);
void delay(double delaySeconds);

bool render3DProjection = true;

enum modes {
        detectBall,
        detectGoal
};

int mode = detectBall;

int CHECKERBOARD[2]{CHECKERBOARD_NUM_HORIZONTAL,CHECKERBOARD_NUM_VERTICAL};

void benchmarkFirst(string flag);
void benchmark(string flag);
void benchmarkPrint();

void takeStereoImages(bool autoCap = false) {
	VideoCapture inputVideo(CAMERA_INDEX,CAMERA_API);

	inputVideo.set(CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
	inputVideo.set(CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);

	//Make sure camera is connected
	if (!inputVideo.isOpened()) {
			CV_Assert("Cam open failed");
			cout << "camera failed to open" << endl;
			return;
	}

	int count = 0;

	cout << "Space takes picture" << endl;
	cout << "Escape key exits first loop" << endl << endl;

	while (true) {
                //get a frame from the camera
                Mat frame;
                inputVideo >> frame;

                if (frame.rows <= 0 || frame.cols <= 0) {
                        continue;
                }
                
                Mat frame2;
                resize(frame, frame2, Size(DISPLAY_W*2, DISPLAY_H), INTER_LINEAR);

                imshow("Camera Footage", frame2);
                

                //split frame into two images
                Mat imgL, imgR;
               
                Rect left_roi(0, 0, frame.cols/2, frame.rows);
                Rect right_roi (frame.cols/2, 0, frame.cols/2, frame.rows);

                Mat crop_left(frame, left_roi);
                Mat crop_right (frame, right_roi);

                crop_left.copyTo(imgL);
                crop_right.copyTo(imgR); 

                if (autoCap == false) {
                        int c = waitKey(10);

                        //Save L/R image if desired
                        if (c % 256 == 32) {
                                //create image path
                                String pathL = PATHL + to_string(count) + IMGEXTENSION;
                                String pathR = PATHR + to_string(count) + IMGEXTENSION;
                                count += 1;
                                //save images
                                cout << "Save Image " << pathL << endl;
                                cout << "Save Image " << pathR << endl;
                                cout << "Rows: " << frame.rows << endl;
                                cout << "Colm: " << frame.cols << endl;
                                imwrite(pathL, imgL);
                                imwrite(pathR, imgR);
                                cout << "Attempted save." << endl;
                                cout << imgL.size() << endl;
                        } else if (c % 256 == 27) {
                                cout << "Exiting Camera" << endl;
                                break;
                        }
                } else {
                        int c = waitKey(250);

                        if (c % 256 == 27) {
                                cout << "Exiting Camera Calibration" << endl;
                                break;
                        }

                        //create image path
                        String pathL = PATHL + to_string(count) + IMGEXTENSION;
                        String pathR = PATHR + to_string(count) + IMGEXTENSION;
                        count += 1;
                        //save images
                        cout << "Save Image " << pathL << endl;
                        cout << "Save Image " << pathR << endl;
                        cout << "Rows: " << frame.rows << endl;
                        cout << "Colm: " << frame.cols << endl;
                        imwrite(pathL, imgL);
                        imwrite(pathR, imgR);
                }
	}

        // When everything done, release the video capture object
	inputVideo.release();
}

void undistortCamerasCalibrateStereo(bool autoCap = false) {

        cout << "Starting picture selection." << endl;
        cout << "For a given photo, if prompted:" << endl;
        cout << "\tPress 'y' for yes" << endl;
        cout << "\tPress 'n' for no" << endl;
        cout << "\tPress 'a' to select all found board pairs" << endl;

        int count = 0;

        std::vector<std::vector<cv::Point3f> > objpointsL;
        std::vector<std::vector<cv::Point3f> > objpointsR;

        std::vector<std::vector<cv::Point2f> > imgpointsL;
        std::vector<std::vector<cv::Point2f> > imgpointsR;

        std::vector<std::vector<cv::Point3f> > objpointsstereo;
        std::vector<std::vector<cv::Point2f> > imgpointsLstereo;
        std::vector<std::vector<cv::Point2f> > imgpointsRstereo;

        std::vector<cv::Point3f> objp;
        for (int i{0}; i<CHECKERBOARD[1]; i++) {
                for (int j{0}; j<CHECKERBOARD[0]; j++) {
                        objp.push_back(cv::Point3f(j*CHECKERBOARD_SIZE_HEIGHT,i*CHECKERBOARD_SIZE_WIDTH,0));
                }
        }

        Mat grayL, grayR;

        bool useAll = autoCap;

        int numLeftChess = 0;
        int numRightChess = 0;
        int numStereoPair = 0;
        int totalStereoPair = 0;

        while (true) {
                //iterate through images after looking for board and check if keypoints are acceptable
                //get path of images
                String pathL = PATHL + to_string(count) + IMGEXTENSION;
                String pathR = PATHR + to_string(count) + IMGEXTENSION;


                cout << endl << "Checking Board Pair: " << count << "..."<< endl;
                count += 1;

                //read pair
                Mat imgL = imread(pathL);
                Mat imgR = imread(pathR);

                if (imgL.rows <= 0 || imgL.cols <= 0 || imgR.rows <= 0 || imgR.cols <= 0) {
                        cout << "No more images found" << endl;
                        break;
                }
                
                Mat imgL2;
                Mat imgR2;
                resize(imgL, imgL2, Size(DISPLAY_W, DISPLAY_H), INTER_LINEAR);
                resize(imgR, imgR2, Size(DISPLAY_W, DISPLAY_H), INTER_LINEAR);

                imshow("Left Frame", imgL2);
                imshow("Right Frame", imgR2);

                //find boards in both images
                cv::cvtColor(imgL,grayL,cv::COLOR_BGR2GRAY);
                cv::cvtColor(imgR,grayR,cv::COLOR_BGR2GRAY);

                std::vector<cv::Point2f> corner_ptsL;
                std::vector<cv::Point2f> corner_ptsR;

                bool successL = cv::findChessboardCorners(grayL, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_ptsL, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
                bool successR = cv::findChessboardCorners(grayR, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_ptsR, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);

                if (successL) {

                        numLeftChess++;

                        //sub pixel calculation
                        cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 100000, 0.000001);
                        //cv::TermCriteria criteria(TERMCRIT_EPS | TERMCRIT_ITER, 30, 0.001);
       
                        // refining pixel coordinates for given 2d points.
                        cv::cornerSubPix(grayL,corner_ptsL,cv::Size(11,11), cv::Size(-1,-1),criteria);

                        if (corner_ptsL[0].x > corner_ptsL[CHECKERBOARD[1]-1].x) {
                                std::reverse(corner_ptsL.begin(), corner_ptsL.end());
                        }

                        cv::drawChessboardCorners(imgL, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_ptsL, successL);
                        Mat imgL3;
                        resize(imgL, imgL3, Size(DISPLAY_W, DISPLAY_H), INTER_LINEAR);
                        imshow("Left Frame", imgL3);

                        

                        if (!useAll) {
                                int c = waitKey(0);

                                if (c % 256 == 121) {
                                        //y
                                        objpointsL.push_back(objp);
                                        imgpointsL.push_back(corner_ptsL);

                                } else if (c % 256 == 110) {
                                        //n
                                } else if (c % 256 == 97) {
                                        //a
                                        //all found boards should be used
                                        useAll = true;
                                        //push points
                                        objpointsL.push_back(objp);
                                        imgpointsL.push_back(corner_ptsL);
                                }
                        } else {
                                objpointsL.push_back(objp);
                                imgpointsL.push_back(corner_ptsL);
                        }
                }

                if (successR) {

                        numRightChess++;

                        //sub pixel calculation
                        cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 100000, 0.000001);
                        //cv::TermCriteria criteria(TERMCRIT_EPS | TERMCRIT_ITER, 30, 0.001);
       
                        // refining pixel coordinates for given 2d points.
                        cv::cornerSubPix(grayR,corner_ptsR,cv::Size(11,11), cv::Size(-1,-1),criteria);

                        //show right board
                        if (corner_ptsR[0].x > corner_ptsR[CHECKERBOARD[1]-1].x) {
                                std::reverse(corner_ptsR.begin(), corner_ptsR.end());
                        }

                        cv::drawChessboardCorners(imgR, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_ptsR, successR);
                        Mat imgR3;
                        resize(imgR, imgR3, Size(DISPLAY_W, DISPLAY_H), INTER_LINEAR);
                        imshow("Right Frame", imgR3);

                        if (!useAll) {
                                int c = waitKey(0);

                                if (c % 256 == 121) {
                                        //y
                                        objpointsR.push_back(objp);
                                        imgpointsR.push_back(corner_ptsR);
                                } else if (c % 256 == 110) {
                                        //n
                                } else if (c % 256 == 97) {
                                        //a
                                        //all found boards should be used
                                        useAll = true;
                                        //push points
                                        objpointsR.push_back(objp);
                                        imgpointsR.push_back(corner_ptsR);
                                }
                        } else {
                                objpointsR.push_back(objp);
                                imgpointsR.push_back(corner_ptsR);
                        }
                }
                
                if (successL && successR) {
                        numStereoPair++;

                        objpointsstereo.push_back(objp);

                        imgpointsLstereo.push_back(corner_ptsL);
                        imgpointsRstereo.push_back(corner_ptsR); 
                }
                
                totalStereoPair++;

                bool endStep = false;

                while (!useAll) {
                        cout << "Press y to continue, q to quit, e to end step: " << endl;
                        int c = waitKey(0);
                        if (c % 256 == 121) {
                                //y was pressed
                                break;
                        } else if (c % 256 == 113) {
                                //q
                                return;
                        } else if (c % 256 == 101) {
                                //e
                                endStep = true;
                                break;
                        }
                }

                if (endStep) {
                        break;
                }

        }

        //Scale output points to correct resolution
        
        cout << "Scaling image points to correct resolution" << endl;

        for (int i = 0; i < imgpointsL.size(); i++) {
                for (int k = 0; k < imgpointsL[i].size(); k++) {
                        imgpointsL[i][k].x = imgpointsL[i][k].x * (float)RECT_WIDTH/(float)((CAMERA_WIDTH)/2);
                        imgpointsL[i][k].y = imgpointsL[i][k].y * (float)RECT_HEIGHT/(float)CAMERA_HEIGHT;
                }
        }

        for (int i = 0; i < imgpointsR.size(); i++) {
                for (int k = 0; k < imgpointsR[i].size(); k++) {
                        imgpointsR[i][k].x = imgpointsR[i][k].x * (float)RECT_WIDTH/(float)((CAMERA_WIDTH)/2);
                        imgpointsR[i][k].y = imgpointsR[i][k].y * (float)RECT_HEIGHT/(float)CAMERA_HEIGHT;
                }
        }


        for (int i = 0; i < imgpointsLstereo.size(); i++) {
                for (int k = 0; k < imgpointsLstereo[i].size(); k++) {
                        imgpointsLstereo[i][k].x = imgpointsLstereo[i][k].x * (float)RECT_WIDTH/(float)((CAMERA_WIDTH)/2);
                        imgpointsLstereo[i][k].y = imgpointsLstereo[i][k].y * (float)RECT_HEIGHT/(float)CAMERA_HEIGHT;
                }
        }

        for (int i = 0; i < imgpointsRstereo.size(); i++) {
                for (int k = 0; k < imgpointsRstereo[i].size(); k++) {
                        imgpointsRstereo[i][k].x = imgpointsRstereo[i][k].x * (float)RECT_WIDTH/(float)((CAMERA_WIDTH)/2);
                        imgpointsRstereo[i][k].y = imgpointsRstereo[i][k].y * (float)RECT_HEIGHT/(float)CAMERA_HEIGHT;
                }
        }
        

        //perform camera calibration

        cout << "Initial Values" << endl;

        Mat stdintL, stdextL, rmsL;
        Mat stdintR, stdextR, rmsR;

        String pathL = PATHL + to_string(0) + IMGEXTENSION;
        String pathR = PATHR + to_string(0) + IMGEXTENSION;

        Mat imgL = imread(pathL);
        Mat imgR = imread(pathR);

        Mat mtxL, distL, RL, TL, new_mtxL;
        Mat mtxR, distR, RR, TR, new_mtxR;

        double f = 290.5;
        double cx = RECT_WIDTH/2;
        double cy = RECT_HEIGHT/2;

        Mat mtx = Mat::zeros(Size(3, 3), CV_64FC1);
        mtx.at<double>(0,0) = f;
        mtx.at<double>(1,1) = f;
        mtx.at<double>(2,2) = 1;
        mtx.at<double>(0,2) = cx;
        mtx.at<double>(1,2) = cy;

        cout << mtx << endl;
        mtxL = mtx.clone();
        mtxR = mtx.clone();
        new_mtxL = mtx.clone();
        new_mtxR = mtx.clone();

        //lense has no distortion
        Mat dist = Mat::zeros(Size(1, 5), CV_64FC1);
        distL = dist.clone();
        distR = dist.clone();

        cout << dist << endl;
        
        int flags = CAMERA_FLAGS;

        //resize to rectification dimensions
        Mat imgL_rect, imgR_rect;

        resize(imgL, imgL_rect, Size(RECT_WIDTH, RECT_HEIGHT), INTER_LINEAR);
        resize(imgR, imgR_rect, Size(RECT_WIDTH, RECT_HEIGHT), INTER_LINEAR);

        cout << endl;
        cout << "Number of Left Chessboards: " << numLeftChess << endl;
        cout << "Number of Right Chessboards: " << numRightChess << endl;
        cout << "Number of Stereo Chessboard Pairs: " << numStereoPair << endl;
        cout << endl;
        cout << "Percentage of Left image: " << (float)numLeftChess/(float)totalStereoPair*100.0 << endl;
        cout << "Percentage of Right image: " << (float)numRightChess/(float)totalStereoPair*100.0 << endl;
        cout << "Percentage of StereoPairs: " << (float)numStereoPair/(float)totalStereoPair*100.0 << endl;
        cout << endl;
        
        cout << "Performing Left Camera Calibration" << endl;

        //make camera matrix
        double el = cv::calibrateCamera(objpointsL,
                                        imgpointsL,
                                        cv::Size(RECT_WIDTH, RECT_HEIGHT),
                                        mtxL,
                                        distL,
                                        RL,
                                        TL,
                                        stdintL,
                                        stdextL,
                                        rmsL,
                                        flags,
                                        cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 1e-6));

        Rect roiL;

        new_mtxL = cv::getOptimalNewCameraMatrix(mtxL,distL,cv::Size(RECT_WIDTH, RECT_HEIGHT),ALPHA_CAMERA,cv::Size(RECT_WIDTH, RECT_HEIGHT),&roiL);

        cout << "Final Left Camera Calibration" << endl;
        cout << "RMS: " << el << endl;
        cout << "cameraMatrix : " << mtxL << endl << endl;
        cout << distL << endl << endl;

        Mat dstL;
        undistort(imgL_rect, dstL, mtxL, distL, new_mtxL);
        

        cout << "Performing Right Camera Calibration" << endl;
       
        Mat stdIntL, stdExtL, pveL;
        double er = cv::calibrateCamera(objpointsR,
                                        imgpointsR,
                                        cv::Size(RECT_WIDTH, RECT_HEIGHT),
                                        mtxR,
                                        distR,
                                        RR,
                                        TR,
                                        stdintR,
                                        stdextR,
                                        rmsR,
                                        flags,
                                        cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 1e-6));

        Rect roiR;

        new_mtxR = cv::getOptimalNewCameraMatrix(mtxR,distR,cv::Size(RECT_WIDTH, RECT_HEIGHT),ALPHA_CAMERA,cv::Size(RECT_WIDTH, RECT_HEIGHT),&roiR);

        cout << "Final Right Camera Calibration" << endl;
        cout << "RMS: " << er << endl;
        cout << "cameraMatrix : " << mtxR << endl << endl;
        cout << distR << endl << endl;

        Mat dstR;
        undistort(imgR_rect, dstR, mtxR, distR, new_mtxR);
        if (ALPHA_CAMERA > 0) {
                rectangle(dstR, roiR, cv::Scalar(0, 255, 0), 4);
                rectangle(dstL, roiL, cv::Scalar(0, 255, 0),4);
        }

        Mat dstCalib;
        cv::hconcat(dstL, dstR, dstCalib);
        resize(dstCalib, dstCalib, Size(DISPLAY_W*2, DISPLAY_H), INTER_LINEAR);
        imshow("Camera Calibration of Stereo Pair", dstCalib);
        
        //cout << "Press any key to continue" << endl;
        waitKey(10);
        

        //perform stereo calibration
        cout << "Performing Stereo Calibration" << endl;

        Mat Rot, Trns, Emat, Fmat;
        int flag = 0;
        flag |= STEREO_FLAGS;

        Mat errors;

        stereoCalibrate(objpointsstereo,
                    imgpointsLstereo,
                    imgpointsRstereo,
                    new_mtxL,
                    distL,
                    new_mtxR,
                    distR,
                    cv::Size(RECT_WIDTH, RECT_HEIGHT),
                    Rot,
                    Trns,
                    Emat,
                    Fmat,
                    errors,
                    flag,
                    cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 1e-6));

        cout << "Stereo Calibration RMS: " << mean(errors) << endl;

        Mat rect_l, rect_r, proj_mat_l, proj_mat_r, Q;
        Rect roi1;
        Rect roi2;

        double alpha = ALPHA_STEREO;

        //Change output resolution for correct size

        cv::stereoRectify(new_mtxL,
                  distL,
                  new_mtxR,
                  distR,
                  cv::Size(RECT_WIDTH, RECT_HEIGHT),
                  Rot,
                  Trns,
                  rect_l,
                  rect_r,
                  proj_mat_l,
                  proj_mat_r,
                  Q,
                  RECTIFY_FLAGS,
                  alpha,
                  cv::Size(RECT_WIDTH, RECT_HEIGHT),      //ouput resolution
                  &roi1,
                  &roi2);

        Mat Left_Stereo_Map1, Left_Stereo_Map2;
        Mat Right_Stereo_Map1, Right_Stereo_Map2;

        initUndistortRectifyMap(new_mtxL,
                                distL,
                                rect_l,
                                proj_mat_l,
                                cv::Size(RECT_WIDTH, RECT_HEIGHT),
                                CV_16SC2,
                                Left_Stereo_Map1,
                                Left_Stereo_Map2);

        initUndistortRectifyMap(new_mtxR,
                                distR,
                                rect_r,
                                proj_mat_r,
                                cv::Size(RECT_WIDTH, RECT_HEIGHT),
                                CV_16SC2,
                                Right_Stereo_Map1,
                                Right_Stereo_Map2);

        cout << "Q Matrix: " << endl;
        cout << Q << endl;
        cout << endl;

        Mat left_correct, right_correct;

        

        remap(imgL_rect,
                left_correct,
                Left_Stereo_Map1,
                Left_Stereo_Map2,
                INTERPOLATION,
                BORDER_CONSTANT,
                0);

        remap(imgR_rect,
                right_correct,
                Right_Stereo_Map1,
                Right_Stereo_Map2,
                INTERPOLATION,
                BORDER_CONSTANT,
                0);

        //plot roi on corrected image if alpha is used
        if (alpha > 0) {
                rectangle(left_correct, roi1, cv::Scalar(0, 255, 0),4);
                rectangle(right_correct, roi2, cv::Scalar(0, 255, 0),4);
        }
        Mat stereoImages;
        cv::hconcat(left_correct, right_correct, stereoImages);
        resize(stereoImages, stereoImages, Size(DISPLAY_W*2, DISPLAY_H), INTER_LINEAR);
        imshow("Corrected Stereo Pair", stereoImages);
        
        Mat original;
        cv::hconcat(imgL_rect, imgR_rect, original);
        resize(original, original, Size(DISPLAY_W*2, DISPLAY_H), INTER_LINEAR);
        imshow("Original Stereo Pair", original);



        //saving stereo data
        cout << "Saving Calibration" << endl;
        cv::FileStorage cv_file = cv::FileStorage("stereo_rectify_maps.xml", cv::FileStorage::WRITE);
        cv_file.write("Left_Stereo_Map_x",Left_Stereo_Map1);
        cv_file.write("Left_Stereo_Map_y",Left_Stereo_Map2);
        cv_file.write("Right_Stereo_Map_x",Right_Stereo_Map1);
        cv_file.write("Right_Stereo_Map_y",Right_Stereo_Map2);
        cv_file.write("Q",Q);
        cv_file.release();

        cout << endl;
        cout << "Number of Left Chessboards: " << numLeftChess << endl;
        cout << "Number of Right Chessboards: " << numRightChess << endl;
        cout << "Number of Stereo Chessboard Pairs: " << numStereoPair << endl;
        cout << endl;
        cout << "Percentage of Left image: " << (float)numLeftChess/(float)totalStereoPair*100.0 << endl;
        cout << "Percentage of Right image: " << (float)numRightChess/(float)totalStereoPair*100.0 << endl;
        cout << "Percentage of StereoPairs: " << (float)numStereoPair/(float)totalStereoPair*100.0 << endl;
        cout << endl;

        cout << "Press any key to continue." << endl;
        waitKey(0);



}

void testStereoLab() {

        VideoCapture inputVideo(CAMERA_INDEX,CAMERA_API);

        inputVideo.set(CAP_PROP_FRAME_WIDTH, CAMERA_READ_WIDTH);
        inputVideo.set(CAP_PROP_FRAME_HEIGHT, CAMERA_READ_HEIGHT);

        //Make sure camera is connected
        if (!inputVideo.isOpened()) {
                CV_Assert("Cam open failed");
                cout << "camera failed to open" << endl;
                return;
        }

        //test stereo implementations here

        //read in stereo rectification data
        Mat Left_Stereo_Map1, Left_Stereo_Map2;
        Mat Right_Stereo_Map1, Right_Stereo_Map2;
        Mat Q;
        

        cv::FileStorage cv_file2 = cv::FileStorage(LAB_MAP_ADDRESS, cv::FileStorage::READ);
        cv_file2["Left_Stereo_Map_x"] >> Left_Stereo_Map1;
        cv_file2["Left_Stereo_Map_y"] >> Left_Stereo_Map2;
        cv_file2["Right_Stereo_Map_x"] >> Right_Stereo_Map1;
        cv_file2["Right_Stereo_Map_y"] >> Right_Stereo_Map2;
        cv_file2["Q"] >> Q;
        cv_file2.release();


        int preFilterSize = 0;
        int preFilterCap = 30;
        int uniqunessRatio = 0;
        int textureThreshold = 0;
        int speckleRange = 2;
        int speckleWindowSize = 10;
        int disp12Diff = 0;

        int lambda = 178;
        int sigma = 50;
        int vis_mult = 20;

        int blue = 0;
        int green = 0;
        int red = 0;

        int minb = 0;
        int maxb = 255;
        int maxInt = 255;
        int minInt = 0;
        int maxa= 255;
        int mina = 0;

        clock_t last = 0;

        Ptr<StereoBM > left_matcher = StereoBM::create(16, 13);
        
        Ptr<ximgproc::DisparityWLSFilter> wls_filter = ximgproc::createDisparityWLSFilter(left_matcher);
        Ptr<StereoMatcher> right_matcher = ximgproc::createRightMatcher(left_matcher);

	while (true) {
                benchmarkFirst("Start");

                //cout << Q << endl << endl;

                /*
                Ptr< StereoSGBM > left_matcher = StereoSGBM::create(minDisp,                            //min disparities
                                                numDisp,                                 //num disparities
                                                block,                                     //block size
                                                pOne,                                    //P1
                                                pTwo,                                    //P2
                                                disp12MaxDiff,                                      //disp12MaxDiff
                                                preFilterCap,                                     //preFilterCap
                                                uniquness,                                      //uniquenessRatio
                                                speckleWindowSize,                                      //speckleWindowSize
                                                speckleRange,                                     //speckleRange
                                                StereoSGBM::MAVOIDEODE_HH4);
                */
                left_matcher->setPreFilterType(1);
                left_matcher->setPreFilterSize(preFilterSize*2+5);
                left_matcher->setPreFilterCap(preFilterCap+1);
                left_matcher->setUniquenessRatio(uniqunessRatio);
                left_matcher->setTextureThreshold(textureThreshold);
                left_matcher->setSpeckleRange(speckleRange);
                left_matcher->setSpeckleWindowSize(speckleWindowSize);
                left_matcher->setDisp12MaxDiff(disp12Diff);
                left_matcher->setMinDisparity(-8);

                Ptr<StereoMatcher> right_matcher = ximgproc::createRightMatcher(left_matcher);
                
                /*
                if (!CORRECT_COLOR) {
                        
                        cout << preFilterSize << " Pre filter size" << endl;
                        cout << preFilterCap << " pre filter cap" << endl;
                        cout << uniqunessRatio << " uniqunessRatio" << endl;
                        cout << textureThreshold << " textureThreshold" << endl;
                        cout << speckleRange << " speckleRange" << endl;
                        cout << speckleWindowSize << " speckleWindowSize" << endl;
                   Affine3d(Mat::zeros(1,3,CV_32F),Vec3f(0,0,0))     cout << disp12Diff << " disp12Diff" << endl;

                        cout << endl;
                        cout << "Lambda: " << lambda << endl;
                        cout << "Sigma: " << sigma << endl;
                        cout << endl;
                } else {
                        cout << "Min H: " << minInt <<endl;
                        cout << "Min S: " << mina <<endl;
                        cout << "Min V: " << minb <<endl<<endl;

                        cout << "Max H: " << maxInt <<endl;      
                        cout << "Max S: " << maxa <<endl;
                        cout << "Max V: " << maxb <<endl;

                        cout << "Blue: " << blue <<endl;
                        cout << "Green: " << green <<endl;
                        cout << "Red: " << red <<endl;
                }*/

                Mat frame;

                inputVideo >> frame;

                if (frame.rows <= 0 || frame.cols <= 0) {
                        continue;
                }

                benchmark("Start getting frames");
                //split frame into two images
                Mat imgL, imgR;
               
                Rect left_roi(0, 0, frame.cols/2, frame.rows);
                Rect right_roi (frame.cols/2, 0, frame.cols/2, frame.rows);

                Mat crop_left(frame, left_roi);
                Mat crop_right (frame, right_roi);

                crop_left.copyTo(imgL);
                crop_right.copyTo(imgR);
                
                Mat imgL_rect, imgR_rect;
                resize(imgL, imgL_rect, Size(RECT_WIDTH, RECT_HEIGHT), INTER_LINEAR);
                resize(imgR, imgR_rect, Size(RECT_WIDTH, RECT_HEIGHT), INTER_LINEAR);

                imshow("Raw Left",imgL);
                benchmark("Got frames");

                //undistort images
                Mat left_correct, right_correct;

                remap(imgL_rect,
                        left_correct,
                        Left_Stereo_Map1,
                        Left_Stereo_Map2,
                        INTERPOLATION,
                        BORDER_CONSTANT,
                        0);

                remap(imgR_rect,
                        right_correct,
                        Right_Stereo_Map1,
                        Right_Stereo_Map2,
                        INTERPOLATION,
                        BORDER_CONSTANT,
                        0);
                
                benchmark("Remapped frames");

                Mat left_small_correct;
                Mat right_small_correct;
                
                
                //imshow("right correct", right_correct);
                resize(left_correct, left_small_correct, Size(DISP_WIDTH, DISP_HEIGHT), INTER_LINEAR);
                resize(right_correct, right_small_correct, Size(DISP_WIDTH, DISP_HEIGHT), INTER_LINEAR);
                imshow("left correct", left_correct);
                imshow("left small", left_small_correct);
                imshow("right small", right_small_correct);


                Mat left_correct_sg, right_correct_sg;
                cvtColor(left_small_correct, left_correct_sg, cv::COLOR_BGR2GRAY);
                cvtColor(right_small_correct, right_correct_sg, cv::COLOR_BGR2GRAY);

                benchmark("Pre-compute disparity");
                //compute disparity
                Mat left_disp, right_disp;
                left_matcher-> compute(left_correct_sg, right_correct_sg,left_disp);
                right_matcher->compute(right_correct_sg, left_correct_sg, right_disp);
                benchmark("Computed disparity");

                Mat filtered_disp;
                wls_filter->setLambda(lambda/10.0);
                wls_filter->setSigmaColor(sigma/10.0);
                wls_filter->filter(left_disp,left_correct_sg,filtered_disp,right_disp);
                benchmark("Used WLS-filter");

                Mat filtered_disp_vis, left_disp_vis;
                ximgproc::getDisparityVis(left_disp, left_disp_vis, vis_mult);
                ximgproc::getDisparityVis(filtered_disp,filtered_disp_vis,vis_mult);
                namedWindow("filtered disparity", WINDOW_AUTOSIZE);
                imshow("filtered disparity", filtered_disp_vis);
                imshow("unfiltered disparity", left_disp_vis);

                Mat xyz;
                resize(filtered_disp, filtered_disp, Size(RECT_WIDTH, RECT_HEIGHT), INTER_LINEAR);

                reprojectImageTo3D(filtered_disp, xyz, Q, true, -1);
                cout << "XYZ size " << xyz.size() << "; type = " << type2str(xyz.type()) << endl;
                benchmark("Reprojected");
                //cout << "Q:" << endl;
                //cout << Q << endl << endl;
                Mat XYZ[3];
                Mat x,y,depth;
                split(xyz, XYZ);
                depth = XYZ[2];
                x = XYZ[0];
                y = XYZ[1];

                Mat BGR;
                cvtColor(depth/1500.0,  BGR, cv::COLOR_GRAY2BGR);


                Mat close;
                int dist = 50;
                inRange(depth*0.15, Scalar(0), Scalar(dist), close);
                Mat mean1, stddev1;
                cv::meanStdDev(depth*0.15, mean1, stddev1, close);
                float tz = mean1.at<double>(0,0);
                //cout << "TZ: " << tz << endl;
                if(!render3DProjection){
                        imshow("close", close);
                }

                //object avoidence
                Mat objectMask;
                inRange(depth*0.15, Scalar(0), Scalar(dist), objectMask);
                Mat avoidence;
                left_correct.copyTo(avoidence, objectMask);
                if(!render3DProjection){
                        imshow("Avoid", avoidence);
                }

                vector<vector<Point> > contoursA;
                vector<Vec4i> hierarchyA;

                findContours(objectMask, contoursA, hierarchyA, RETR_TREE, CHAIN_APPROX_SIMPLE);

                bool avoid = false;

                vector<Point> largestContour;
                float largestArea = 0;

                for (unsigned int i = 0; i < contoursA.size(); i++) {
                        //perfom initial area filter
                        double area = contourArea(contoursA[i]);
                        if (hierarchyA[i][3] == -1 && (area > AVOIDENCE_SIZE)) {
                        
                                if (area > largestArea) {
                                        largestArea = area;
                                        largestContour = contoursA[i];
                                }
                        }
                }

                int quad = 10;

                if (largestArea > 0) {
                        //cout << "Avoid Area: " << largestArea << endl;
                                        
                        Moments moment = moments(largestContour);
                        double cx = moment.m10 / moment.m00; //int(M['m10']/M['m00'])
                        double cy = moment.m01 / moment.m00; //int(M['m01']/M['m00'])

                        //cout << "XX: " << cx/(float)RECT_WIDTH << endl;
                        //cout << "YY: " << cy/(float)RECT_HEIGHT << endl;
                        
                        int xx = (int)((cx/((float)RECT_WIDTH))*3.0)+1;
                        int yy = (int)((cy/((float)RECT_HEIGHT))*3.0)+1;

                        //cout << "xx: " << xx << endl;
                        //cout << "yy: " << yy << endl;

                        quad = 3*(yy-1)+(xx);
                        
                }

                //cout << "Quad: " << quad << endl;

                //Detect Goal
                if (mode == detectGoal) {
                        Mat bMask;

                        Mat imgGhsv;
                        Mat GoalCorrected;
                        Mat GoalCorrectM = Mat::zeros(left_correct.size(), left_correct.type());
                        if (CORRECT_COLOR) {
                                GoalCorrectM.setTo(Scalar(blue,green,red));
                        } else {
                                GoalCorrectM.setTo(G_CORRECTION);
                        }
                        add(left_correct,GoalCorrectM, GoalCorrected);

                        cvtColor(GoalCorrected, imgGhsv, cv::COLOR_BGR2HSV);

                        if (CORRECT_COLOR) {
                                inRange(imgGhsv, Scalar(minInt,mina,minb), Scalar(maxInt,maxa,maxb), bMask);
                        } else {
                                inRange(imgGhsv, G_MIN, G_MAX, bMask);
                        }

                        Mat HSV[3];
                        split(imgGhsv, HSV);
                        HSV[1].setTo(255);
                        HSV[2].setTo(255);
                        vector<Mat> channels;
                        for (int i = 0; i<3; i++) {
                                channels.push_back(HSV[i]);
                        }
                        Mat onlyH;
                        merge(channels, onlyH);
                        cvtColor(onlyH, onlyH, COLOR_HSV2BGR);
                        if(!render3DProjection){
                                imshow("onlyH", onlyH);
                        }

                        Mat erosionElem = getStructuringElement(MORPH_ELLIPSE, Size(2*E_SIZE+1,2*E_SIZE+1),Point(E_SIZE, E_SIZE));
                        Mat dilationElem = getStructuringElement(MORPH_ELLIPSE, Size(2*D_SIZE+1,2*D_SIZE+1),Point(D_SIZE, D_SIZE));

                        erode(bMask, bMask, erosionElem, Point(-1,-1), 0);
                        dilate(bMask, bMask, dilationElem, Point(-1,-1), D_ITER+GOAL_DILATION_ADJUST);

                        if(!render3DProjection){
                                imshow("Goal Mask", bMask);
                        }

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
                                                Mat innerSingleThin, goalSingleThin;
                                                Mat innerSingleBit, goalSingleBit;

                                                Mat inner = Mat::zeros(bMask.rows, bMask.cols, CV_8UC3);
                                                Mat goal = Mat::zeros(bMask.rows, bMask.cols, CV_8UC3);
                                                Mat innerThin = Mat::zeros(bMask.rows, bMask.cols, CV_8UC3);
                                                Mat goalThin = Mat::zeros(bMask.rows, bMask.cols, CV_8UC3);

                                                drawContours(inner, outScaled, 0, Scalar(255, 255, 255), FILLED, 8);
                                                drawContours(goal, contours, i, Scalar(255, 255, 255), FILLED, 8, hierarchy);

                                                //get thin goal
                                                vector<vector<Point> > thinOutScaled;
                                                vector<vector<Point> > thinGoal;
                                                thinOutScaled.push_back(scaleContour(outScaled[0], 1.15));
                                                thinGoal.push_back(scaleContour(contours[i], 0.85));

                                                drawContours(innerThin, thinOutScaled, 0, Scalar(255, 255, 255), FILLED, 8);
                                                drawContours(goalThin, thinGoal, 0, Scalar(255, 255, 255), FILLED, 8);

                                                cvtColor(inner, innerSingle, COLOR_BGR2GRAY);
                                                cvtColor(goal, goalSingle, COLOR_BGR2GRAY);
                                                cvtColor(innerThin, innerSingleThin, COLOR_BGR2GRAY);
                                                cvtColor(goalThin, goalSingleThin, COLOR_BGR2GRAY);

                                                //get just the goal detected
                                                bitwise_xor(goalSingle, goalSingle, goalSingle, innerSingle);
                                                bitwise_xor(goalSingleThin, goalSingleThin, goalSingleThin, innerSingleThin);

                                                //set as optical flow mask
                                                tempSingle = goalSingle;
                                                //bitwise_or(debug, tempSingle, debug);
                                                

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

                                                if(!render3DProjection){
                                                        imshow("goal", innerSingle);
                                                }

                                                //cout << "inner total" << areaI << endl;
                                                //cout << "inner detected" << innerDetect << endl;
                                                //cout << "goal total" << areaG << endl;
                                                //cout << "goal detected" << goalDetect<< endl;

                                                Mat depthSingle;
                                                bitwise_and(goalSingleThin, close, depthSingle);

                                                float depthConversion = 0.15;
                                                Mat mean, stddev;
                                                cv::meanStdDev(depth, mean, stddev, depthSingle);
                                                float distance = mean.at<double>(0,0)*depthConversion;

                                                float confidence = 1-innerDetect;
                                                if (confidence > GOAL_CONFIDENCE) {
                                                        //cout << "condfidence:" << confidence << endl;
                                                        cout << area << endl;
                                                        cout << "Distance: " << distance << endl << endl;
                                                        //add goal data to info
                                                }
                                                cout << "Goal dectect: " << countNonZero(goalSingleBit) << endl;

                                                if (countNonZero(goalSingleBit)  <= GOAL_CONFIRM) {
                                                        distance = 1000;
                                                }


                                                if (confidence > GOAL_CONFIDENCE) {
                                                        //drawContours(BGR, contours, i, Scalar(255,0,0), LINE_4, 8, hierarchy);

                                                        //do approx poly to close contours and smooth them
                                                        double epsilon = G_POLY_APPROX_E*cv::arcLength(contours[i],true);
                                                        cv::approxPolyDP(contours[i], approx, epsilon,true);
                                                        outApprox.push_back(approx);
                                                        //scale contour
                                                        outScaled.push_back(scaleContour(approx, GOAL_INNER_CONTOUR_SCALE));
                                                        //imshow("Goal", goalSingleThin);
                                                }

                                                for (unsigned int i = 0; i < outApprox.size(); i++) {
                                                        if (confidence > GOAL_CONFIDENCE) {
                                                                drawContours(BGR, outApprox, i, Scalar(0,255,0), LINE_4, 8);
                                                                drawContours(BGR, outScaled, i, Scalar(0,0,255), LINE_4, 8);
                                                        }
                                                }
                                                if(!render3DProjection){
                                                        imshow("Targets", BGR);
                                                }

                                        }
                                }
                        }
                } else if (mode == detectBall) {
                        Mat bMask;
                        Mat imgLhsv;

                        Mat balloonCorrected;
                        Mat balloonCorrectM = Mat::zeros(imgL_rect.size(), imgL_rect.type());
                        if (CORRECT_COLOR) {
                                balloonCorrectM.setTo(Scalar(blue, green, red));
                        } else {
                                balloonCorrectM.setTo(B_CORRECTION);
                        }
                        add(imgL_rect,balloonCorrectM, balloonCorrected);

                        cvtColor(balloonCorrected, imgLhsv, cv::COLOR_BGR2HSV);

                        if (CORRECT_COLOR) {
                                inRange(imgLhsv, Scalar(minInt,mina,minb), Scalar(maxInt,maxa,maxb), bMask);
                        } else {
                                inRange(imgLhsv, B_MIN, B_MAX, bMask);
                        }

                        Mat HSV[3];
                        split(imgLhsv, HSV);
                        HSV[1].setTo(255);
                        HSV[2].setTo(255);
                        vector<Mat> channels;
                        for (int i = 0; i<3; i++) {
                                channels.push_back(HSV[i]);
                        }
                        Mat onlyH;
                        merge(channels, onlyH);
                        cvtColor(onlyH, onlyH, COLOR_HSV2BGR);
                        if(!render3DProjection){
                                imshow("onlyH", onlyH);
                        }

                        vector<vector<Point> > contours;
                        vector<Vec4i> hierarchy;
                        if(!render3DProjection){
                                imshow("Ball Mask", bMask);
                        }
                        findContours(bMask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

                        std::vector<vector<float> > balloons;

                        //cout << "Number of contours: " << contours.size() << endl;

                        // iterate through all the top-level contours,
                        // draw each connected component with its own random color
                        for (unsigned int i = 0; i < contours.size(); i++) {
                                //perfom initial area filter
                                double area = contourArea(contours[i]);
                                if (hierarchy[i][3] == -1 && (area > MIN_AREA) && (contours[i].size() > 5)) {

                                        RotatedRect objEllipse = fitEllipse(contours[i]);

                                        Mat temp = Mat::zeros(depth.rows, depth.cols, CV_8UC1);
                                        cout << depth.rows << endl;
                                        cout << depth.cols << endl;

                                        if (objEllipse.size.width/objEllipse.size.height > 1/(float)SIZE_RATIO && objEllipse.size.width/objEllipse.size.height < SIZE_RATIO) {
                                                //compute distance from depth map

                                                vector<Point> smallcnt = scaleContour(contours[i], 1.0);
                                                RotatedRect smallEllipse = fitEllipse(smallcnt);

                                                ellipse(temp, smallEllipse, Scalar(255), -1);
                                                ellipse(BGR, smallEllipse, Scalar(0,255,0), 2);
                                                bitwise_and(temp, close, temp);

                                                Mat mean, stddev;
                                                cv::meanStdDev(depth, mean, stddev, temp);
                                                //cout << type2str(mean.type()) << endl;

                                                double conversion = 0.15;

                                                float x = objEllipse.center.x;
                                                float y = objEllipse.center.y;
                                                float z = mean.at<double>(0,0)*conversion;

                                                if (z < 0 ) {
                                                        z = z * -1;
                                                }

                                                if (z == 0) {
                                                        z = 10000;
                                                }

                                                //cout << "Distance: " << z << endl;

                                                //add to vector
                                                std::vector<float> balloon;
                                                balloon.push_back(x);
                                                balloon.push_back(y);
                                                balloon.push_back(z);
                                                balloon.push_back(area);
                                                balloons.push_back(balloon);

                                                ellipse(imgL_rect, objEllipse, Scalar(0,0,255), 3);
                                                //cout << "Depth of object: " << z << "in\tstd: " << stddev.at<double>(0,0)*conversion << endl;
                                        }
                                }
                        }
                        if(!render3DProjection){
                                imshow("Targets", BGR);
                        }
                } else {

                }
                benchmark("Everything else");
                int c = waitKey(WAIT);
                if (c % 256 == 27) {
                        break;
                } else if (c % 256 == 32) {
                        if (mode == detectBall) {
                                mode = detectGoal;
                        } else {
                                mode = detectBall;
                        }
                }

                //cout << "MODE----------------->" << mode << endl;

                clock_t now = clock();
                float time = (float)(now - last)/(float)CLOCKS_PER_SEC;
                //cout << "Stereo Compute Time: " << time << "s" << endl;
                //cout << "Stereo Compute Rate: " << 1/time << "Hz" << endl; //Inaccurate
                last = now;
                benchmark("Last");
                //benchmarkPrint();
	}
}

//Display camera feed
int main(int argc, char** argv) {

        delay(0.2);
        cout << "Attempting... ";

        //waitKey(1);
        //delay(2);
        //return 0;

        
        //viz::Viz3d myWindow("Coordinate Frame");
        //myWindow.spinOnce();
        //cout << "Done!" << endl;
        //delay(5);
        //myWindow.showWidget("Coordinate Widget", viz::WCoordinateSystem());
        
        /*Mat xyz = Mat::zeros(Size(3,3),CV_32FC3);
        viz::WCloud cloud(xyz);
        myWindow.showWidget("Cloud", cloud);
        myWindow.spinOnce();

        return 0;*/

        VideoCapture inputVideo(2,CAP_V4L);
        
        inputVideo.set(CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
        inputVideo.set(CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);

        //Make sure camera is connected
        if (!inputVideo.isOpened()) {
                CV_Assert("Cam open failed");
                cout << "camera failed to open" << endl;
                return 0;
        }
        /*
        if(!render3DProjection){
                Mat frame;
                inputVideo >> frame;
                namedWindow("Camera Footage", CV_WINDOW_AUTOSIZE);
                resizeWindow("Camera Footage", DISPLAY_W*2, DISPLAY_H);
                imshow("Camera Footage", frame);
        }*/
        Mat mat;
        inputVideo >> mat;

        imshow("Window",mat);    
        inputVideo.release();   

        while (true) {
                //Print menu
                waitKey(10);
                cout << "Select an open window and press key for program input." << endl;
                cout << "Stereo Calibration" << endl;
                cout << "1. Take Chessboard Photos" << endl;
                cout << "2. Undistort Cameras and Calibrate Stereo" << endl;
                cout << "3. Test Stereo Lab" << endl;
                cout << "4. Exit Program" << endl << endl;
                int mode = waitKey(0);
                cout << "Mode: " << mode << endl;
                /*
                if(render3DProjection){
                        mode = 51;
                }else{
                        mode = waitKey(0);
                }*/

                switch(mode) {
                        case 49: //1
                                takeStereoImages();
                        break;
                        case 50: //2
                                undistortCamerasCalibrateStereo();
                        break;
                        case 51: //3
                                testStereoLab();
                        break;
                        case 52: //4
                                cout << "Exiting Program" << endl;
                                // Closes all the frames
                                destroyAllWindows();
                                return 0;
                        default:
                                cout << "Invalid Option: Choose Again" << endl << endl;
                        break;
                }
        }    
}

vector<Point> scaleContour(vector<Point> contour, double scale) {
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

string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

vector<chrono::system_clock::time_point> times;
vector<string> flags;

void benchmarkFirst(string flag){
	times.clear();
	flags.clear();
	times.push_back(chrono::system_clock::now());
	flags.push_back(flag);
}

void benchmark(string flag){
	times.push_back(chrono::system_clock::now());
	flags.push_back(flag);
}

double timeDiff(chrono::system_clock::time_point a, chrono::system_clock::time_point b){
        chrono::duration<double> elapsedTime = a-b; //For some reason, this is required to output as seconds
        return elapsedTime.count();
}

void benchmarkPrint(){
	if(times.size() < 2) return;
        
	double deltaTotal = timeDiff(times[times.size()-1],times[0]);
	for(int i=1; i<times.size(); i++){
                double deltaTime = timeDiff(times[i],times[i-1]);
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