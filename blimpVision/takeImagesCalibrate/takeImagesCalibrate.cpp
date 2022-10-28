#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/ximgproc.hpp"

#include <iostream>
#include <vector>
#include <string>
#include <ctime>

//<-----------Constants------------->
#define CAMERA_INDEX			2
#define CAMERA_WIDTH            2560   //px
#define CAMERA_HEIGHT           960    //px

#define CAMERA_READ_WIDTH       1280
#define CAMERA_READ_HEIGHT      960

#define RECT_WIDTH              320
#define RECT_HEIGHT             240

#define DISP_WIDTH              80
#define DISP_HEIGHT             60

#define LAB_MAP_ADDRESS         "stereo_rectify_maps.xml"
#define HOME_MAP_ADDRESS        "stereo_rectify_maps.xml"

#define INTERPOLATION           INTER_AREA

#define CAMERA_FLAGS            cv::CALIB_USE_INTRINSIC_GUESS | cv::CALIB_RATIONAL_MODEL | cv::CALIB_THIN_PRISM_MODEL
#define STEREO_FLAGS            cv::CALIB_USE_INTRINSIC_GUESS | cv::CALIB_RATIONAL_MODEL | cv::CALIB_THIN_PRISM_MODEL
#define RECTIFY_FLAGS           cv::CALIB_ZERO_DISPARITY

#define ALPHA_CAMERA            0.0
#define ALPHA_STEREO            0.0

#define WAIT                    250

#define BASELINE                50.0      //mm
#define FOCAL                   650.0     //mm

#define LAMBDA                  0.5
#define SIGMA                   0.5
#define VIS_MULT                255.0

#define CHECKERBOARD_SIZE_WIDTH         30.0*0.9
#define CHECKERBOARD_SIZE_HEIGHT        30.0*0.9 

#define PATHL                   "takeImagesCalibrate/tempImages/imgL"
#define PATHR                   "takeImagesCalibrate/tempImages/imgR"
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

enum modes {
        detectBall,
        detectGoal
};

int mode = detectBall;

int CHECKERBOARD[2]{13,9};

void takeStereoImages(bool autoCap = false) {
	VideoCapture inputVideo(CAMERA_INDEX);

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

                imshow("Camera Footage", frame);
                

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

                imshow("Left Frame", imgL);
                imshow("Right Frame", imgR);

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
                        imshow("Left Frame", imgL);

                        

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
                        imshow("Right Frame", imgR);

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

        cout << "Final Rigth Camera Calibration" << endl;
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
        imshow("Corrected Stereo Pair", stereoImages);
        
        Mat original;
        cv::hconcat(imgL_rect, imgR_rect, original);
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

        waitKey(0);



}

void testStereoLab() {

        VideoCapture inputVideo(CAMERA_INDEX);

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


        int disp = 2;

        int pOne = 4;
        int pTwo = 32;
        int disp12MaxDiff = 0;
        int preFilterCap = 0;
        int uniquness = 5;
        int speckleWindowSize = 0;
        int speckleRange = 0;
        int block = 5;

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

        namedWindow("Track");
        if (!CORRECT_COLOR) {
                createTrackbar("lambda", "Track", &lambda, 100);
                createTrackbar("sigma", "Track", &sigma, 100);
                createTrackbar("vis mult", "Track", &vis_mult, 100);

                createTrackbar("P1", "Track", &pOne, 1000);
                createTrackbar("P2", "Track", &pTwo, 2000);
                createTrackbar("disp12MaxDiff", "Track", &disp12MaxDiff, 200);
                createTrackbar("preFilterCap", "Track", &preFilterCap, 300);
                createTrackbar("uniquness", "Track", &uniquness, 100);
                createTrackbar("speckleWindowSize", "Track", &speckleWindowSize, 250);
                createTrackbar("speckleRange", "Track", &speckleRange, 5);
                createTrackbar("disp", "Track", &disp, 10);
                createTrackbar("block", "Track", &block, 75);
        } else {
                createTrackbar("blue", "Track", &blue, 100);
                createTrackbar("green", "Track", &green, 100);
                createTrackbar("red", "Track", &red, 100);

                createTrackbar("Min H", "Track", &minInt, 255);
                createTrackbar("Max H", "Track", &maxInt, 255);
                createTrackbar("Min S", "Track", &mina, 255);
                createTrackbar("Max S", "Track", &maxa, 255);
                createTrackbar("Min V", "Track", &minb, 255);
                createTrackbar("Max V", "Track", &maxb, 255);
        }
        

        clock_t last = 0;

        

	while (true) {
                int minDisp = -8*disp;
                int numDisp = 16*disp;

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
                                                StereoSGBM::MODE_HH4);
                Ptr<ximgproc::DisparityWLSFilter> wls_filter = ximgproc::createDisparityWLSFilter(left_matcher);
                Ptr<StereoMatcher> right_matcher = ximgproc::createRightMatcher(left_matcher);

                if (!CORRECT_COLOR) {
                        cout << "P1 " << pOne << endl;
                        cout << pTwo << endl;
                        cout << disp12MaxDiff << endl;
                        cout << preFilterCap << endl;
                        cout << uniquness << endl;
                        cout << speckleWindowSize << endl;
                        cout << "speckleRange " << speckleRange << endl;
                        cout << "disp" << disp << endl;
                        cout << "block" << block << endl;
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
                }

                Mat frame;

                inputVideo >> frame;

                if (frame.rows <= 0 || frame.cols <= 0) {
                        continue;
                }

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

                //imshow("Correct Right", right_correct);
                //imshow("Correct Left", left_correct);
                
                Mat left_small_correct;
                Mat right_small_correct;
                resize(left_correct, left_small_correct, Size(DISP_WIDTH, DISP_HEIGHT), INTER_LINEAR);
                resize(right_correct, right_small_correct, Size(DISP_WIDTH, DISP_HEIGHT), INTER_LINEAR);

                Mat left_correct_sg, right_correct_sg;
                cvtColor(left_small_correct, left_correct_sg, cv::COLOR_BGR2GRAY);
                cvtColor(right_small_correct, right_correct_sg, cv::COLOR_BGR2GRAY);

                //compute disparity
                Mat left_disp, right_disp;
                left_matcher-> compute(left_correct_sg, right_correct_sg,left_disp);
                right_matcher->compute(right_correct_sg, left_correct_sg, right_disp);

                Mat filtered_disp;
                wls_filter->setLambda(17.8);
                wls_filter->setSigmaColor(5.0);
                wls_filter->filter(left_disp,left_correct_sg,filtered_disp,right_disp);

                Mat filtered_disp_vis, left_disp_vis;
                ximgproc::getDisparityVis(left_disp, left_disp_vis, vis_mult);
                ximgproc::getDisparityVis(filtered_disp,filtered_disp_vis,vis_mult);
                namedWindow("filtered disparity", WINDOW_AUTOSIZE);
                imshow("filtered disparity", filtered_disp_vis);
                imshow("unfiltered disparity", left_disp_vis);

                Mat xyz;
                resize(filtered_disp, filtered_disp, Size(RECT_WIDTH, RECT_HEIGHT), INTER_LINEAR);

                reprojectImageTo3D(filtered_disp, xyz, Q, true, -1);
                cout << "Q:" << endl;
                cout << Q << endl << endl;
                Mat XYZ[3];
                Mat x,y,depth;
		split(xyz, XYZ);
		depth = XYZ[2];
                x = XYZ[0];
                y = XYZ[1];

                Mat close;
                inRange(depth*0.15, Scalar(0), Scalar(30), close);
                Mat mean1, stddev1;
                cv::meanStdDev(depth*0.15, mean1, stddev1, close);
                float tz = mean1.at<double>(0,0);
                cout << "TZ: " << tz << endl;
                imshow("close", close);

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
                        imshow("onlyH", onlyH);

                        Mat erosionElem = getStructuringElement(MORPH_ELLIPSE, Size(2*E_SIZE+1,2*E_SIZE+1),Point(E_SIZE, E_SIZE));
                        Mat dilationElem = getStructuringElement(MORPH_ELLIPSE, Size(2*D_SIZE+1,2*D_SIZE+1),Point(D_SIZE, D_SIZE));

                        erode(bMask, bMask, erosionElem, Point(-1,-1), 0);
                        dilate(bMask, bMask, dilationElem, Point(-1,-1), D_ITER+GOAL_DILATION_ADJUST);

                        imshow("Goal Mask", bMask);

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

                                                float depthConversion = 16.923;
                                                Mat mean, stddev;
                                                cv::meanStdDev(depth, mean, stddev, goalSingleThin);
                                                float distance = mean.at<double>(0,0)*depthConversion;

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
                                                //cout << "goal detected" << goalDetect<< endl;

                                                float confidence = 1-innerDetect;
                                                if (confidence > GOAL_CONFIDENCE) {
                                                        cout << "condfidence:" << confidence << endl;
                                                        cout << area << endl;
                                                        cout << "Distance: " << distance << endl << endl;
                                                        //add goal data to info
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
                                                                //drawContours(BGR, outApprox, i, Scalar(0,255,0), LINE_4, 8);
                                                                //drawContours(BGR, outScaled, i, Scalar(0,0,255), LINE_4, 8);
                                                        }
                                                }

                                                //imshow("Targets", BGR);

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
                        imshow("onlyH", onlyH);

                        vector<vector<Point> > contours;
                        vector<Vec4i> hierarchy;

                        imshow("Ball Mask", bMask);

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
                                                //ellipse(BGR, smallEllipse, Scalar(0,255,0), 3);

                                                Mat mean, stddev;
                                                cv::meanStdDev(depth, mean, stddev, temp);
                                                cout << type2str(mean.type()) << endl;

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

                                                cout << "Distance: " << z << endl;

                                                //add to vector
                                                std::vector<float> balloon;
                                                balloon.push_back(x);
                                                balloon.push_back(y);
                                                balloon.push_back(z);
                                                balloon.push_back(area);
                                                balloons.push_back(balloon);

                                                ellipse(imgL_rect, objEllipse, Scalar(0,0,255), 3);
                                                cout << "Depth of object: " << z << "in\tstd: " << stddev.at<double>(0,0)*conversion << endl;
                                        }
                                }
                        }
                        //imshow("Targets", BGR);
                } else {

                }

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

                cout << "MODE----------------->" << mode << endl;

                clock_t now = clock();
                float time = (float)(now - last)/(float)CLOCKS_PER_SEC;
                cout << "Stereo Compute Time: " << time << "s" << endl;
                cout << "Stereo compute rate: " << 1/time << "Hz" << endl;
                last = now;
	}
}


void testStereoHOME() {

        VideoCapture inputVideo(CAMERA_INDEX);

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
        

        cv::FileStorage cv_file2 = cv::FileStorage(HOME_MAP_ADDRESS, cv::FileStorage::READ);
        cv_file2["Left_Stereo_Map_x"] >> Left_Stereo_Map1;
        cv_file2["Left_Stereo_Map_y"] >> Left_Stereo_Map2;
        cv_file2["Right_Stereo_Map_x"] >> Right_Stereo_Map1;
        cv_file2["Right_Stereo_Map_y"] >> Right_Stereo_Map2;
        cv_file2["Q"] >> Q;
        cv_file2.release();


        int disp = 2;

        int pOne = 8;
        int pTwo = 64;
        int disp12MaxDiff = 100;
        int preFilterCap = 0;
        int uniquness = 25;
        int speckleWindowSize = 0;
        int speckleRange = 0;
        int block = 1;

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

        namedWindow("Track");
        if (!CORRECT_COLOR) {
                createTrackbar("lambda", "Track", &lambda, 100);
                createTrackbar("sigma", "Track", &sigma, 100);
                createTrackbar("vis mult", "Track", &vis_mult, 100);

                createTrackbar("P1", "Track", &pOne, 1000);
                createTrackbar("P2", "Track", &pTwo, 2000);
                createTrackbar("disp12MaxDiff", "Track", &disp12MaxDiff, 200);
                createTrackbar("preFilterCap", "Track", &preFilterCap, 300);
                createTrackbar("uniquness", "Track", &uniquness, 100);
                createTrackbar("speckleWindowSize", "Track", &speckleWindowSize, 250);
                createTrackbar("speckleRange", "Track", &speckleRange, 5);
                createTrackbar("disp", "Track", &disp, 10);
                createTrackbar("block", "Track", &block, 75);
        } else {
                createTrackbar("blue", "Track", &blue, 100);
                createTrackbar("green", "Track", &green, 100);
                createTrackbar("red", "Track", &red, 100);

                createTrackbar("Min H", "Track", &minInt, 255);
                createTrackbar("Max H", "Track", &maxInt, 255);
                createTrackbar("Min S", "Track", &mina, 255);
                createTrackbar("Max S", "Track", &maxa, 255);
                createTrackbar("Min V", "Track", &minb, 255);
                createTrackbar("Max V", "Track", &maxb, 255);
        }
        

        clock_t last = 0;

        

	while (true) {
                int minDisp = -8*disp;
                int numDisp = 16*disp;

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
                                                StereoSGBM::MODE_HH4);
                Ptr<ximgproc::DisparityWLSFilter> wls_filter = ximgproc::createDisparityWLSFilter(left_matcher);
                Ptr<StereoMatcher> right_matcher = ximgproc::createRightMatcher(left_matcher);

                if (!CORRECT_COLOR) {
                        cout << "P1 " << pOne << endl;
                        cout << pTwo << endl;
                        cout << disp12MaxDiff << endl;
                        cout << preFilterCap << endl;
                        cout << uniquness << endl;
                        cout << speckleWindowSize << endl;
                        cout << "speckleRange " << speckleRange << endl;
                        cout << "disp" << disp << endl;
                        cout << "block" << block << endl;
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
                }

                Mat frame;

                inputVideo >> frame;

                if (frame.rows <= 0 || frame.cols <= 0) {
                        continue;
                }

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

                //imshow("Correct Right", right_correct);
                //imshow("Correct Left", left_correct);
                
                Mat left_small_correct;
                Mat right_small_correct;
                resize(left_correct, left_small_correct, Size(DISP_WIDTH, DISP_HEIGHT), INTER_LINEAR);
                resize(right_correct, right_small_correct, Size(DISP_WIDTH, DISP_HEIGHT), INTER_LINEAR);

                Mat left_correct_sg, right_correct_sg;
                cvtColor(left_small_correct, left_correct_sg, cv::COLOR_BGR2GRAY);
                cvtColor(right_small_correct, right_correct_sg, cv::COLOR_BGR2GRAY);

                //compute disparity
                Mat left_disp, right_disp;
                double matching_time = (double)getTickCount();
                left_matcher-> compute(left_correct_sg, right_correct_sg,left_disp);
                right_matcher->compute(right_correct_sg, left_correct_sg, right_disp);
                matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();
                cout << "Matching Time: " << matching_time << "s" << endl;

                Mat filtered_disp;
                wls_filter->setLambda((float)lambda/10.0);
                wls_filter->setSigmaColor((float)sigma/10.0);
                double filtering_time = (double)getTickCount();
                wls_filter->filter(left_disp,left_correct_sg,filtered_disp,right_disp);
                filtering_time = ((double)getTickCount() - filtering_time)/getTickFrequency();
                cout << "Filtering Time: " << matching_time << "s" << endl;

                cout << lambda/10.0 << endl;
                cout << sigma/10.0 << endl;
                cout << vis_mult << endl;

                Mat filtered_disp_vis, left_disp_vis;
                ximgproc::getDisparityVis(left_disp, left_disp_vis, vis_mult);
                ximgproc::getDisparityVis(filtered_disp,filtered_disp_vis,vis_mult);
                namedWindow("filtered disparity", WINDOW_AUTOSIZE);
                imshow("filtered disparity", filtered_disp_vis);
                imshow("unfiltered disparity", left_disp_vis);

                filtered_disp.convertTo(filtered_disp,CV_32F, 1.0);

                //remove invalid disparities and set them to zero
                Mat zero = Mat::zeros(filtered_disp.size(), CV_32F);
                Mat a = (zero < filtered_disp) & 1;
                Mat b = (zero > filtered_disp) & 1;
                Mat valid;
                addWeighted(a, 1.0, b, 0.0, 0.0, valid, CV_32F);
                filtered_disp = filtered_disp.mul(valid);

                ximgproc::getDisparityVis(filtered_disp,filtered_disp_vis,vis_mult);
                //imshow("Adjusted Disp", filtered_disp_vis);

                //compute depth from disparity (shift is to add a small number to zeros)
                Mat addshift;
                addshift = Mat::zeros(filtered_disp.size(),  CV_32F);
                addshift = addshift+0.01;
                Mat depth2;
                depth2 = (BASELINE*FOCAL*Mat::ones(filtered_disp.size(), CV_32F))/(filtered_disp+addshift);
                resize(depth2, depth2, Size(RECT_WIDTH, RECT_HEIGHT), INTER_LINEAR);
                //imshow("Depth", depth2/3500.0);


                Mat BGR;
                Mat hsv;
                depth2 = depth2/3500.0;
                cvtColor(depth2, BGR, cv::COLOR_GRAY2BGR);

                BGR.convertTo(BGR, CV_8U, 255);

                //hsv only
                Mat XYZc[3];
                split(BGR, XYZc);
                XYZc[1].setTo(255);
                XYZc[2].setTo(255);
                //cout << HSV[0] << endl;
                vector<Mat> channelsxyz;
                for (int i = 0; i<3; i++) {
                        channelsxyz.push_back(XYZc[i]);
                }
                Mat hsvdepth;
                merge(channelsxyz, hsvdepth);
                cvtColor(hsvdepth, hsvdepth, COLOR_HSV2BGR);
                //imshow("color depth", hsvdepth);


                //object avoidence
                Mat objectMask;
                int dist = 25;
                inRange(BGR, Scalar(0, 0, 0), Scalar(dist, dist, dist), objectMask);
                bitwise_not(objectMask, objectMask);
                //imshow("Objects to avoid", objectMask);


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
                        //imshow("onlyH", onlyH);

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

                                                float depthConversion = 16.923;
                                                Mat mean, stddev;
                                                cv::meanStdDev(depth2, mean, stddev, goalSingleThin);
                                                float distance = mean.at<double>(0,0)*depthConversion;

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
                                                //cout << "goal detected" << goalDetect<< endl;

                                                float confidence = 1-innerDetect;
                                                if (confidence > GOAL_CONFIDENCE) {
                                                        cout << "condfidence:" << confidence << endl;
                                                        cout << area << endl;
                                                        cout << "Distance: " << distance << endl << endl;
                                                        //add goal data to info
                                                }


                                                if (confidence > GOAL_CONFIDENCE) {
                                                        drawContours(BGR, contours, i, Scalar(255,0,0), LINE_4, 8, hierarchy);

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

                                                imshow("Targets", BGR);

                                        }
                                }
                        }
                } else if (mode == detectBall) {
                        Mat bMask;
                        Mat imgLhsv;

                        Mat balloonCorrected;
                        Mat balloonCorrectM = Mat::zeros(imgL_rect.size(), imgL_rect.type());
                        balloonCorrectM.setTo(B_CORRECTION);
                        add(imgL_rect,balloonCorrectM, balloonCorrected);

                        cvtColor(balloonCorrected, imgLhsv, cv::COLOR_BGR2HSV);

                        inRange(imgLhsv, B_MIN, B_MAX, bMask);

                        vector<vector<Point> > contours;
                        vector<Vec4i> hierarchy;

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

                                        Mat temp = Mat::zeros(BGR.rows, BGR.cols, CV_8UC1);

                                        if (objEllipse.size.width/objEllipse.size.height > 1/(float)SIZE_RATIO && objEllipse.size.width/objEllipse.size.height < SIZE_RATIO) {
                                                //compute distance from depth map

                                                vector<Point> smallcnt = scaleContour(contours[i], 1.0);
                                                RotatedRect smallEllipse = fitEllipse(smallcnt);

                                                ellipse(temp, smallEllipse, Scalar(255), -1);
                                                ellipse(BGR, smallEllipse, Scalar(0,255,0), 3);

                                                Mat mean, stddev;
                                                cv::meanStdDev(depth2, mean, stddev, temp);

                                                double conversion = 0.246666;

                                                float x = objEllipse.center.x;
                                                float y = objEllipse.center.y;
                                                float z = mean.at<double>(0,0)*conversion;

                                                if (z < 0 ) {
                                                        z = z * -1;
                                                }

                                                if (z == 0) {
                                                        z = 10000;
                                                }

                                                cout << "Distance: " << z << endl;

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
                        imshow("Targets", BGR);
                } else {

                }

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

                cout << "MODE----------------->" << mode << endl;

                clock_t now = clock();
                float time = (float)(now - last)/(float)CLOCKS_PER_SEC;
                cout << "Stereo Compute Time: " << time << "s" << endl;
                cout << "Stereo compute rate: " << 1/time << "Hz" << endl;
                last = now;
	}
}

//Display camera feed
int main(int argc, char** argv) {

        if (argc > 1) {
                //begin automatic calibration
                takeStereoImages(true);
                undistortCamerasCalibrateStereo(true);
                return 0;
        }

        VideoCapture inputVideo(CAMERA_INDEX);

        inputVideo.set(CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
        inputVideo.set(CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);

        //Make sure camera is connected
        if (!inputVideo.isOpened()) {
                CV_Assert("Cam open failed");
                cout << "camera failed to open" << endl;
                return 0;
        }

        Mat frame;
        inputVideo >> frame;
        imshow("Camera Footage", frame);
        inputVideo.release();

        while (true) {
                //Print menu
                waitKey(10);
                cout << "Stereo Calibration" << endl;
                cout << "1. Take Chessboard Photos" << endl;
                cout << "2. Undistort Cameras and Calibrate Stereo" << endl;
                cout << "3. Test Stereo Lab" << endl;
                cout << "4. Test Stereo Home" << endl;
                cout << "5. Exit Program" << endl << endl;

                int mode = waitKey(0);

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
                                testStereoHOME();
                        break;
                        case 53: //5
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
