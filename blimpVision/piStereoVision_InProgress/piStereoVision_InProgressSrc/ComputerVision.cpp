// ============================== INCLUDES ==============================
#include <libv4l2.h>
#include <iostream>

#include "ComputerVision.h"

using namespace std;

// ============================== CLASS ==============================

void ComputerVision::init(){
    // Init video capture
    cap.open(CAMERA_INDEX, CAP_V4L);
    if (!cap.isOpened()) {
		CV_Assert("Cam open failed");
	}

	//cap.set(CAP_PROP_FPS, 120);
	cap.set(CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
	cap.set(CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);

    // Init stereo matcher
    left_matcher = StereoBM::create(16, 13); //Num disp, block size
	left_matcher->setPreFilterType(1);
	left_matcher->setPreFilterSize(PRE_FILTER_SIZE);
	left_matcher->setPreFilterCap(PRE_FILTER_CAP);
	left_matcher->setUniquenessRatio(UNIQUENESS_RATIO);

	wls_filter = ximgproc::createDisparityWLSFilter(left_matcher);
	right_matcher = ximgproc::createRightMatcher(left_matcher);

	wls_filter->setLambda(LAMBDA);
	wls_filter->setSigmaColor(SIGMA);
}

void ComputerVision::readCalibrationFiles(){
	cout << "Reading Stereo Camera Parameters" << endl;
	FileStorage cv_file2 = FileStorage(STEREO_CAL_PATH, FileStorage::READ);
	
    cv_file2["Left_Stereo_Map_x"] >> Left_Stereo_Map1;
	cv_file2["Left_Stereo_Map_y"] >> Left_Stereo_Map2;
	cv_file2["Right_Stereo_Map_x"] >> Right_Stereo_Map1;
	cv_file2["Right_Stereo_Map_y"] >> Right_Stereo_Map2;
	cv_file2["Q"] >> Q;
	cv_file2.release();
	cout << "Read Complete" << endl;
}

// Big image processing function
void ComputerVision::update(autoState mode, goalType goalColor){
    //get frame
    Mat frame2;
    cap >> frame2;

    //split frame into two images, L and R
    Mat imgL, imgR;

    Rect left_roi(0, 0, frame2.cols/2, frame2.rows);
    Rect right_roi (frame2.cols/2, 0, frame2.cols/2, frame2.rows);

    Mat crop_left(frame2, left_roi);
    Mat crop_right (frame2, right_roi);

    crop_left.copyTo(imgL);
    crop_right.copyTo(imgR);

    //send left image to base
    //TO BE COMPLETED

    //reduce image size for rectification
    Mat imgL_rect, imgR_rect;
    resize(imgL, imgL_rect, Size(RECT_WIDTH, RECT_HEIGHT), INTER_LINEAR);
    resize(imgR, imgR_rect, Size(RECT_WIDTH, RECT_HEIGHT), INTER_LINEAR);

    //remap images for stereo
    Mat left_correct, right_correct;

    remap(imgL_rect,
            left_correct,
            Left_Stereo_Map1,
            Left_Stereo_Map2,
            INTER_AREA,
            BORDER_CONSTANT,
            0);

    remap(imgR_rect,
            right_correct,
            Right_Stereo_Map1,
            Right_Stereo_Map2,
            INTER_AREA,
            BORDER_CONSTANT,
            0);

    //shrink to disparity size
    Mat left_small_correct, right_small_correct;
    resize(left_correct, left_small_correct, Size(DISP_WIDTH, DISP_HEIGHT), INTER_LINEAR);
    resize(right_correct, right_small_correct, Size(DISP_WIDTH, DISP_HEIGHT), INTER_LINEAR);

    Mat left_correct_sg, right_correct_sg;
    cvtColor(left_small_correct, left_correct_sg, cv::COLOR_BGR2GRAY);
    cvtColor(right_small_correct, right_correct_sg, cv::COLOR_BGR2GRAY);

    //compute stereo and filter
    Mat left_disp, right_disp;
    left_matcher->compute(left_correct_sg, right_correct_sg, left_disp);
    right_matcher->compute(right_correct_sg, left_correct_sg, right_disp);

    Mat filtered_disp;
    wls_filter->filter(left_disp, left_correct_sg, filtered_disp, right_disp);

    Mat xyz;
    reprojectImageTo3D(filtered_disp, xyz, Q, true, -1);
    Mat XYZ[3];
    Mat x, y, z;
    split(xyz, XYZ);
    z = XYZ[2];

    resize(z, z, Size(RECT_WIDTH, RECT_HEIGHT), INTER_LINEAR);

    //infinite value removal
    Mat close;
    inRange(z*0.15, Scalar(0), Scalar(200), close);


    //add object avoidence detection here
    Mat objectMask;
    inRange(z*0.15, Scalar(0), Scalar(AVOID_DIST),  objectMask);


    vector<vector<Point> > contoursA;
    vector<Vec4i> heirarchyA;

    findContours(objectMask, contoursA, heirarchyA, RETR_TREE, CHAIN_APPROX_SIMPLE);

    vector<Point> largestContour;
    float largestArea = 0;

    for (unsigned int i = 0; i < contoursA.size(); i++) {
        double area = contourArea(contoursA[i]);
        if (heirarchyA[i][3] == -1 && (area > AVOID_AREA)) {

            if (area > largestArea) {
                largestArea = area;
                largestContour = contoursA[i];
            }
        }
    }

    quad = 10;

    if (largestArea > 0) {
        Moments moment = moments(largestContour);
        float cx = moment.m10/moment.m00;
        float cy = moment.m01/moment.m00;

        int xx = (int)((cx/((float)RECT_WIDTH))*3.0)+1;
        int yy = (int)((cy/((float)RECT_HEIGHT))*3.0)+1;

        quad = 3*(yy-1)+xx;
    }

    //cout << "Area: " << largestArea << endl;
    cout << "Quad: " << quad << endl;
    cout << "Targeting Goal: " << goalColor << endl;
    //plotUDP("Quad",quad);
    
    /*
    //recording frames-----------------------------------------
    Mat tempRecord;
    cvtColor(temp, tempRecord, COLOR_GRAY2BGR);
    //cout << "TempNewType: " << tempRecord.type() << ", NewSize: " << tempRecord.size() << endl;
    cvtColor(bMask, tempRecord, COLOR_GRAY2BGR);
    //cout << "BMaskNewType: " << tempRecord.type() << ", NewSize: " << tempRecord.size() << endl;
    //drawContours(tempRecord, contours, i, Scalar(125), 4, 8, hierarchy);
    //cout << "GoodType: " << imgL_rect.type() << ", GoodSize: " << imgL_rect.size() << endl;
    //cout << "NewType: " << tempRecord.type() << ", NewSize: " << tempRecord.size() << endl;

    
    tempRecord = 0.5*tempRecord + 0.5*left_correct;
    */
    /*
    if(framesLeftToRecord > 0){
        framesLeftToRecord--;
        saveToVideo(left_correct);
        cout << "Recording video." << endl;
    }*/


    if (mode == searching || mode == approach || mode == catching) {
        //perform colorvision
        Mat bMask;
        Mat imgLhsv;

        Mat balloonCorrected;
        Mat balloonCorrectM = Mat::zeros(left_correct.size(), left_correct.type());
        balloonCorrectM.setTo(B_CORRECTION);
        add(left_correct,balloonCorrectM, balloonCorrected);

        cvtColor(balloonCorrected, imgLhsv, cv::COLOR_BGR2HSV);

        inRange(imgLhsv, B_MIN, B_MAX, bMask);

        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;

        findContours(bMask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        //cout << "Number of contours: " << contours.size() << endl;

        // iterate through all the top-level contours,
        // draw each connected component with its own random color
        for (unsigned int i = 0; i < contours.size(); i++) {
                //perfom initial area filter
                double area = contourArea(contours[i]);
                if (hierarchy[i][3] == -1 && (area > MIN_AREA) && (contours[i].size() > 5)) {

                        RotatedRect objEllipse = fitEllipse(contours[i]);

                        Mat temp = Mat::zeros(z.rows, z.cols, CV_8UC1);

                        if (objEllipse.size.width/objEllipse.size.height > 1/(float)SIZE_RATIO && objEllipse.size.width/objEllipse.size.height < SIZE_RATIO) {
                                //compute distance from depth map

                                vector<Point> smallcnt = scaleContour(contours[i], 1.0);
                                RotatedRect smallEllipse = fitEllipse(smallcnt);

                                ellipse(temp, smallEllipse, Scalar(255), -1);
                                bitwise_and(temp, close, temp);

                                
                                Mat mean, stddev;
                                cv::meanStdDev(z*CONVERSION, mean, stddev, temp);

                                float x = objEllipse.center.x;
                                float y = objEllipse.center.y;
                                float pz = mean.at<double>(0,0);
                                //cout << "BallDistance: " << pz << endl;


                                if (pz < 0) {
                                    pz = pz*-1;
                                }

                                if (pz == 0) {
                                    pz = 10000;
                                }

                                if (pz > 10000) {
                                    pz = 10000;
                                }

                                //add to vector
                                std::vector<float> balloon;
                                balloon.push_back(x);
                                balloon.push_back(y);
                                balloon.push_back(pz);
                                balloon.push_back(area);
                                balloons.push_back(balloon);


                                //cout << "Depth of object: " << pz << "in\tstd: " << stddev.at<double>(0,0)*conversion << endl;
                        }
                }
        }
    } else if (mode == goalSearch || mode == approachGoal || mode == scoringStart) {
        Mat bMask;
        Mat imgLhsv;

        Mat imgGhsv;
        Mat GoalCorrected;
        Mat GoalCorrectM = Mat::zeros(left_correct.size(), left_correct.type());

        if (goalColor == orange) {
            GoalCorrectM.setTo(ORANGE_G_CORRECTION);
        } else {
            GoalCorrectM.setTo(YELLOW_G_CORRECTION);
        }

        add(left_correct,GoalCorrectM, GoalCorrected);

        cvtColor(GoalCorrected, imgGhsv, cv::COLOR_BGR2HSV);

        if (goalColor == orange) {
            inRange(imgGhsv, ORANGE_G_MIN, ORANGE_G_MAX, bMask);
        } else {
            inRange(imgGhsv, YELLOW_G_MIN, YELLOW_G_MAX, bMask);
        }

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
                                Mat innerSingleBit, goalSingleBit;
                                Mat inner = Mat::zeros(bMask.rows, bMask.cols, CV_8UC3);
                                Mat goal = Mat::zeros(bMask.rows, bMask.cols, CV_8UC3);

                                drawContours(inner, outScaled, 0, Scalar(255, 255, 255), FILLED, 8);
                                drawContours(goal, contours, i, Scalar(255, 255, 255), FILLED, 8, hierarchy);

                                cvtColor(inner, innerSingle, COLOR_BGR2GRAY);
                                cvtColor(goal, goalSingle, COLOR_BGR2GRAY);

                                //get just the goal detected
                                bitwise_xor(goalSingle, goalSingle, goalSingle, innerSingle);

                                //set as optical flow mask
                                tempSingle = goalSingle;
                                //bitwise_or(debug, tempSingle, debug);

                                //imshow("goal", goalSingle);

                                Mat debug = Mat::zeros(bMask.rows, bMask.cols, CV_8UC3);

                                for (unsigned int i = 0; i < contours.size(); i++) {
                                        if (hierarchy[i][3] == -1) {
                                                drawContours(debug, contours, i, Scalar(255,0,0), LINE_4, 8, hierarchy);

                                                //do approx poly to close contours and smooth them
                                                double epsilon = G_POLY_APPROX_E*cv::arcLength(contours[i],true);
                                                cv::approxPolyDP(contours[i], approx, epsilon,true);
                                                outApprox.push_back(approx);
                                                //scale contour
                                                outScaled.push_back(scaleContour(approx, GOAL_INNER_CONTOUR_SCALE));
                                        }
                                }

                                for (unsigned int i = 0; i < outApprox.size(); i++) {
                                        drawContours(debug, outApprox, i, Scalar(0,255,0), LINE_4, 8);
                                        drawContours(debug, outScaled, i, Scalar(0,0,255), LINE_4, 8);
                                }

                                //imshow("orange goal contours", debug);

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
                                //cout << "goal detected" << goalDetect<< endl;temp
                                
                                Mat depthMask;
                                
                                bitwise_and(tempSingle, close, depthMask);

                                float confidence = 1-innerDetect;
                                if (confidence > 0.5) {
                                        cout << "condfidence:" << confidence << endl;
                                        std::vector<float> goal;

                                        Point2f center;
                                        float radius;
                                        minEnclosingCircle(contours[i], center, radius);
                                        //cout <<"Crash 2" << endl;
                                        Mat mean, stddev;
                                        cv::meanStdDev(z, mean, stddev, depthMask);



                                        //cout << "Crash 3" << endl;
                                        float pz = (float)mean.at<double>(0,0)*CONVERSION;

                                        if (countNonZero(depthMask) < GOAL_CONFIRM) {
                                            pz = 10000;
                                        }

                                        if (pz < 0) {
                                            pz = pz*-1;
                                        }

                                        if (pz == 0) {
                                            pz = 10000;
                                        }
                                        //cout << "Goal Distance: " << pz << endl;

                                        goal.push_back(center.x);
                                        goal.push_back(center.y);
                                        goal.push_back(pz);
                                        goal.push_back(area);
                                        goals.push_back(goal);
                                }

                        }
                }
        }
    }
}

vector<vector<float>> ComputerVision::getTargetBalloon(){
    std::vector<std::vector<float> > target;
    //send back balloon data
    float area = 0;
    int index = -1;
    for (int i = 0; i < balloons.size(); i++) {
        if (area < balloons[i][3]) {
            area = balloons[i][3];
            index = i;
        }
    }

    if (index != -1) {
        target.push_back(balloons[index]);
    }
    return target;
}

vector<vector<float>> ComputerVision::getTargetGoal(){
    std::vector<std::vector<float> > target;
    //send back goal data
    float area = 0;
    int index = -1;
    for (int i = 0; i < goals.size(); i++) {
        if (area < goals[i][3]) {
            area = goals[i][3];
            index = i;
        }
    }

    if (index != -1) {
        target.push_back(goals[index]);
    }
    return target;
}
    
int ComputerVision::getQuad(){
    return quad;
}

vector<Point> ComputerVision::scaleContour(vector<Point> contour, float scale) {
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