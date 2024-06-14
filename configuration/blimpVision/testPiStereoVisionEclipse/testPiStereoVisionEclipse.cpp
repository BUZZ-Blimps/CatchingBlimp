#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>

//<-----------Constants------------->
//Camera
#define CAMERA_WIDTH        640
#define CAMERA_HEIGHT       480
#define OPTICAL_FLOW_SCALE  0.2
#define MASK_SCALE          0.4
#define CAMERA_INDEX        0
#define WAIT                250


//FILTERING
#define MIN_AREA                 100
#define B_MIN_CONFIDENCE         0.5
#define G_MIN_CONFIDENCE         0.6
#define G_POLY_APPROX_E          0.001
#define GOAL_INNER_CONTOUR_SCALE 0.7

//COLORS
#define COLOR_SPACE COLOR_BGR2HSV

//balloon
#define B_MIN           Scalar(109,0,0)
#define B_MAX           Scalar(135,255,255)
#define B_Correction    Scalar(56,31,23)

//yellow goal
#define GY_MIN          Scalar(0,0,0)
#define GY_MAX          Scalar(0,0,0)
#define GY_Correction   Scalar(43,0,0)

//orange goal
#define GO_MIN          Scalar(44,16,34)
#define GO_MAX          Scalar(255,191,85)
#define GO_Correction   Scalar(46,46,16)


//blue blimp
#define BB_MIN          Scalar(255,255,255)
#define BB_MAX          Scalar(255,255,255)
#define BB_Correction   Scalar(0,0,0)


//red blimp
#define BR_MIN          Scalar(255,255,255)
#define BR_MAX          Scalar(255,255,255)
#define BR_Correction   Scalar(0,0,0)


//EROSION AND DILATION
#define E_ITER 1
#define D_ITER 7
#define E_SIZE 1
#define D_SIZE 1

#define GOAL_DILATION_ADJUST 1

//GAUSIAN BLUR
#define KERNEL_SIZE     9

//BALLOON CONFIDENCE
#define MIN_GAP_DISTANCE 10

using namespace std;
using namespace cv;

enum object {
    balloon,
    blimpB,
    blimpR,
    goalO,
    goalY,
    first = balloon,
    last = goalY
};

vector<Point> scaleContour(vector<Point> contour, double scale);
vector<vector<double>> getObjects(int type, Mat mask);

//Display camera feed
int main(int argc, char** argv) {

    int minb = 0;
    int maxb = 255;
    int maxInt = 255;
    int minInt = 0;
    int maxa= 255;
    int mina = 0;
    int blueCorrection = 0;
    int redCorrection = 0;
    int greenCorrection = 0;
    int EBSatValue = 0;
    int EBVibValue = 0;

    //adjustment colors
    Scalar adjustMin = Scalar(minInt,mina,minb);
    Scalar adjustMax = Scalar(maxInt,maxa,maxb);

	VideoCapture inputVideo(CAMERA_INDEX);

	//Make sure camera is connected
	if (!inputVideo.isOpened()) {
		CV_Assert("Cam open failed");
	}

	inputVideo.set(CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
	inputVideo.set(CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);

	Size S = Size((int) inputVideo.get(CAP_PROP_FRAME_WIDTH),    // Acquire input size
	              (int) inputVideo.get(CAP_PROP_FRAME_HEIGHT));


    namedWindow("Track");
    createTrackbar("Min H", "Track", &minInt, 255);
    createTrackbar("Max H", "Track", &maxInt, 255);
    createTrackbar("Min S", "Track", &mina, 255);
    createTrackbar("Max S", "Track", &maxa, 255);
    createTrackbar("Min V", "Track", &minb, 255);
    createTrackbar("Max V", "Track", &maxb, 255);
    createTrackbar("BlueCorrection", "Track", &blueCorrection, 255);
    createTrackbar("RedCorrection", "Track", &redCorrection, 255);
    createTrackbar("GreenCorrection", "Track", &greenCorrection, 255);


	while (true) {
        Mat LAB, mask, frame1;
        Mat balloonMask, goalOMask, goalYMask, blimpBMask, blimpRMask;

        //get frame
        Mat frame2, frame3;
        inputVideo >> frame2;
        if (frame2.empty())
            break;

        //resize down
        GaussianBlur(frame2, frame2, Size(KERNEL_SIZE, KERNEL_SIZE), 0);
        //color correction
        Scalar blueCorrect = Scalar(blueCorrection,0,0);
        Scalar redCorrect = Scalar(0,0,redCorrection);
        Scalar greenCorrect = Scalar(0,greenCorrection, 0);

        //color correct frame for better detection
        Mat yellowCorrected;
        Mat yellowCorrectM = Mat::zeros(frame2.size(), frame2.type());
        yellowCorrectM.setTo(blueCorrect);
        add(frame2,yellowCorrectM,yellowCorrected);

        Mat redCorrected;
        Mat redCorrectM = Mat::zeros(frame2.size(), frame2.type());
        redCorrectM.setTo(redCorrect);
        add(yellowCorrected,redCorrectM, redCorrected);

        Mat greenCorrected;
        Mat greenCorrectM = Mat::zeros(frame2.size(), frame2.type());
        greenCorrectM.setTo(greenCorrect);
        add(redCorrected,greenCorrectM, greenCorrected);

        //imshow("color Corrected", greenCorrected);

        Mat balloonCorrected;
        Mat balloonCorrectM = Mat::zeros(frame2.size(), frame2.type());
        balloonCorrectM.setTo(B_Correction);
        add(frame2,balloonCorrectM, balloonCorrected);

        Mat goalYCorrected;
        Mat goalYCorrectM = Mat::zeros(frame2.size(), frame2.type());
        goalYCorrectM.setTo(GY_Correction);
        add(frame2,goalYCorrectM, goalYCorrected);

        Mat goalOCorrected;
        Mat goalOCorrectM = Mat::zeros(frame2.size(), frame2.type());
        goalOCorrectM.setTo(GO_Correction);
        add(frame2,goalOCorrectM, goalOCorrected);

         Mat BBCorrected;
        Mat BBCorrectM = Mat::zeros(frame2.size(), frame2.type());
        BBCorrectM.setTo(BB_Correction);
        add(frame2,BBCorrectM, BBCorrected);

         Mat BRCorrected;
        Mat BRCorrectM = Mat::zeros(frame2.size(), frame2.type());
        BRCorrectM.setTo(BR_Correction);
        add(frame2,goalYCorrectM, BRCorrected);



        //get labs and greyscale for detection
        Mat B_HSV, GY_HSV, GO_HSV, BB_HSV, BR_HSV;
        cvtColor(greenCorrected, LAB, COLOR_SPACE);
        cvtColor(balloonCorrected, B_HSV, COLOR_SPACE);
        cvtColor(goalYCorrected, GY_HSV, COLOR_SPACE);
        cvtColor(goalOCorrected, GO_HSV, COLOR_SPACE);
        cvtColor(BBCorrected, BB_HSV, COLOR_SPACE);
        cvtColor(BRCorrected, BR_HSV, COLOR_SPACE);

        //hsv only
        Mat HSV[3];
        split(LAB, HSV);
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

        //fixed thresholding
        cv::inRange(LAB, Scalar(minInt,mina,minb), Scalar(maxInt,maxa,maxb), mask);
        cv::inRange(B_HSV, B_MIN, B_MAX, balloonMask);
        cv::inRange(GO_HSV, GO_MIN, GO_MAX, goalOMask);
        cv::inRange(GY_HSV, GY_MIN, GY_MAX, goalYMask);
        cv::inRange(BB_HSV, BB_MIN, BB_MAX, blimpBMask);
        cv::inRange(BR_HSV, BR_MIN, BR_MAX, blimpRMask);

        imshow("Mask", mask);

        //imshow("Mask", mask);
        Mat erosionElem = getStructuringElement(MORPH_ELLIPSE, Size(2*E_SIZE+1,2*E_SIZE+1),Point(E_SIZE, E_SIZE));
        Mat dilationElem = getStructuringElement(MORPH_ELLIPSE, Size(2*D_SIZE+1,2*D_SIZE+1),Point(D_SIZE, D_SIZE));

        for (unsigned int i = 0; i < E_ITER; i++) {
            erode(mask, mask, erosionElem, Point(-1,-1), 1);
        }
        for (unsigned int i = 0; i < D_ITER; i++) {
            dilate(mask, mask, dilationElem, Point(-1,-1), 1);
        }

        erode(balloonMask, balloonMask, erosionElem, Point(-1,-1), E_ITER);
        dilate(balloonMask, balloonMask, dilationElem, Point(-1,-1), D_ITER);

        erode(goalYMask, goalYMask, erosionElem, Point(-1,-1), 0);
        dilate(goalYMask, goalYMask, dilationElem, Point(-1,-1), D_ITER+GOAL_DILATION_ADJUST);

        erode(goalOMask, goalOMask, erosionElem, Point(-1,-1), 0);
        dilate(goalOMask, goalOMask, dilationElem, Point(-1,-1), D_ITER+GOAL_DILATION_ADJUST);

        erode(blimpBMask, blimpBMask, erosionElem, Point(-1,-1), E_ITER);
        dilate(blimpBMask, blimpBMask, dilationElem, Point(-1,-1), D_ITER);

        erode(blimpRMask, blimpRMask, erosionElem, Point(-1,-1), E_ITER);
        dilate(blimpRMask, blimpRMask, dilationElem, Point(-1,-1), D_ITER);

        //resize mask for speed
        Mat balloonMasks, goalYMasks, goalOMasks;
        resize(balloonMask, balloonMasks, Size(CAMERA_WIDTH*MASK_SCALE, CAMERA_HEIGHT*MASK_SCALE), INTER_LINEAR);
        resize(goalOMask, goalOMasks, Size(CAMERA_WIDTH*MASK_SCALE, CAMERA_HEIGHT*MASK_SCALE), INTER_LINEAR);
        resize(goalYMask, goalYMasks, Size(CAMERA_WIDTH*MASK_SCALE, CAMERA_HEIGHT*MASK_SCALE), INTER_LINEAR);


        //get data on object

        vector<vector<double>> balloons;
        vector<vector<double>> goalYs;
        vector<vector<double>> goalOs;
        vector<vector<double>> blimpRs;
        vector<vector<double>> blimpBs;

        balloons = getObjects(balloon, balloonMasks);
        goalYs = getObjects(goalY, goalYMasks);
        goalOs = getObjects(goalO, goalOMasks);
        blimpRs = getObjects(blimpR, blimpRMask);
        blimpBs = getObjects(blimpB, blimpBMask);

        cout << "Balloons: " << balloons.size() << endl;
        cout << "Yellow Goals: " << goalYs.size() << endl;
        cout << "Orange Goals: " << goalOs.size() << endl;
        cout << "Red Blimps: " << blimpRs.size() << endl;
        cout << "Blue Blimps: " << blimpBs.size() << endl << endl;

        //visualization

        imshow("Frame", frame2);
        imshow("Adjust Mask", mask);


        //imshow("balloonMask", balloonMask);
        //imshow("goalOMask", goalOMask);
        //imshow("goalYMask", goalYMask);
        //imshow("blimpRMask", blimpRMask);
        //imshow("blimpBMask", blimpBMask);

        //combined masks
        Mat temp;
        bitwise_or(balloonMask, goalYMask, temp);
        bitwise_or(goalOMask, temp, temp);
        bitwise_or(blimpRMask, temp, temp);
        bitwise_or(blimpBMask, temp, temp);

        //imshow("Masks", temp);

        Mat detections = Mat::zeros(balloonMask.rows, balloonMask.cols, CV_8UC3);
        /*
        //print balloon info for debug
        for (unsigned int i = 0; i < balloons.size(); i++) {
            for (unsigned int j = 0; j < balloons[i].size(); j++) {
                cout << ": " << balloons[i][j] << "\t";
            }
            cout << endl;
        }
        cout << endl;
        */

        //draw points of detections on the mask based on color
        for (unsigned int i = 0; i < balloons.size(); i++) {
            double x = balloons[i][0];
            double y = balloons[i][1];
            double r = balloons[i][2];

            //cout << "balloon " << i << ": " << x << ": " << y << ": " << r << endl;

            //draw circle with point on detected object
            cv::circle(detections,Point(x,y), r, Scalar(0,255,0), 4);
            cv::circle(detections,Point(x,y), 1, Scalar(0,255,0), 1);
        }
        for (unsigned int i = 0; i < goalYs.size(); i++) {
            int x = (int)goalYs[i][0];
            int y = (int)goalYs[i][1];
            int r = (int)goalYs[i][2];

            //draw circle with point on detected object
            cv::rectangle(detections,Point(x-r/2,y-r/2), Point(x+r/2,y+r/2), Scalar(0,255,188), 4);
            cv::circle(detections,Point(x,y), 2, Scalar(0,255,188), 1);
        }
        for (unsigned int i = 0; i < goalOs.size(); i++) {
            int x = (int)goalOs[i][0];
            int y = (int)goalOs[i][1];
            int r = (int)goalOs[i][2];

            //draw circle with point on detected object
            cv::rectangle(detections,Point(x-r/2,y-r/2), Point(x+r/2,y+r/2), Scalar(0,137,255), 4);
            cv::circle(detections,Point(x,y), 2, Scalar(0,137,255), 1);
        }
        for (unsigned int i = 0; i < blimpBs.size(); i++) {
            int x = (int)blimpBs[i][0];
            int y = (int)blimpBs[i][1];
            int r = (int)blimpBs[i][2];

            //draw circle with point on detected object
            cv::circle(detections,Point(x,y), r, Scalar(255,0,0), 4);
            cv::circle(detections,Point(x,y), 2, Scalar(255,0,0), 1);
        }
        for (unsigned int i = 0; i < blimpRs.size(); i++) {
            int x = (int)blimpRs[i][0];
            int y = (int)blimpRs[i][1];
            int r = (int)blimpRs[i][2];

            //draw circle with point on detected object
            cv::circle(detections,Point(x,y), r, Scalar(0,0,255), 4);
            cv::circle(detections,Point(x,y), 2, Scalar(0,0,255), 1);
        }


        vector<vector<Point> > contours;
        vector<Point> approx;
        vector<vector<Point>> outApprox;
        vector<Point> scaled;
        vector<vector<Point>> outScaled;
        vector<Vec4i> hierarchy;
        findContours(goalOMask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);


        Mat debug = Mat::zeros(goalOMask.rows, goalOMask.cols, CV_8UC3);

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

        imshow("orange goal contours", debug);

        debug = Mat();

        contours.clear();
        approx.clear();
        outApprox.clear();
        scaled.clear();
        outScaled.clear();
        hierarchy.clear();
        findContours(goalYMask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);


        debug = Mat::zeros(goalYMask.rows, goalYMask.cols, CV_8UC3);

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

        imshow("yellow goal contours", debug);

        imshow("Detections", detections);


        //show labled points on mask of detections by color

        cout << "Min H: " << minInt <<endl;
        cout << "Min S: " << mina <<endl;
        cout << "Min V: " << minb <<endl;

        cout << "Max H: " << maxInt <<endl;
        cout << "Max S: " << maxa <<endl;
        cout << "Max V: " << maxb <<endl;

        cout << "Blue Correction: " << blueCorrection << endl;
        cout << "Green Correction: " << greenCorrection << endl;
        cout << "Red Correction: " << redCorrection << endl;

        waitKey(WAIT);
	}


	// When everything done, release the video capture object
	inputVideo.release();

	// Closes all the frames
	destroyAllWindows();

	return 0;
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

vector<vector<double>> getObjects(int type, Mat mask) {

    vector<double> object;
    vector<vector<double>> set;

    //check if inputs are in the correct range
    if (!mask.empty()) {
        //get object data
        Mat locations;
        findNonZero(mask, locations);

        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;

        findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        //hierarchy[i] = [next, previous, first_child, parent]

        Mat debug = Mat::zeros(mask.rows, mask.cols, CV_8UC3);

        //cout << "Number of contours: " << contours.size() << endl;

        // iterate through all the top-level contours,
        // draw each connected component with its own random color
        for (unsigned int i = 0; i < contours.size(); i++) {
            //perfom initial area filter
            double area = contourArea(contours[i]);
            if (hierarchy[i][3] == -1 && (area > MIN_AREA || type == goalO || type == goalY)) {

                //temp matrix for drawing
                Mat temp = Mat::zeros(mask.rows, mask.cols, CV_8UC3);
                Scalar color(255, 255, 255);
                Mat tempSingle;

                unsigned int indexOfInner = i;

                vector<Point> approx;
                vector<vector<Point>> outApprox;
                vector<vector<Point>> outScaled;

                if (type == goalO || type == goalY) {
                    //do approx poly to close contours and smooth them
                    double epsilon = G_POLY_APPROX_E*cv::arcLength(contours[i],true);
                    cv::approxPolyDP(contours[i], approx, epsilon,true);
                    outApprox.push_back(approx);
                    //scale contour
                    outScaled.push_back(scaleContour(approx, GOAL_INNER_CONTOUR_SCALE));

                    Mat innerSingle, goalSingle;
                    Mat innerSingleBit, goalSingleBit;
                    Mat inner = Mat::zeros(mask.rows, mask.cols, CV_8UC3);
                    Mat goal = Mat::zeros(mask.rows, mask.cols, CV_8UC3);

                    drawContours(inner, outScaled, 0, color, FILLED, 8);
                    drawContours(goal, contours, i, color, FILLED, 8, hierarchy);

                    cvtColor(inner, innerSingle, COLOR_BGR2GRAY);
                    cvtColor(goal, goalSingle, COLOR_BGR2GRAY);

                    //get just the goal detected
                    bitwise_xor(goalSingle, goalSingle, goalSingle, innerSingle);

                    //set as optical flow mask
                    tempSingle = goalSingle;
                    //bitwise_or(debug, tempSingle, debug);


                } else {
                    drawContours(temp, contours, i, color, FILLED, 8, hierarchy);
                    //get grayscale of specified filled coutour for optical flow mask
                    cvtColor(temp, tempSingle, COLOR_BGR2GRAY);
                }

                //<------------Confidence---------------->
                double confidence = 0;

                if (type == goalO || type == goalY) {
                    //get confidence interval of goal
                    Mat innerSingle;
                    Mat innerSingleBit, goalSingleBit;
                    Mat inner = Mat::zeros(mask.rows, mask.cols, CV_8UC3);

                    drawContours(inner, outScaled, 0, color, FILLED, 8);

                    cvtColor(inner, innerSingle, COLOR_BGR2GRAY);

                    //get area of each goal mask
                    //float areaI = contourArea(outScaled[0]);
                    //float areaG = contourArea(approx)-areaI;

                    double areaI = (double)countNonZero(innerSingle);
                    double areaG = (double)countNonZero(tempSingle);

                    //get the pixels detected in each region
                    bitwise_and(innerSingle, mask, innerSingleBit);
                    bitwise_and(tempSingle, mask, goalSingleBit);

                    double innerDetect = ((double)countNonZero(innerSingleBit))/areaI;
                    double goalDetect = ((double)countNonZero(goalSingleBit))/areaG;

                    //cout << "inner total" << areaI << endl;
                    //cout << "inner detected" << innerDetect << endl;

                    confidence = goalDetect*(1-innerDetect);

                    //cout << "condfidence:" << confidence << endl;
                } else if (type == balloon) {
                    //get confidence of balloon (from adams code)
                    Point2f center;
					float radius;
					minEnclosingCircle(contours[i],center,radius);

					//Establish confidence
					Mat innerCircleMask = Mat::zeros(mask.size(),CV_8U);
					Mat gapRingMask = Mat::zeros(mask.size(),CV_8U);

					circle(innerCircleMask, center, radius, 255, -1);
					circle(gapRingMask, center, radius + MIN_GAP_DISTANCE, 255, -1);
					subtract(gapRingMask, innerCircleMask, gapRingMask);

					float innerCircleArea = 3.1415*pow(radius,2);
					float gapRingArea = 3.1415*(pow(radius+MIN_GAP_DISTANCE,2)-pow(radius,2));

					Mat positiveDetection;
					Mat negativeDetection;
					bitwise_and(innerCircleMask, mask, positiveDetection);
					bitwise_and(gapRingMask, mask, negativeDetection);

					float innerCirclePixels = countNonZero(positiveDetection);
					float gapRingPixels = countNonZero(negativeDetection);

					float good = innerCirclePixels/innerCircleArea;
					float bad = gapRingPixels/gapRingArea;

					confidence = good * (1-bad);
                } else {
                    //is blimp
                    confidence = 1;
                }

                //<------------x, y coordinates---------------->
                Point2f center;
			    float radius;
				minEnclosingCircle(contours[i],center,radius);

                double x = (double)center.x;
                double y = (double)center.y;

                double confidenceCheck = 0;

                if (type == goalO || type == goalY) {
                    confidenceCheck = G_MIN_CONFIDENCE;
                } else if (type == balloon) {
                    confidenceCheck = B_MIN_CONFIDENCE;
                }

                //add parameters to object and push to output
                if (confidence >= confidenceCheck) {
                    //cout << confidence << endl;
                    object.push_back(x/MASK_SCALE);
                    object.push_back(y/MASK_SCALE);
                    object.push_back((double)radius/MASK_SCALE);
                    object.push_back(area/(MASK_SCALE*MASK_SCALE));
                    object.push_back(confidence);
                    set.push_back(object);
                    object.clear();
                }
            }
        }
        //imshow("Mask of goals", mask);
    }

    return set;
}
