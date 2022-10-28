#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/stereo.hpp>

#include <iostream>
#include <vector>
#include <ctime>

//<-----------Constants------------->
#define CAMERA_INDEX    0
#define CAMERA_WIDTH    640
#define CAMERA_HEIGHT   240

//<------------Color Correction----->
#define B_Correction    Scalar(56,31,23)
#define R_Correction    Scalar(0,0,0)
#define B_MIN           Scalar(38,0,0)
#define B_MAX           Scalar(86,255,255)

#define R_MIN           Scalar(0,0,81)
#define R_MAX           Scalar(87,78,255)

//<------------Stereo--------------->
#define STEREO_BLUR     9

#define DISP_WIDTH      160/2
#define DISP_HEIGHT     120/2
#define F               289
#define B               60.85

//<-----------Objects------------->
#define MIN_AREA         25
#define SIZE_RATIO       1.5



//Camera
using namespace cv;
using namespace std;

vector<Point> scaleContour(vector<Point> contour, double scale);

//Display camera feed
int main(int argc, char** argv) {

        
        VideoCapture inputVideo(CAMERA_INDEX);

        inputVideo.set(CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
        inputVideo.set(CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);

        //Make sure camera is connected
        if (!inputVideo.isOpened()) {
                CV_Assert("Cam open failed");
                cout << "camera failed to open" << endl;
                return 0;
        }

        

        //read in stereo rectification data
        Mat Left_Stereo_Map1, Left_Stereo_Map2;
        Mat Right_Stereo_Map1, Right_Stereo_Map2;
        Mat Q;
        cv::FileStorage cv_file2 = cv::FileStorage("stereo_rectify_maps.xml", cv::FileStorage::READ);
        cv_file2["Left_Stereo_Map_x"] >> Left_Stereo_Map1;
        cv_file2["Left_Stereo_Map_y"] >> Left_Stereo_Map2;
        cv_file2["Right_Stereo_Map_x"] >> Right_Stereo_Map1;
        cv_file2["Right_Stereo_Map_y"] >> Right_Stereo_Map2;
        cv_file2["Q"] >> Q;
        cv_file2.release();
        
        //pull first frame
        Mat frametemp;

        inputVideo >> frametemp;

        //split frame into two images
        Mat imgLt, imgRt;
        
        Rect left_roit(0, 0, frametemp.cols/2, frametemp.rows);
        Rect right_roit(frametemp.cols/2, 0, frametemp.cols/2, frametemp.rows);

        Mat crop_leftt(frametemp, left_roit);
        Mat crop_rightt (frametemp, right_roit);

        crop_leftt.copyTo(imgLt);
        crop_rightt.copyTo(imgRt);
        

       //Mat imgLt = imread("niceL.jpg");

        //create stereo object
        int minDisp = -8;
        int numDisp = 16;
        Ptr< StereoSGBM > stereo = StereoSGBM::create(minDisp,                            //min disparities
                                                numDisp,                                 //num disparities
                                                9,                                     //block size
                                                200,                                    //P1
                                                800,                                    //P2
                                                100,                                      //disp12MaxDiff
                                                5,                                     //preFilterCap
                                                0,                                      //uniquenessRation
                                                0,                                      //speckleWindowSize
                                                0,                                     //speckleRange
                                                StereoSGBM::MODE_HH);


        int minb = 0;
        int maxb = 255;
        int maxInt = 255;
        int minInt = 0;
        int maxa= 255;
        int mina = 0;

        namedWindow("Track");
        createTrackbar("Min H", "Track", &minInt, 255);
        createTrackbar("Max H", "Track", &maxInt, 255);
        createTrackbar("Min S", "Track", &mina, 255);
        createTrackbar("Max S", "Track", &maxa, 255);
        createTrackbar("Min V", "Track", &minb, 255);
        createTrackbar("Max V", "Track", &maxb, 255);

        clock_t last = 0;

	while (true) {
                //get frame
                
                Mat frame;

                inputVideo >> frame;

                //split frame into two images
                Mat imgL, imgR;
               
                Rect left_roi(0, 0, frame.cols/2, frame.rows);
                Rect right_roi (frame.cols/2, 0, frame.cols/2, frame.rows);

                Mat crop_left(frame, left_roi);
                Mat crop_right (frame, right_roi);

                crop_left.copyTo(imgL);
                crop_right.copyTo(imgR);

                imshow("Frame L", imgL);
                imshow("Frame R", imgR);

                //apply gaussian blur


                //convert to greyscale
                Mat imgL_gray, imgR_gray;
                cv::cvtColor(imgL, imgL_gray, cv::COLOR_BGR2GRAY);
                cv::cvtColor(imgR, imgR_gray, cv::COLOR_BGR2GRAY);

                Mat imgL_gray_blur, imgR_gray_blur;
                GaussianBlur(imgL_gray, imgL_gray_blur, Size(9, 9), 100, 100);
                GaussianBlur(imgR_gray, imgR_gray_blur, Size(9, 9), 100, 100);

                imshow("processed stereo", imgL_gray_blur);

                //undistort images
                Mat left_correct, right_correct;

                remap(imgL_gray_blur,
                        left_correct,
                        Left_Stereo_Map1,
                        Left_Stereo_Map2,
                        INTER_LANCZOS4,
                        BORDER_CONSTANT,
                        0);

                remap(imgR_gray_blur,
                        right_correct,
                        Right_Stereo_Map1,
                        Right_Stereo_Map2,
                        INTER_LANCZOS4,
                        BORDER_CONSTANT,
                        0);

                //resize images
                Mat imgL_gray_small;
                Mat imgR_gray_small;

                resize(imgL_gray, imgL_gray_small, Size(DISP_WIDTH, DISP_HEIGHT), INTER_LINEAR);
                resize(imgR_gray, imgR_gray_small, Size(DISP_WIDTH, DISP_HEIGHT), INTER_LINEAR);

                //compute disparity
                clock_t start, end;

                start = clock();
                Mat disp;
                stereo->compute(imgL_gray_small, imgR_gray_small, disp);
                end = clock();


                disp.convertTo(disp,CV_32F, 1.0);
                Mat disp2 = (disp/16.0f - minDisp)/(numDisp);
                //imshow("Disparity",disp2);

                //calculate depth
                Mat depth;
                reprojectImageTo3D(disp, depth, Q, true, -1);
                Mat XYZ[3];
                split(depth, XYZ);
                depth = XYZ[2];
                //imshow("Depth", depth/1000);

                
                //convert depth to a 3 channel array
                Mat BGR;
                Mat hsv;
                cvtColor(depth/1000, BGR, cv::COLOR_GRAY2BGR);

                //filter out zeros from BGR
                Mat zmask;
                inRange(BGR, Scalar(0,0,0), Scalar(2,2,2), zmask);
                bitwise_not(zmask, zmask);
                //imshow("zmask", zmask);

                multiply(zmask, 255, zmask);
                cvtColor(zmask, zmask, cv::COLOR_GRAY2BGR);
                zmask.convertTo(zmask, CV_32F, 1.0);
                add(BGR, zmask, BGR);

                BGR.convertTo(BGR, CV_8U, 255);

                //resize BGR for color detection
                resize(BGR, BGR, Size(CAMERA_WIDTH/2, CAMERA_HEIGHT), INTER_LINEAR);
                resize(depth, depth, Size(CAMERA_WIDTH/2, CAMERA_HEIGHT), INTER_LINEAR);

                //get mask for backgourd of image
                Mat backMask;
                Mat foregroundMask;
                inRange(BGR, Scalar(255,255,255), Scalar(10000, 10000, 10000), backMask);
                bitwise_not(backMask, foregroundMask);
                imshow("Foreground Depth", foregroundMask);
                //color detections on bgr depth map
                Mat bMask;
                Mat imgLhsv;

                Mat balloonCorrected;
                Mat balloonCorrectM = Mat::zeros(imgL.size(), imgL.type());
                balloonCorrectM.setTo(B_Correction);
                add(imgL,balloonCorrectM, balloonCorrected);

                cvtColor(balloonCorrected, imgLhsv, cv::COLOR_BGR2HSV);

                //hsv only
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


                //inRange(imgL, Scalar(minInt, mina, minb), Scalar(maxInt, maxa, maxb), bMask);
                inRange(imgLhsv, B_MIN, B_MAX, bMask);
                //inRange(imgLhsv, Scalar(minInt,mina,minb),Scalar(maxInt,maxa,maxb), bMask);
                //imshow("Mask", bMask);


                vector<vector<Point> > contours;
                vector<Vec4i> hierarchy;

                findContours(bMask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
        

                cout << "Number of contours: " << contours.size() << endl;

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
                                        bitwise_and(temp, foregroundMask, temp);

                                        Mat mean, stddev;
                                        cv::meanStdDev(depth, mean, stddev, temp);

                                        imshow("Temp", temp);

                                        double conversion = 0.171428/2.0;
                                        cout << "Depth of object: " << mean.at<double>(0,0)*conversion << "in\tstd: " << stddev.at<double>(0,0)*conversion << endl;

                                        //draw shape on image
                                        ellipse(BGR, objEllipse, Scalar(0,0,255), 3);
                                }


                        
                        }
                }

                //imshow("BGR", BGR);

                
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
                imshow("color depth", hsvdepth);

                cout << "Min H: " << minInt <<endl;
                cout << "Min S: " << mina <<endl;
                cout << "Min V: " << minb <<endl;

                cout << "Max H: " << maxInt <<endl;      
                cout << "Max S: " << maxa <<endl;
                cout << "Max V: " << maxb <<endl;
                

                clock_t now = clock();
                cout << "Vision Compute Time: " << (float)(now - last)/(float)CLOCKS_PER_SEC << "s" << endl;
                cout << "Vision Compute Rate: " << 1/((float)(now - last)/(float)CLOCKS_PER_SEC) << "Hz" << endl;
                last = now;

                int c = waitKey(50);
                if (c % 256 == 27) {
                        break;
                }
	}

        cout << "Exiting Program" << endl;
    

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

