//C++ includes
#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <iostream>
#include <list>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <vector>

//C includes
#include <errno.h>
#include <fcntl.h>
#include <libv4l2.h>
#include <linux/videodev2.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include <wiringPi.h>
#include <wiringSerial.h>

#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;

#define YELLOW_CIRCLE 1
#define YELLOW_SQUARE 3
#define YELLOW_TRIANGLE 5
#define ORANGE_CIRCLE 7
#define ORANGE_SQUARE 9
#define ORANGE_TRIANGLE 11

// int targetID = RED_SQUARE;

//Current detection state (controlled by ESP32)
enum VisionState {
    idle,
    detectBall,
    detectOrangeGoal,
    detectYellowGoal
} visionState;

//Use camera setting:
//v4l2-ctl -d /dev/video0 -c white_balance_temperature_auto=0 -c white_balance_temperature=5 -c exposure_auto=1 -c exposure_auto_priority=0 -c exposure_absolute=110

using namespace std;
using namespace cv;

const int m_width = 1920; // image size in pixels
const int m_height = 1080;
const double m_tagSize = 0.914; // April tag side length in meters of square black frame
const double m_fx = 2.1*m_width/0.94; // camera focal length in pixels
const double m_fy = 2.1*m_height/0.94;
const double m_px = m_width/2; // camera principal point
const double m_py = m_height/2;
const int m_deviceId = 0; // camera id (in case of multiple cameras)

//Set camera exposure
//exposure_absolute 0x009a0902 (int)    : min=10 max=626 step=1 default=156 value=156 flags=inactive
const int m_exposure = -1;

cv::VideoCapture m_cap;

void setupVideo() {
    // manually setting camera exposure settings; OpenCV/v4l1 doesn't
    // support exposure control; so here we manually use v4l2 before
    // opening the device via OpenCV; confirmed to work with Logitech
    // C270; try exposure=20, gain=100, brightness=150
    string video_str = "/dev/video0";
    video_str[10] = '0' + m_deviceId;
    int device = v4l2_open(video_str.c_str(), O_RDWR | O_NONBLOCK);

    if (m_exposure >= 0) {
        // not sure why, but v4l2_set_control() does not work for
        // V4L2_CID_EXPOSURE_AUTO...
        struct v4l2_control c;
        c.id = V4L2_CID_EXPOSURE_AUTO;
        c.value = 1; // 1=manual, 3=auto; V4L2_EXPOSURE_AUTO fails...
        if (v4l2_ioctl(device, VIDIOC_S_CTRL, &c) != 0) {
            cout << "Failed to set... " << strerror(errno) << endl;
        }
        cout << "exposure: " << m_exposure << endl;
        v4l2_set_control(device, V4L2_CID_EXPOSURE_ABSOLUTE, m_exposure*6);
    } else {
        struct v4l2_control c;
        c.id = V4L2_CID_EXPOSURE_AUTO;
        c.value = 3; // 1=manual, 3=auto; V4L2_EXPOSURE_AUTO fails...
        if (v4l2_ioctl(device, VIDIOC_S_CTRL, &c) != 0) {
            cout << "Failed to set... " << strerror(errno) << endl;
        }
    }

    v4l2_close(device);

    // find and open a USB camera (built in laptop camera, web cam etc)
    m_cap = cv::VideoCapture(m_deviceId);
    if(!m_cap.isOpened()) {
        cerr << "ERROR: Can't find video device " << m_deviceId << "\n";
        exit(1);
    }

    //Set camera resolution
    m_cap.set(cv::CAP_PROP_FRAME_WIDTH, m_width);
    m_cap.set(cv::CAP_PROP_FRAME_HEIGHT, m_height);
    cout << "Camera successfully opened (ignore error messages above...)" << endl;
    cout << "Actual resolution: "
    << m_cap.get(cv::CAP_PROP_FRAME_WIDTH) << "x"
    << m_cap.get(cv::CAP_PROP_FRAME_HEIGHT) << endl;
}

int missCount = 0;

int tagX;
double tagZ, tagDistance;

int lastTagX;
double lastTagZ, lastTagDistance;

int main(int, char**) {

    //Open serial port for ESP32 comms
    int fd;
    if ((fd = serialOpen ("/dev/ttyS0", 115200)) < 0) {
        fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno));
        return 1 ;
    }

    if (wiringPiSetup () == -1) {
        fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno));
        return 1 ;
    }
    cout << "opencv version: " << CV_VERSION << endl;

    setupVideo();

    int detectedX, detectedY, detectedDiameter;
    int balloonX, balloonY, balloonDiameter;
    int goalX, goalY, goalArea;
    Mat img, imgHSV, mask, imgBlur, imgCanny, imgDil;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(2,2));
    cv::Mat image_gray;

    //HSV thresholds
    Scalar balloonHSVLower(40,58,60);
    Scalar balloonHSVUpper(75,171,255);

    //Buffers for reading and sending data to/from the ESP32
    char sendBuf[32];
    char readBuf[32];

    //Timing variables
    clock_t now;
    clock_t lastMsgRcvTime = 0;
    clock_t lastMsgSendTime = 0;

    //Initialize vision state to idle
    visionState = idle;

    //Wait for handshake from ESP32 (must receive a zero)
    bool handshakeReceived = false;
    do {
        int bytesReceived = serialDataAvail(fd);
        if (bytesReceived > 0) {
            //Read all pending bytes into the buffer
            for (int i = 0; i < bytesReceived; i++) {
                readBuf[i] = serialGetchar(fd);
            }
            readBuf[bytesReceived] = '\0';

            int valueReceived = atoi(&readBuf[bytesReceived-1]);

            printf("Received %d\n", valueReceived);

            //Make sure the ESP32 sent a valid state to initialize transmission
            if (valueReceived >= idle && valueReceived <= detectYellowGoal) {
                fprintf(stdout, "Handshake received!\n");
                handshakeReceived = true;
                visionState = (VisionState)valueReceived;
                switch (valueReceived) {
                    case idle:
                        fprintf(stdout, "Switched to idling\n");
                        break;
                    case detectBall: 
                        fprintf(stdout, "Switched to ball detection\n");
                        break;
                    case detectOrangeGoal:
                        fprintf(stdout, "Switched to orange goal detection\n");
                        break;
                    case detectYellowGoal:
                        fprintf(stdout, "Switched to yellow goal detection\n");
                        break;
                }

                //Send the received state back as a handshake
                sprintf(sendBuf, "%d", valueReceived);
                serialPuts(fd, sendBuf);
                printf("return handshake sent\n");
            } else {
                fprintf(stdout, "Invalid vision state received. Handshake rejected.\n");
            }
        } else {
            //Sleep for 1 second
            printf("Waiting for handshake...\n");
            
            sprintf(sendBuf, "%04d,%04d,%04d;", -1, -1, -1);
            serialPuts(fd, sendBuf);

            sleep(1);
        }
    } while (!handshakeReceived);
    
    while (true) {

        m_cap.read(img);

        switch (visionState) {
            case idle: {
                //Do nothing but send -1s to keep comms alive
                detectedX = -1;
                detectedY = -1;
                detectedDiameter = -1;
                break;
            } case detectBall: {
                //Balloon detection opencv code
                //Preprocessing - blur + HSV filter
                GaussianBlur(img, imgBlur, Size(15,15), 0);
                cvtColor(imgBlur, imgHSV, COLOR_BGR2HSV);
                inRange(imgHSV, balloonHSVLower, balloonHSVUpper, mask);

                //Find contours in the masked image
                findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

                // cout << (int)contours.size() << " contours detected" << endl;

                //Find the contour with the greatest area using polygonal approximations
                vector<vector<Point>> conPoly(contours.size());
                double maxArea = -1;
                double secondMaxArea = -1;
                int iMax = -1;
                int iSecondMax = -1;
                for (int i = 0; i < (int)contours.size(); i++) {
                    double area = contourArea(contours[i]);
                    if (area > 100) {
                        if (area > maxArea) {
                            //Store the second greatest area
                            secondMaxArea = maxArea;
                            iSecondMax = iMax;

                            //Update the greatest area
                            maxArea = area;
                            iMax = i;
                        }
                    }
                }

                if (iMax > -1) {
                    //Todo: filter by area of cirlce and contour?
                    Point2f maxCenter, secondMaxCenter, finalCenter;
                    float maxRadius, secondMaxRadius, finalRadius;

                    //First, find the minimum enclosing circle for the biggest contour
                    minEnclosingCircle(contours[iMax], maxCenter, maxRadius);
                    double maxArea = M_PI*maxRadius*maxRadius;

                    //Initialize final center and radius to match the max (they will be updated if combined with second max)
                    finalCenter = maxCenter;
                    finalRadius = maxRadius;
                    double finalArea = maxArea;

                    //If second greatest area exists, check its distance from the first area
                    if (iSecondMax > -1) {
                        minEnclosingCircle(contours[iSecondMax], secondMaxCenter, secondMaxRadius);

                        //Compute the distance between circle centers
                        double centerDist = sqrt(pow(maxCenter.x-secondMaxCenter.x, 2) + pow(maxCenter.y-secondMaxCenter.y, 2));

                        //If the distance is contained within the diameter of the biggest circle, combine them
                        if (centerDist <= 2*maxRadius) {
                            //First, combine both contours into a single contour
                            vector<Point> finalContour;
                            finalContour.reserve(contours[iMax].size() + contours[iSecondMax].size());
                            finalContour.insert(finalContour.end(), contours[iMax].begin(), contours[iMax].end());
                            finalContour.insert(finalContour.end(), contours[iSecondMax].begin(), contours[iSecondMax].end());

                            //Next, combine all points into a single convex hull
                            vector<Point> hull;
                            convexHull(Mat(finalContour), hull, false);

                            //Find the min enclosing circle for the hull
                            minEnclosingCircle(hull, finalCenter, finalRadius);
                        }
                    }

                    balloonX = round(finalCenter.x);
                    balloonY = round(finalCenter.y);
                    balloonDiameter = round(finalRadius*2);

                    // cout << "pos: " << finalCenter.x << ", " << finalCenter.y << " area: " << finalArea << endl;
                } else {
                    //Balloon not found
                    balloonX = -1;
                    balloonY = -1;
                    balloonDiameter = -1;
                }

                detectedX = balloonX;
                detectedY = balloonY;
                detectedDiameter = balloonDiameter;
                break;
            } case detectOrangeGoal: {

                break;
            } case detectYellowGoal: {

                break;
            }
        }

        //If not idling, send the current detected position at 10Hz maximum
        now = clock();
        if (double(now - lastMsgSendTime) / CLOCKS_PER_SEC >= 0.1) {
            switch (visionState) {
                case detectBall:
                    //Pad the numbers sent to make reading on the esp straightforward
                    sprintf(sendBuf, "%04d,%04d,%04d;", detectedX, detectedY, detectedDiameter);
                    break;
                case detectOrangeGoal:
                case detectYellowGoal:
                    sprintf(sendBuf, "%04d,%0.4f,%0.4f;", tagX, tagZ, tagDistance);
                    break;
                default:
                    //idle
                    sprintf(sendBuf, "%04d,%04d,%04d;", -1, -1, -1);
            }

            serialPuts(fd, sendBuf);
            lastMsgSendTime = clock();

            fprintf(stdout, "Sent %s\n", sendBuf);
        }

        //Check for data received from ESP32
        int bytesReceived = serialDataAvail(fd);
        if (bytesReceived > 0) {
            //Read all pending bytes into the buffer
            for (int i = 0; i < bytesReceived; i++) {
                readBuf[i] = serialGetchar(fd);
            }
            readBuf[bytesReceived] = '\0';

            //Parse the final byte received as the latest command
            int latestCommand = atoi(&readBuf[bytesReceived-1]);

            //If the command is valid, update the vision state
            if (latestCommand >= idle && latestCommand <= detectYellowGoal) {
                //Only print update if it switches the state
                if (visionState != latestCommand) {
                    visionState = (VisionState)latestCommand;
                    switch (visionState) {
                        case idle:
                            printf("Switched to idling\n");
                            break;
                        case detectBall: 
                            printf("Switched to ball detection\n");
                            break;
                        case detectOrangeGoal:
                            printf("Switched to orange goal detection\n");
                            break;
                        case detectYellowGoal:
                            printf("Switched to yellow goal detection\n");
                            break;
                    }
                }
            } else {
                visionState = idle;
                fprintf(stdout, "Received invalid command! Idling...\n");
            }

            //Update the last transmission received time
            lastMsgRcvTime = now;
        }

        //If no messages received for 2 seconds, timeout and switch to idle state
        if (visionState != idle && double(now - lastMsgRcvTime) / CLOCKS_PER_SEC >= 2.0) {
            visionState = idle;
            fprintf(stdout, "ESP stopped responding! Idling...\n");
        }
    }

    return 0;
}
