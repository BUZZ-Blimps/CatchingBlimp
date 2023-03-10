#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/ximgproc.hpp"
#include <opencv2/viz.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <ctime>
#include <chrono>

using namespace cv;
using namespace std;

void delay(double delaySeconds){
	clock_t start = clock();
	while(double(clock() - start)/CLOCKS_PER_SEC < delaySeconds);
}

int main(int argc, char** argv) {

        viz::Viz3d myWindow("Coordinate Frame");
        myWindow.spinOnce();
        delay(5);

        myWindow.showWidget("Coordinate Widget", viz::WCoordinateSystem());
        Mat xyz = Mat::zeros(Size(2,2),CV_32FC3);
        xyz.at<Vec3f>(0,0) = Vec3f(2,0,0);
        xyz.at<Vec3f>(0,1) = Vec3f(0,2,0);
        xyz.at<Vec3f>(1,0) = Vec3f(0,0,2);
        xyz.at<Vec3f>(1,1) = Vec3f(1,1,1);
        viz::WCloud cloud(xyz);
        myWindow.showWidget("Cloud", cloud);
        while(true){
                myWindow.spinOnce();
                //imshow("Test", xyz);
                //waitKey(1);
        }
        cout << "Bonk" << endl;
        return 0;
}