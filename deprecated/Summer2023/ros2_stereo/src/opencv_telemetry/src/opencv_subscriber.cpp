// Copyright 2021, Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

std::string cameraNs_ = "BurnCreamBlimp"; // Camera ns

// initialize values for StereoSGBM parameters
int numDisparities = 192;
int blockSize = 5;
int preFilterType = 1;
int preFilterSize = 5;
int preFilterCap = 4;
int minDisparity = -64;
int textureThreshold = 0;
int uniquenessRatio = 1;
int speckleRange = 2;
int speckleWindowSize = 150;
int disp12MaxDiff = 10;
int dispType = CV_16S;
 
// Creating an object of StereoSGBM algorithm
Ptr<cv::StereoBM> stereo = StereoBM::create();
// Ptr<cv::StereoSGBM> sgbm = StereoSGBM::create();

Mat imgL;
Mat imgR;
Mat imgL_gray;
Mat imgR_gray;
 

void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  try {
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    // Resize the image to HD
    Mat resized_image;
    resize(cv_image->image, resized_image, cv::Size(cv_image->image.cols, cv_image->image.rows)); //original size right now

    // cout << "image size : " << resized_image.size << endl;

    // Crop the left and right imagesom
    Rect left_roi(0, 0, resized_image.cols/2, resized_image.rows);
    Rect right_roi(resized_image.cols/2, 0, resized_image.cols/2, resized_image.rows);
    Mat imgL(resized_image, left_roi);
    Mat imgR(resized_image, right_roi);

    namedWindow("imgL");
    imshow("imgL", imgL);
    waitKey(1);

    namedWindow("imgR");
    imshow("imgR", imgR);
    waitKey(1);

    // Creating a named window to be linked to the trackbars
    namedWindow("disparity",WINDOW_NORMAL);

    stereo->setBlockSize(9);
    stereo->setNumDisparities(112);
    stereo->setPreFilterSize(5);
    stereo->setPreFilterCap(61);
    stereo->setMinDisparity(-39);
    stereo->setTextureThreshold(507);
    stereo->setUniquenessRatio(0);
    stereo->setSpeckleWindowSize(0);
    stereo->setSpeckleRange(8);
    stereo->setDisp12MaxDiff(1);

    // Set the parameters
    // sgbm->setBlockSize(5);
    // sgbm->setNumDisparities(192);
    // sgbm->setPreFilterCap(4);
    // sgbm->setMinDisparity(-64);
    // sgbm->setUniquenessRatio(1);
    // sgbm->setSpeckleWindowSize(150);
    // sgbm->setSpeckleRange(2);
    // sgbm->setDisp12MaxDiff(10);
    // // sgbm->setMode(cv::StereoSGBM::MODE_HH); // Set the operation mode to fullDP
    // sgbm->setP1(600);
    // sgbm->setP2(2400);

    Mat disp, disparity;

    // Converting images to grayscale
    cvtColor(imgL, imgL_gray, cv::COLOR_BGR2GRAY);
    cvtColor(imgR, imgR_gray, cv::COLOR_BGR2GRAY);
    
    // Calculating disparith using the StereoBM algorithm
    stereo->compute(imgL_gray,imgR_gray,disp);

    
    // NOTE: Code returns a 16bit signed single channel image,
    // CV_16S containing a disparity map scaled by 16. Hence it 
    // is essential to convert it to CV_32F and scale it down 16 times.
    
    // // Converting disparity values to CV_32F from CV_16S
    // disp.convertTo(disparity,CV_32F, 1.0);
    
    // // Scaling down the disparity values and normalizing them 
    // disparity = (disparity/16.0f - (float)minDisparity)/((float)numDisparities);

    normalize(disp,disparity,0,255,CV_MINMAX,CV_8U);
    
    // Displaying the disparity map
    imshow("disparity",disparity);
    waitKey(1);

    // cv::imshow("view", resized_image);
    // cv::waitKey(10);
    
  } catch (const cv_bridge::Exception & e) {
    auto logger = rclcpp::get_logger("my_subscriber");
    RCLCPP_ERROR(logger, "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_listener", options);
  // cv::namedWindow("view");
  startWindowThread();
  image_transport::ImageTransport it(node);
  image_transport::Subscriber sub = it.subscribe("/" + cameraNs_ + "/sync/image_raw", 1, imageCallback);
  rclcpp::spin(node);
  // cv::destroyWindow("view");
  // cv::destroyWindow("imgL");
  // cv::destroyWindow("imgR");
  return 0;
} 