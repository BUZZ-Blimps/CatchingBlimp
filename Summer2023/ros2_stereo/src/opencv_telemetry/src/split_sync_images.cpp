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

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <chrono> // Add this header for timing

class SplitImage : public rclcpp::Node
{
public:
    SplitImage();

private:
    void syncImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr leftImagePub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rightImagePub_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfoLeft_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfoRight_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr leftCameraInfoPub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rightCameraInfoPub_;
    std::string cameraNs_ ; // Camera ns
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr syncImageSub_;
};

SplitImage::SplitImage() : Node("split_sync_image_node")
{
    cinfoLeft_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "elp_left");
    cinfoRight_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "elp_right");

    this->declare_parameter<std::string>("calibration_file", "camera1");
    std::string calibrationFile = this->get_parameter("calibration_file").as_string();
    std::string cinfoLeftFilePath = "package://opencv_telemetry/calibration/" + calibrationFile + "_elp_left.yaml";
    std::string cinfoRightFilePath = "package://opencv_telemetry/calibration/" + calibrationFile + "_elp_right.yaml";

    // Load calibration files
    cinfoLeft_->loadCameraInfo(cinfoLeftFilePath);
    cinfoRight_->loadCameraInfo(cinfoRightFilePath);
    // cinfoLeft_->loadCameraInfo("package://opencv_telemetry/calibration/elp_left.yaml");
    // cinfoRight_->loadCameraInfo("package://opencv_telemetry/calibration/elp_right.yaml");

    // Set camera namespace
    this->declare_parameter<std::string>("camera_ns", "BurnCreamBlimp");
    cameraNs_ = this->get_parameter("camera_ns").as_string();

    // Create publishers for left and right images
    leftImagePub_ = this->create_publisher<sensor_msgs::msg::Image>("/" + cameraNs_ + "/left/image_raw", 1);
    rightImagePub_ = this->create_publisher<sensor_msgs::msg::Image>("/" + cameraNs_ + "/right/image_raw", 1);

     // Create publishers for left and right camera info
    leftCameraInfoPub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "/" + cameraNs_ + "/left/camera_info", 1);
    rightCameraInfoPub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "/" + cameraNs_ + "/right/camera_info", 1);

    // Subscribe to synchronized image topic
    syncImageSub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/" + cameraNs_ + "/sync/image_raw", 10, std::bind(&SplitImage::syncImageCallback, this, std::placeholders::_1));
}

void SplitImage::syncImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // Given Image, convert to OpenCV Mat
    cv_bridge::CvImagePtr cvPtrLeft;
    cv_bridge::CvImagePtr cvPtrRight;
    std::string leftFrame = cameraNs_ + "_left_optical_frame";
    std::string rightFrame = cameraNs_ + "_right_optical_frame";
    try
    {
         // RCLCPP_ERROR(this->get_logger(), "uwuwuwuwuwuw");
        cvPtrLeft = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cvPtrRight = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    int combinedRows = cvPtrLeft->image.rows;
    int combinedCols = cvPtrLeft->image.cols;
    int imageCols = combinedCols / 2;
    int imageRows = combinedRows;

    // Split the combined image into left and right images
    cv::Rect leftROI(0, 0, imageCols, imageRows);
    cv::Rect rightROI(imageCols, 0, imageCols, imageRows);
    cv::Mat leftCrop = cvPtrLeft->image(leftROI);
    cv::Mat rightCrop = cvPtrRight->image(rightROI);
    cvPtrLeft->image = leftCrop;
    cvPtrRight->image = rightCrop;
    cvPtrLeft->header.frame_id = leftFrame;
    cvPtrRight->header.frame_id = rightFrame;
    //get the header for each   
    int32_t seconds_left = cvPtrLeft->header.stamp.sec;
    int32_t nanoseconds_left = cvPtrLeft->header.stamp.nanosec;

    int32_t seconds_right = cvPtrRight->header.stamp.sec;
    int32_t nanoseconds_right = cvPtrRight->header.stamp.nanosec;

    // Get camera infos
    sensor_msgs::msg::CameraInfo ci_left_= cinfoLeft_->getCameraInfo();
    sensor_msgs::msg::CameraInfo ci_right_= cinfoRight_->getCameraInfo();
    
    //Set camera info headers
    ci_left_.header.frame_id = leftFrame;
    ci_left_.header.stamp.sec = seconds_left;
    ci_left_.header.stamp.nanosec = nanoseconds_left;

    ci_right_.header.frame_id = rightFrame;
    ci_right_.header.stamp.sec = seconds_right;
    ci_right_.header.stamp.nanosec = nanoseconds_right;

    // Publish the left and right images
    leftImagePub_->publish(*(cvPtrLeft->toImageMsg()));
    rightImagePub_->publish(*(cvPtrRight->toImageMsg()));

    // Publish the left and right camera info
    leftCameraInfoPub_->publish(ci_left_);
    rightCameraInfoPub_->publish(ci_right_);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("split_sync_image_node"), "Split synchronized elp image node running");

    auto splitImage = std::make_shared<SplitImage>();

    rclcpp::spin(splitImage);
    rclcpp::shutdown();
    return 0;
}
