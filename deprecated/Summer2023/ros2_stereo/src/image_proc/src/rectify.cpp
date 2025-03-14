// Copyright 2008, 2019 Willow Garage, Inc., Andreas Klintberg, Joshua Whitley
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.hpp>


#include <thread>
#include <memory>
#include <vector>

#include "tracetools_image_pipeline/tracetools.h"
#include "image_proc/rectify.hpp"

namespace image_proc
{

RectifyNode::RectifyNode(const rclcpp::NodeOptions & options)
: Node("RectifyNode", options)
{
  queue_size_ = this->declare_parameter("queue_size", 7);
  interpolation = this->declare_parameter("interpolation", 1);
  pub_rect_ = image_transport::create_publisher(this, "image_rect");
  subscribeToCamera();
}

// Handles (un)subscribing when clients (un)subscribe
void RectifyNode::subscribeToCamera()
{
  std::lock_guard<std::mutex> lock(connect_mutex_);


  // *  SubscriberStatusCallback not yet implemented
  // *
  // if (pub_rect_.getNumSubscribers() == 0)
  //   sub_camera_.shutdown();
  // else if (!sub_camera_)
  // {

  sub_camera_ = image_transport::create_camera_subscription(this, "image", std::bind(&RectifyNode::imageCb, this, std::placeholders::_1, std::placeholders::_2), "raw");
  // }
}

void RectifyNode::imageCb(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
{
  // TRACEPOINT(
  //   image_proc_rectify_init,
  //   static_cast<const void *>(this),
  //   static_cast<const void *>(&(*image_msg)),
  //   static_cast<const void *>(&(*info_msg)));

  if (pub_rect_.getNumSubscribers() < 1) {
    // TRACEPOINT(
    //   image_proc_rectify_fini,
    //   static_cast<const void *>(this),
    //   static_cast<const void *>(&(*image_msg)),
    //   static_cast<const void *>(&(*info_msg)));

    return;
  }

  // Verify camera is actually calibrated
  if (info_msg->k[0] == 0.0) {
    RCLCPP_ERROR(
      this->get_logger(), "Rectified topic '%s' requested but camera publishing '%s' "
      "is uncalibrated", pub_rect_.getTopic().c_str(), sub_camera_.getInfoTopic().c_str());
    // TRACEPOINT(
    //   image_proc_rectify_fini,
    //   static_cast<const void *>(this),
    //   static_cast<const void *>(&(*image_msg)),
    //   static_cast<const void *>(&(*info_msg)));
    return;
  }

  // If zero distortion, just pass the message along
  bool zero_distortion = true;

  for (size_t i = 0; i < info_msg->d.size(); ++i) {
    if (info_msg->d[i] != 0.0) {
      zero_distortion = false;
      break;
    }
  }

  // This will be true if D is empty/zero sized
  if (zero_distortion) {
    pub_rect_.publish(image_msg);
    // TRACEPOINT(
    //   image_proc_rectify_fini,
    //   static_cast<const void *>(this),
    //   static_cast<const void *>(&(*image_msg)),
    //   static_cast<const void *>(&(*info_msg)));
    return;
  }

  // Update the camera model
  // model_.fromCameraInfo(info_msg);

  // Create cv::Mat views onto both buffers
  // const cv::Mat image = cv_bridge::toCvShare(image_msg)->image; 
  cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_msg);
  cv::Mat resized_image;
  cv::resize(cv_image->image, resized_image, cv::Size(cv_image->image.cols, cv_image->image.rows)); //original size right now
  
  cv::Mat rect; //empty matrix

  // RCLCPP_ERROR(
  // this->get_logger(), "focal length '%d' ", image.depth());

  // Rectify and publish
  
  cv::Mat map_1,map_2;
  cv::Size size = cv::Size(info_msg->width,info_msg->height);
 
  cv::Matx33d K(info_msg->k[0], info_msg->k[1], info_msg->k[2],
                info_msg->k[3], info_msg->k[4], info_msg->k[5],
                info_msg->k[6], info_msg->k[7], info_msg->k[8]);

  cv::Matx33d R(info_msg->r[0], info_msg->r[1], info_msg->r[2],
                info_msg->r[3], info_msg->r[4], info_msg->r[5],
                info_msg->r[6], info_msg->r[7], info_msg->r[8]);

  cv::Matx34d P(info_msg->p[0], info_msg->p[1], info_msg->p[2], info_msg->p[3],
                info_msg->p[4], info_msg->p[5], info_msg->p[6], info_msg->p[7],
                info_msg->p[8], info_msg->p[9], info_msg->p[10], info_msg->p[11]);

  cv::Mat D(1, 5, CV_32F);
  D.at<float>(0, 0) = info_msg->d[0];
  D.at<float>(0, 1) = info_msg->d[1];
  D.at<float>(0, 2) = info_msg->d[2];
  D.at<float>(0, 3) = info_msg->d[3];
  D.at<float>(0, 4) = info_msg->d[4];

  cv::initUndistortRectifyMap(K,D,R,P,size,CV_16SC2,map_1,map_2);
  cv::remap(resized_image,rect,map_1,map_2,interpolation);
  // model_.rectifyImage(resized_image, rect, interpolation);


  // Allocate new rectified image message
  sensor_msgs::msg::Image::SharedPtr rect_msg = cv_bridge::CvImage(image_msg->header, image_msg->encoding, rect).toImageMsg();
  pub_rect_.publish(rect_msg);
  
  // TRACEPOINT(
  //   image_proc_rectify_fini,
  //   static_cast<const void *>(this),
  //   static_cast<const void *>(&(*image_msg)),
  //   static_cast<const void *>(&(*info_msg)));
}

}  // namespace image_proc

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(image_proc::RectifyNode)
