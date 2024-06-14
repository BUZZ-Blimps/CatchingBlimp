// Copyright (c) 2008, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
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

#include <image_geometry/stereo_camera_model.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>

#include <yolo_msgs/msg/bounding_box.hpp>

#include <limits>
#include <memory>
#include <string>
#include <iostream>
#include <vector>

namespace stereo_image_proc
{

class PointCloudNode : public rclcpp::Node
  {
  public:
    explicit PointCloudNode(const rclcpp::NodeOptions & options);

  private:
    // Subscriptions
    image_transport::SubscriberFilter sub_l_image_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> sub_l_info_, sub_r_info_;
    message_filters::Subscriber<stereo_msgs::msg::DisparityImage> sub_disparity_;

    message_filters::Subscriber<yolo_msgs::msg::BoundingBox> sub_bbox_;

    using ExactPolicy = message_filters::sync_policies::ExactTime<
      sensor_msgs::msg::Image,
      sensor_msgs::msg::CameraInfo,
      sensor_msgs::msg::CameraInfo,
      stereo_msgs::msg::DisparityImage,
      yolo_msgs::msg::BoundingBox>;

    using ApproximatePolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image,
      sensor_msgs::msg::CameraInfo,
      sensor_msgs::msg::CameraInfo,
      stereo_msgs::msg::DisparityImage,
      yolo_msgs::msg::BoundingBox>;

    using ExactSync = message_filters::Synchronizer<ExactPolicy>;
    using ApproximateSync = message_filters::Synchronizer<ApproximatePolicy>;

    std::shared_ptr<ExactSync> exact_sync_;
    std::shared_ptr<ApproximateSync> approximate_sync_;

    // Publications
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_points2_;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> pub_targets_;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int64MultiArray>> pub_pixels_;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> pub_avoidance_;

    // Processing state (note: only safe because we're single-threaded!)
    image_geometry::StereoCameraModel model_;
    cv::Mat_<cv::Vec3f> points_mat_;  // scratch buffer

    void connectCb();

    void imageCb(
      const sensor_msgs::msg::Image::ConstSharedPtr & l_image_msg,
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr & l_info_msg,
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr & r_info_msg,
      const stereo_msgs::msg::DisparityImage::ConstSharedPtr & disp_msg,
      const yolo_msgs::msg::BoundingBox::ConstSharedPtr & bbox_msg);
  };

PointCloudNode::PointCloudNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("point_cloud_node", options)
  {
    using namespace std::placeholders;

    // Declare/read parameters
    int queue_size = this->declare_parameter("queue_size", 30);
    bool approx = this->declare_parameter("approximate_sync", false);
    this->declare_parameter("use_system_default_qos", false);
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    // TODO(ivanpauno): Confirm if using point cloud padding in `sensor_msgs::msg::PointCloud2`
    // can improve performance in some cases or not.
    descriptor.description =
      "This parameter avoids using alignment padding in the generated point cloud."
      "This reduces bandwidth requirements, as the point cloud size is halved."
      "Using point clouds without alignment padding might degrade performance for some algorithms.";
    this->declare_parameter("avoid_point_cloud_padding", false, descriptor);
    this->declare_parameter("use_color", true);

    // Synchronize callbacks
    if (approx) {
      approximate_sync_.reset(
        new ApproximateSync(
          ApproximatePolicy(queue_size),
          sub_l_image_, sub_l_info_,
          sub_r_info_, sub_disparity_, sub_bbox_));
      approximate_sync_->registerCallback(
        std::bind(&PointCloudNode::imageCb, this, _1, _2, _3, _4, _5));
    } else {
      exact_sync_.reset(
        new ExactSync(
          ExactPolicy(queue_size),
          sub_l_image_, sub_l_info_,
          sub_r_info_, sub_disparity_, sub_bbox_));
      exact_sync_->registerCallback(
        std::bind(&PointCloudNode::imageCb, this, _1, _2, _3, _4, _5));

    }

    pub_targets_ = create_publisher<std_msgs::msg::Float64MultiArray>("BurnCreamBlimp/targets", 10);
    pub_points2_ = create_publisher<sensor_msgs::msg::PointCloud2>("BurnCreamBlimp/points2", 10);
    pub_pixels_ = create_publisher<std_msgs::msg::Int64MultiArray>("BurnCreamBlimp/pixels", 10);
    pub_avoidance_ = create_publisher<std_msgs::msg::Float64MultiArray>("BurnCreamBlimp/avoidance", 10);

    //Only subscribe if there's a subscription listening to our publisher.
    connectCb();
  }

  // Handles (un)subscribing when clients (un)subscribe
  void PointCloudNode::connectCb()
  {
    // TODO(jacobperron): Add unsubscribe logic when we use graph events
    image_transport::TransportHints hints(this, "raw");
    const bool use_system_default_qos = this->get_parameter("use_system_default_qos").as_bool();
    rclcpp::QoS image_sub_qos = rclcpp::SensorDataQoS();
    if (use_system_default_qos) {
      image_sub_qos = rclcpp::SystemDefaultsQoS();
    }
    const auto image_sub_rmw_qos = image_sub_qos.get_rmw_qos_profile();
    sub_l_image_.subscribe(this, "BurnCreamBlimp/left/image_rect_color", hints.getTransport(), image_sub_rmw_qos);
    sub_l_info_.subscribe(this, "BurnCreamBlimp/left/camera_info", image_sub_rmw_qos);
    sub_r_info_.subscribe(this, "BurnCreamBlimp/right/camera_info", image_sub_rmw_qos);
    sub_disparity_.subscribe(this, "BurnCreamBlimp/disparity", image_sub_rmw_qos);
    sub_bbox_.subscribe(this, "BurnCreamBlimp/bounding_box", image_sub_rmw_qos);
  }

  //check functions
  inline bool isValidPoint(const cv::Vec3f & pt)

  {
    // Check both for disparities explicitly marked as invalid (where OpenCV maps pt.z to MISSING_Z)
    // and zero disparities (point mapped to infinity).
    return pt[2] != image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(pt[2]);
  }

  inline bool bboxCheck(int x, int y, int w, int h, int u, int v)
  {
    bool conditionCheck = u < x + w/2 && u > x - w/2 && v < y + h/2 && v > y - h/2; //in bounding box  
    return conditionCheck;
  }

  inline bool isInBoundingBoxBalloon(const yolo_msgs::msg::BoundingBox::ConstSharedPtr & bbox_msg, int u, int v)
  {
    int64 x_center_balloon= bbox_msg->x_center_balloon;
    int64 y_center_balloon= bbox_msg->y_center_balloon;
    int64 width_balloon= bbox_msg->width_balloon;
    int64 height_balloon= bbox_msg->height_balloon;

    if (x_center_balloon == -1){
      return false;
    } else if (!bboxCheck(x_center_balloon,y_center_balloon,width_balloon,height_balloon,u,v)){
      return false;
    } else{ //in bounding box
      return true;
    }
  }

  inline bool isInBoundingBoxOgoal(const yolo_msgs::msg::BoundingBox::ConstSharedPtr & bbox_msg, int u, int v)
  {
    int64 x_center_o_goal= bbox_msg->x_center_o_goal;
    int64 y_center_o_goal= bbox_msg->y_center_o_goal;
    int64 width_o_goal= bbox_msg->width_o_goal;
    int64 height_o_goal= bbox_msg->height_o_goal;

    if (x_center_o_goal == -1){
      return false;
    } else if (!bboxCheck(x_center_o_goal,y_center_o_goal,width_o_goal,height_o_goal,u,v)){
      return false;
    } else { //in bounding box
      return true;
    }
  }

  inline bool isInBoundingBoxYgoal(const yolo_msgs::msg::BoundingBox::ConstSharedPtr & bbox_msg, int u, int v)
  {
    int64 x_center_y_goal= bbox_msg->x_center_y_goal;
    int64 y_center_y_goal= bbox_msg->y_center_y_goal;
    int64 width_y_goal= bbox_msg->width_y_goal;
    int64 height_y_goal= bbox_msg->height_y_goal;

    if (x_center_y_goal == -1){
      return false;
    } else if (!bboxCheck(x_center_y_goal,y_center_y_goal,width_y_goal,height_y_goal,u,v)){
      return false;
    } else { //in bounding box
      return true;
    }
  }

  //TODO: examine imageCb
  //************Important***************** 
  void PointCloudNode::imageCb(
    const sensor_msgs::msg::Image::ConstSharedPtr & l_image_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & l_info_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & r_info_msg,
    const stereo_msgs::msg::DisparityImage::ConstSharedPtr & disp_msg,
    const yolo_msgs::msg::BoundingBox::ConstSharedPtr & bbox_msg)
  {
      // RCLCPP_INFO(this->get_logger(), "My name jame! 5");
      
      //uncomment for faster operation speeed while no subscirption is present
      // if (pub_points2_->get_subscription_count() == 0u) {
      //   return;
      // }

      // Update the camera model
      model_.fromCameraInfo(l_info_msg, r_info_msg);

      // Calculate point cloud
      const sensor_msgs::msg::Image & dimage = disp_msg->image;
      // The cv::Mat_ constructor doesn't accept a const data data pointer
      // so we remove the constness before reinterpreting into float.
      // This is "safe" since our cv::Mat is const.
      float * data = reinterpret_cast<float *>(const_cast<uint8_t *>(&dimage.data[0]));

      const cv::Mat_<float> dmat(dimage.height, dimage.width, data, dimage.step);
      model_.projectDisparityImageTo3d(dmat, points_mat_, true);
      cv::Mat_<cv::Vec3f> mat = points_mat_;

      // Fill in new PointCloud2 message (2D image-like layout)
      auto points_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
      points_msg->header = disp_msg->header;
      points_msg->height = mat.rows;
      points_msg->width = mat.cols;
      points_msg->is_bigendian = false;
      points_msg->is_dense = false;  // there may be invalid points

      sensor_msgs::PointCloud2Modifier pcd_modifier(*points_msg);

      // Avoidance message initialization 
      auto avoidance_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();

      // Target message initialization 
      auto targets_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();

      // Pixel message initialization 
      auto pixels_msg = std::make_shared<std_msgs::msg::Int64MultiArray>();

      if (!this->get_parameter("avoid_point_cloud_padding").as_bool()) {
        if (this->get_parameter("use_color").as_bool()) {
          // Data will be packed as (DC=don't care, each item is a float):
          //   x, y, z, DC, rgb, DC, DC, DC
          // Resulting step size: 32 bytes
          pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        } else {
          // Data will be packed as:
          //   x, y, z, DC
          // Resulting step size: 16 bytes
          pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
        }
      } else {
        if (this->get_parameter("use_color").as_bool()) {
          // Data will be packed as:
          //   x, y, z, rgb
          // Resulting step size: 16 bytes
          pcd_modifier.setPointCloud2Fields(
            4,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
        } else {
          // Data will be packed as:
          //   x, y, z
          // Resulting step size: 12 bytes
          pcd_modifier.setPointCloud2Fields(
            3,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32);
        }
      }

      sensor_msgs::PointCloud2Iterator<float> iter_x(*points_msg, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(*points_msg, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(*points_msg, "z");
      

      //position vectors for each object (separated to x,y,z vecs and x,y,z)
      std::vector<float> balloon_x_vec;
      std::vector<float> balloon_y_vec;
      std::vector<float> balloon_z_vec;
      std::vector<float> o_goal_x_vec;
      std::vector<float> o_goal_y_vec;
      std::vector<float> o_goal_z_vec;
      std::vector<float> y_goal_x_vec;
      std::vector<float> y_goal_y_vec;
      std::vector<float> y_goal_z_vec;

      //xyz vector of 3 objects, vectors are set to be the size of 3
      //initialize as if there is no objects
      std::vector<float> balloon_xyz;
      balloon_xyz.resize(3);
      balloon_xyz[0] = 1000.0;
      balloon_xyz[1] = 1000.0;
      balloon_xyz[2] = 1000.0;
      std::vector<float> o_goal_xyz;
      o_goal_xyz.resize(3);
      o_goal_xyz[0] = 1000.0;
      o_goal_xyz[1] = 1000.0;
      o_goal_xyz[2] = 1000.0;
      std::vector<float> y_goal_xyz;
      y_goal_xyz.resize(3);
      y_goal_xyz[0] = 1000.0;
      y_goal_xyz[1] = 1000.0;
      y_goal_xyz[2] = 1000.0;


      //balloon pixel in integers
      //initialize as if there is no objects

      std::vector<int64_t> balloon_pixel;
      balloon_pixel.resize(3);
      balloon_pixel[0] = 1000;
      balloon_pixel[1] = 1000;
      balloon_pixel[2] = 0;
      std::vector<int64_t> o_goal_pixel;
      o_goal_pixel.resize(3);
      o_goal_pixel[0] = 1000;
      o_goal_pixel[1] = 1000;
      o_goal_pixel[2] = 0;
      std::vector<int64_t> y_goal_pixel;
      y_goal_pixel.resize(3);
      y_goal_pixel[0] = 1000;
      y_goal_pixel[1] = 1000;
      y_goal_pixel[2] = 0;

      // Define the boundaries for the 9 quadrants
      int numRows = mat.rows;
      int numCols = mat.cols;
      int numRowsPerQuadrant = numRows / 3;
      int numColsPerQuadrant = numCols / 3;

      // Variables to store the mat(v,u)[2] values for each quadrant
     float quadrantSums[3][3];
     int quadrantCounts[3][3] = {0};

      // Set every value to 1000.0f
     for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
              quadrantSums[i][j] = 1000.0f;
        }
      }

      float bad_point = std::numeric_limits<float>::quiet_NaN();
      for (int v = 0; v < mat.rows; ++v) {
        for (int u = 0; u < mat.cols; ++u, ++iter_x, ++iter_y, ++iter_z) {
          if (isValidPoint(mat(v, u))) {
            // x,y,z
            *iter_x = mat(v, u)[0];
            *iter_y = mat(v, u)[1];
            *iter_z = mat(v, u)[2];

            // Determine the quadrant
            int quadrantRow = v / numRowsPerQuadrant;
            int quadrantCol = u / numColsPerQuadrant;

            // Add mat(v,u)[2] to the sum of the corresponding quadrant, also record the number of points that are good
            quadrantSums[quadrantRow][quadrantCol] += mat(v, u)[2];
            quadrantCounts[quadrantRow][quadrantCol]++;

             // check if points belong in any of the 3 objects 
              if (isInBoundingBoxBalloon(bbox_msg,u,v)) {
                // x,y,z
              balloon_x_vec.push_back(mat(v, u)[0]);
              balloon_y_vec.push_back(mat(v, u)[1]);
              balloon_z_vec.push_back(mat(v, u)[2]);
              }
              
              if (isInBoundingBoxOgoal(bbox_msg,u,v)) {
                // x,y,z
              o_goal_x_vec.push_back(mat(v, u)[0]);
              o_goal_y_vec.push_back(mat(v, u)[1]);
              o_goal_z_vec.push_back(mat(v, u)[2]);
              }
      
              if (isInBoundingBoxYgoal(bbox_msg,u,v)) {
                // x,y,z
              y_goal_x_vec.push_back(mat(v, u)[0]);
              y_goal_y_vec.push_back(mat(v, u)[1]);
              y_goal_z_vec.push_back(mat(v, u)[2]);  
              }

            //else the point is bad
          } else {
            *iter_x = *iter_y = *iter_z = bad_point;
          }
        }
      }


    // Calculate the average for each quadrant
    float quadrantAverages[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (quadrantCounts[i][j] > 0) {
                quadrantAverages[i][j] = quadrantSums[i][j] / quadrantCounts[i][j];
            } else {
                quadrantAverages[i][j] = 1000.0f;
            }
        }
    }

    //check balloon xyz centroid
    //Filter outliers using standard deviation

    if (balloon_x_vec.size() != 0) {
    const float numStdDevs = 1.96; // Adjust as needed

    auto calculateFilteredMean = [&](const std::vector<float>& values) -> float {
        float mean = std::accumulate(values.begin(), values.end(), 0.0) / values.size();
        float sumSqDiff = std::accumulate(values.begin(), values.end(), 0.0,
            [mean](float acc, float value) {
                float diff = value - mean;
                return acc + diff * diff;
            });
        float stdDev = std::sqrt(sumSqDiff / values.size());

        std::vector<float> filteredValues;
        std::copy_if(values.begin(), values.end(), std::back_inserter(filteredValues),
            [mean, stdDev, numStdDevs](float value) {
                return std::abs(value - mean) <= numStdDevs * stdDev;
            });

        return filteredValues.empty() ?
            1000.0 :
            std::accumulate(filteredValues.begin(), filteredValues.end(), 0.0) / filteredValues.size();
    };

        balloon_xyz[0] = calculateFilteredMean(balloon_x_vec);
        balloon_xyz[1] = calculateFilteredMean(balloon_y_vec);
        balloon_xyz[2] = calculateFilteredMean(balloon_z_vec);
    } else {
        balloon_xyz[0] = 1000.0;
        balloon_xyz[1] = 1000.0;
        balloon_xyz[2] = 1000.0;
    }

    if (o_goal_x_vec.size() != 0) {
    const float numStdDevs = 1.96; // Adjust as needed

    auto calculateFilteredMean = [&](const std::vector<float>& values) -> float {
        float mean = std::accumulate(values.begin(), values.end(), 0.0) / values.size();
        float sumSqDiff = std::accumulate(values.begin(), values.end(), 0.0,
            [mean](float acc, float value) {
                float diff = value - mean;
                return acc + diff * diff;
            });
        float stdDev = std::sqrt(sumSqDiff / values.size());

        std::vector<float> filteredValues;
        std::copy_if(values.begin(), values.end(), std::back_inserter(filteredValues),
            [mean, stdDev, numStdDevs](float value) {
                return std::abs(value - mean) <= numStdDevs * stdDev;
            });

        return filteredValues.empty() ?
            1000.0 :
            std::accumulate(filteredValues.begin(), filteredValues.end(), 0.0) / filteredValues.size();
    };

        o_goal_xyz[0] = calculateFilteredMean(o_goal_x_vec);
        o_goal_xyz[1] = calculateFilteredMean(o_goal_y_vec);
        o_goal_xyz[2] = calculateFilteredMean(o_goal_z_vec);
    } else {
        o_goal_xyz[0] = 1000.0;
        o_goal_xyz[1] = 1000.0;
        o_goal_xyz[2] = 1000.0;
    }

    if (y_goal_x_vec.size() != 0) {
    const float numStdDevs = 1.96; // Adjust as needed

    auto calculateFilteredMean = [&](const std::vector<float>& values) -> float {
        float mean = std::accumulate(values.begin(), values.end(), 0.0) / values.size();
        float sumSqDiff = std::accumulate(values.begin(), values.end(), 0.0,
            [mean](float acc, float value) {
                float diff = value - mean;
                return acc + diff * diff;
            });
        float stdDev = std::sqrt(sumSqDiff / values.size());

        std::vector<float> filteredValues;
        std::copy_if(values.begin(), values.end(), std::back_inserter(filteredValues),
            [mean, stdDev, numStdDevs](float value) {
                return std::abs(value - mean) <= numStdDevs * stdDev;
            });

        return filteredValues.empty() ?
            1000.0 :
            std::accumulate(filteredValues.begin(), filteredValues.end(), 0.0) / filteredValues.size();
    };

        y_goal_xyz[0] = calculateFilteredMean(y_goal_x_vec);
        y_goal_xyz[1] = calculateFilteredMean(y_goal_y_vec);
        y_goal_xyz[2] = calculateFilteredMean(y_goal_z_vec);
    } else {
        y_goal_xyz[0] = 1000.0;
        y_goal_xyz[1] = 1000.0;
        y_goal_xyz[2] = 1000.0;
    }

      // static_cast<double>(bbox_msg->x_center_balloon - 1280/2),
      // static_cast<double>(bbox_msg->y_center_balloon - 960/2),
      // static_cast<double>(bbox_msg->x_center_o_goal - 1280/2),
      // static_cast<double>(bbox_msg->y_center_o_goal - 960/2),
      // static_cast<double>(bbox_msg->x_center_y_goal - 1280/2),
      // static_cast<double>(bbox_msg->y_center_y_goal - 960/2),
    //check pixels
    if (bbox_msg->x_center_balloon != -1){
        balloon_pixel[0] = bbox_msg->x_center_balloon - (1280/2);
        balloon_pixel[1] = bbox_msg->y_center_balloon - (960/2);
        balloon_pixel[2] = (bbox_msg->height_balloon)*(bbox_msg->width_balloon); //pixel area
    }else{
        balloon_pixel[0] = 1000;
        balloon_pixel[1] = 1000;
        balloon_pixel[2] = 0;
    }

    if (bbox_msg->x_center_o_goal != -1){
        o_goal_pixel[0] = bbox_msg->x_center_o_goal - (1280/2);
        o_goal_pixel[1] = bbox_msg->y_center_o_goal - (960/2);
        o_goal_pixel[2] = (bbox_msg->height_o_goal)*(bbox_msg->width_o_goal); //pixel area
    }else{
        o_goal_pixel[0] = 1000;
        o_goal_pixel[1] = 1000;
        o_goal_pixel[2] = 0;
    }

    if (bbox_msg->x_center_y_goal != -1){
        y_goal_pixel[0] = bbox_msg->x_center_y_goal - (1280/2);
        y_goal_pixel[1] = bbox_msg->y_center_y_goal - (960/2);
        y_goal_pixel[2] = (bbox_msg->height_y_goal)*(bbox_msg->width_y_goal); //pixel area
    }else{
        y_goal_pixel[0] = 1000;
        y_goal_pixel[1] = 1000;
        y_goal_pixel[2] = 0;
    }
    
    //basic averaging working code

    /*
        if (balloon_x_vec.size() != 0){
      float balloon_x_sum = std::accumulate(balloon_x_vec.begin(), balloon_x_vec.end(), 0);
      float balloon_x = balloon_x_sum/balloon_x_vec.size();
      float balloon_y_sum = std::accumulate(balloon_y_vec.begin(), balloon_y_vec.end(), 0);
      float balloon_y = balloon_y_sum/balloon_y_vec.size();
      float balloon_z_sum = std::accumulate(balloon_z_vec.begin(), balloon_z_vec.end(), 0);
      float balloon_z = balloon_z_sum/balloon_z_vec.size();

      balloon_xyz[0] = balloon_x;
      balloon_xyz[1] = balloon_y;
      balloon_xyz[2] = balloon_z;

    }else{
      balloon_xyz[0] = 1000.0;
      balloon_xyz[1] = 1000.0;
      balloon_xyz[2] = 1000.0;
    }

    //check orange goal xyz centroid
    if (o_goal_x_vec.size() != 0){
      float o_goal_x_sum = std::accumulate(o_goal_x_vec.begin(), o_goal_x_vec.end(), 0);
      float o_goal_x = o_goal_x_sum/o_goal_x_vec.size();
      float o_goal_y_sum = std::accumulate(o_goal_y_vec.begin(), o_goal_y_vec.end(), 0);
      float o_goal_y = o_goal_y_sum/o_goal_y_vec.size();
      float o_goal_z_sum = std::accumulate(o_goal_z_vec.begin(), o_goal_z_vec.end(), 0);
      float o_goal_z = o_goal_z_sum/o_goal_z_vec.size();

      o_goal_xyz[0] = o_goal_x;
      o_goal_xyz[1] = o_goal_y;
      o_goal_xyz[2] = o_goal_z;
      
    }else{
      o_goal_xyz[0] = 1000.0;
      o_goal_xyz[1] = 1000.0;
      o_goal_xyz[2] = 1000.0;
    }

    //check yellow goal xyz centroid
    if (y_goal_x_vec.size() != 0){
      //TODO:filter points through Gaussian fiter etc
      float y_goal_x_sum = std::accumulate(y_goal_x_vec.begin(), y_goal_x_vec.end(), 0);
      float y_goal_x = y_goal_x_sum/y_goal_x_vec.size();
      float y_goal_y_sum = std::accumulate(y_goal_y_vec.begin(), y_goal_y_vec.end(), 0);
      float y_goal_y = y_goal_y_sum/y_goal_y_vec.size();
      float y_goal_z_sum = std::accumulate(y_goal_z_vec.begin(), y_goal_z_vec.end(), 0);
      float y_goal_z = y_goal_z_sum/y_goal_z_vec.size();

      y_goal_xyz[0] = y_goal_x;
      y_goal_xyz[1] = y_goal_y;
      y_goal_xyz[2] = y_goal_z;
      
    }else{
      y_goal_xyz[0] = 1000.0;
      y_goal_xyz[1] = 1000.0;
      y_goal_xyz[2] = 1000.0;
    }
    */

    // place all avoidance data in one float64multiarray
    //cast to double

      avoidance_msg->data = {
      static_cast<double>(quadrantAverages[0][0]),
      static_cast<double>(quadrantAverages[0][1]),
      static_cast<double>(quadrantAverages[0][2]),
      static_cast<double>(quadrantAverages[1][0]),
      static_cast<double>(quadrantAverages[1][1]),
      static_cast<double>(quadrantAverages[1][2]),
      static_cast<double>(quadrantAverages[2][0]),
      static_cast<double>(quadrantAverages[2][1]),
      static_cast<double>(quadrantAverages[2][2])
      };


    // place all target data in one float64multiarray
    //cast to double

      targets_msg->data = {
      static_cast<double>(balloon_xyz[0]),
      static_cast<double>(balloon_xyz[1]),
      static_cast<double>(balloon_xyz[2]),
      static_cast<double>(o_goal_xyz[0]),
      static_cast<double>(o_goal_xyz[1]),
      static_cast<double>(o_goal_xyz[2]),
      static_cast<double>(y_goal_xyz[0]),
      static_cast<double>(y_goal_xyz[1]),
      static_cast<double>(y_goal_xyz[2])
      };

    //send pixel value (0,0 as the center of the camera frame)
    //cast to double

      pixels_msg->data = {
      balloon_pixel[0],
      balloon_pixel[1],
      balloon_pixel[2],
      o_goal_pixel[0],
      o_goal_pixel[1],
      o_goal_pixel[2],
      y_goal_pixel[0],
      y_goal_pixel[1],
      y_goal_pixel[2]
      };

      if (this->get_parameter("use_color").as_bool()) {
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*points_msg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*points_msg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*points_msg, "b");

        // Fill in color
        namespace enc = sensor_msgs::image_encodings;
        const std::string & encoding = l_image_msg->encoding;
        if (encoding == enc::MONO8) {
          const cv::Mat_<uint8_t> color(
            l_image_msg->height, l_image_msg->width,
            const_cast<uint8_t *>(&l_image_msg->data[0]),
            l_image_msg->step);
          for (int v = 0; v < mat.rows; ++v) {
            for (int u = 0; u < mat.cols; ++u, ++iter_r, ++iter_g, ++iter_b) {
              uint8_t g = color(v, u);
              *iter_r = *iter_g = *iter_b = g;
            }
          }
        } else if (encoding == enc::RGB8) {
          const cv::Mat_<cv::Vec3b> color(
            l_image_msg->height, l_image_msg->width,
            (cv::Vec3b *)(&l_image_msg->data[0]),
            l_image_msg->step);
          for (int v = 0; v < mat.rows; ++v) {
            for (int u = 0; u < mat.cols; ++u, ++iter_r, ++iter_g, ++iter_b) {
              const cv::Vec3b & rgb = color(v, u);
              *iter_r = rgb[0];
              *iter_g = rgb[1];
              *iter_b = rgb[2];
            }
          }
        } else if (encoding == enc::BGR8) {
          const cv::Mat_<cv::Vec3b> color(
            l_image_msg->height, l_image_msg->width,
            (cv::Vec3b *)(&l_image_msg->data[0]),
            l_image_msg->step);
          for (int v = 0; v < mat.rows; ++v) {
            for (int u = 0; u < mat.cols; ++u, ++iter_r, ++iter_g, ++iter_b) {
              const cv::Vec3b & bgr = color(v, u);
              *iter_r = bgr[2];
              *iter_g = bgr[1];
              *iter_b = bgr[0];
            }
          }
        } else {
          // Throttle duration in milliseconds
          RCUTILS_LOG_WARN_THROTTLE(
            RCUTILS_STEADY_TIME, 30000,
            "Could not fill color channel of the point cloud, "
            "unsupported encoding '%s'", encoding.c_str());
        }
      }
    pub_avoidance_->publish(*avoidance_msg);
    pub_targets_->publish(*targets_msg);
    pub_points2_->publish(*points_msg);
    pub_pixels_->publish(*pixels_msg);
  }

} // namespace stereo_image_proc

// Register node
RCLCPP_COMPONENTS_REGISTER_NODE(stereo_image_proc::PointCloudNode)