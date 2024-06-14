#ifndef STEREO_COMBINED_HPP
#define STEREO_COMBINED_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>

#include <camera_info_manager/camera_info_manager.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <image_geometry/stereo_camera_model.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>

//Messages
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>

//Message filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <yolo_msgs/msg/bounding_box.hpp>

#include "edge_aware.hpp"
#include "stereo_processor.hpp"

class StereoCombined : public rclcpp::Node {
private:
    //Splitter
    void syncImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    //Bounding box sub
    void boundingBoxCallback(const yolo_msgs::msg::BoundingBox::SharedPtr bounding_box_msg);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr syncImageSub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr leftImagePub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rightImagePub_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfoLeft_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfoRight_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr leftCameraInfoPub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rightCameraInfoPub_;
    std::string cameraNs_ ; // Camera ns
    
    //Debayer
    int debayer_;

    int debayer_bilinear_ = 0;
    int debayer_edgeaware_ = 1;
    int debayer_edgeaware_weighted_ = 2;
    int debayer_vng_ = 3;

    image_transport::Publisher pub_mono_left_;
    image_transport::Publisher pub_color_left_;

    image_transport::Publisher pub_mono_right_;
    image_transport::Publisher pub_color_right_;

    //Rectifier
    int queue_size_;
    int interpolation;
    image_transport::Publisher pub_rect_mono_left_;
    image_transport::Publisher pub_rect_color_left_;
    image_transport::Publisher pub_rect_mono_right_;
    image_transport::Publisher pub_rect_color_right_;

    // image_geometry::PinholeCameraModel pinhole_model_left_;
    // image_geometry::PinholeCameraModel pinhole_model_right_;

    // Disparity
    // Publications
    std::shared_ptr<rclcpp::Publisher<stereo_msgs::msg::DisparityImage>> pub_disparity_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> pub_disparity_img_;

    // // Handle to parameters callback
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

    // // Processing state (note: only safe because we're single-threaded!)
    image_geometry::StereoCameraModel stereo_model_;

    // // contains scratch buffers for block matching
    StereoProcessor block_matcher_;

    rcl_interfaces::msg::SetParametersResult parameterSetCb(const std::vector<rclcpp::Parameter> & parameters);

    //Point cloud
    rclcpp::Subscription<yolo_msgs::msg::BoundingBox>::SharedPtr boundingBoxSub_;
    yolo_msgs::msg::BoundingBox::SharedPtr bbox_msg;

    // Publications
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_points2_;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> pub_targets_;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int64MultiArray>> pub_pixels_;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> pub_avoidance_;

    cv::Mat_<cv::Vec3f> points_mat_;  // scratch buffer

public:
    StereoCombined();
};

// Some helper functions for adding a parameter to a collection
static void add_param_to_map(
    std::map<std::string, std::pair<int, rcl_interfaces::msg::ParameterDescriptor>> & parameters,
    const std::string & name,
    const std::string & description,
    const int default_value,
    const int from_value,
    const int to_value,
    const int step) {
    rcl_interfaces::msg::IntegerRange integer_range;
    integer_range.from_value = from_value;
    integer_range.to_value = to_value;
    integer_range.step = step;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.integer_range = {integer_range};
    parameters[name] = std::make_pair(default_value, descriptor);
}

static void add_param_to_map(
    std::map<std::string, std::pair<double, rcl_interfaces::msg::ParameterDescriptor>> & parameters,
    const std::string & name,
    const std::string & description,
    const double default_value,
    const double from_value,
    const double to_value,
    const double step) {
    rcl_interfaces::msg::FloatingPointRange floating_point_range;
    floating_point_range.from_value = from_value;
    floating_point_range.to_value = to_value;
    floating_point_range.step = step;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.floating_point_range = {floating_point_range};
    parameters[name] = std::make_pair(default_value, descriptor);
}

//Point cloud functions
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

  inline bool isInBoundingBoxBalloon(const yolo_msgs::msg::BoundingBox::ConstSharedPtr &bbox_msg, int u, int v)
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

  inline bool isInBoundingBoxOgoal(const yolo_msgs::msg::BoundingBox::ConstSharedPtr &bbox_msg, int u, int v)
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

  inline bool isInBoundingBoxYgoal(const yolo_msgs::msg::BoundingBox::ConstSharedPtr &bbox_msg, int u, int v)
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

#endif
