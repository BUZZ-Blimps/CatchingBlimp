#include <sstream>

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <chrono>
#include <sstream>
using namespace std;

std::string cameraNs_ = "BurnCreamBlimp"; // Camera ns

int main(int argc, char ** argv)
{
  // Check if video source has been passed as a parameter
  if (argv[1] == NULL) {
    cout << "please give a source input as argument: camera index (0)"<<'\n' ;
    return 1;
  }

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_publisher", options);
  image_transport::ImageTransport it(node);
  image_transport::Publisher pub = it.advertise("/" + cameraNs_ + "/sync/image_raw", 30);

  // Convert the command line parameter index for the video device to an integer
  std::istringstream video_sourceCmd(argv[1]);
  int video_source;

  // Check if it is indeed a number
  if (!(video_sourceCmd >> video_source)) {return 1;}

  cv::VideoCapture cap(video_source);
  // std::string vid = std::string("/dev/elp_sync");
  // cv::VideoCapture cap(vid);

  // Set the desired resolution (2560x960)
  cap.set(cv::CAP_PROP_FRAME_WIDTH, 2560);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 960);
  
  // Check if video device can be opened with the given index
  if (!cap.isOpened()) {return 1;}
  cv::Mat frame;
  std_msgs::msg::Header hdr;
  sensor_msgs::msg::Image::SharedPtr msg;

  // Create a timer to update the header every 1 second (adjust as needed)
    rclcpp::TimerBase::SharedPtr timer = node->create_wall_timer(
        std::chrono::seconds(1), 
        [&pub, &hdr]() {
            // Get the current time in seconds and nanoseconds
            auto current_time = std::chrono::system_clock::now();
            auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(current_time);
            auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(current_time - seconds);

            // Update the header timestamp with the current time
            hdr.stamp.sec = static_cast<int32_t>(seconds.time_since_epoch().count());
            hdr.stamp.nanosec = static_cast<uint32_t>(nanoseconds.count());

            // Update the header frame_id with the current time
            std::stringstream ss;
            ss << "Timer" ;
            hdr.frame_id = ss.str();
        });

  rclcpp::WallRate loop_rate(30); //30 fps
  while (rclcpp::ok()) {
    cap >> frame;
    
    // Check if grabbed frame is actually full with some content
    if (!frame.empty()) {
      // Update the header with the current time before publishing the message
      auto current_time = std::chrono::system_clock::now();
      auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(current_time);
      auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(current_time - seconds);

      // Update the header timestamp with the current time
      hdr.stamp.sec = static_cast<int32_t>(seconds.time_since_epoch().count());
      hdr.stamp.nanosec = static_cast<uint32_t>(nanoseconds.count());

      // Update the header frame_id with the current time
      std::stringstream ss;
      ss << "Timer";
      hdr.frame_id = ss.str();

      msg = cv_bridge::CvImage(hdr, "bgr8", frame).toImageMsg();
      pub.publish(msg);
      cv::waitKey(1);
    }

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
}