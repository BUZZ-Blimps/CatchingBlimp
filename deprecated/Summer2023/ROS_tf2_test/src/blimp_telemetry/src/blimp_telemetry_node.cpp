#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>


using std::placeholders::_1;

class TelemetryPublisher : public rclcpp::Node {
public:
    TelemetryPublisher() : Node("blimp_telemetry_node") {

    // Declare and acquire `blimpname` parameter
     blimpname_ = this->declare_parameter<std::string>("blimpname", "BurnCreamBlimp");

    // Initialize the transform broadcaster
     tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
     
    // subscribe to a topic 
    // callback function on each message
     std::ostringstream stream;
     stream << "/" << blimpname_.c_str() << "/imu";
     std::string topic_name = stream.str();

     imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>("/BurnCreamBlimp/imu", 10, std::bind(&TelemetryPublisher::imu_callback, this, _1));
     RCLCPP_INFO(this->get_logger(), "Telemetry publisher running");

    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const {
     geometry_msgs::msg::TransformStamped t;

     rclcpp::Time msg_time = msg->header.stamp;
     RCLCPP_INFO(this->get_logger(), "Timestamp: '%f'", msg_time.seconds());

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = msg_time;
    t.header.frame_id = "world";
    t.child_frame_id = blimpname_.c_str();


    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;

    
    t.transform.rotation.x = msg->orientation.x;
    t.transform.rotation.y = msg->orientation.y;
    t.transform.rotation.z = msg->orientation.z;
    t.transform.rotation.w = msg->orientation.w;

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string blimpname_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TelemetryPublisher>());
    rclcpp::shutdown();
    return 0;
}
