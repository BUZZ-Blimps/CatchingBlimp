#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

//The following names can be commented/uncommented based on the blimp that is used
// Define the name of the blimp/robot
//  std::string blimpNameSpace = "BurnCreamBlimp";
//std::string blimpNameSpace = "SillyAhBlimp";
//std::string blimpNameSpace = "TurboBlimp";
//std::string blimpNameSpace = "GameChamberBlimp";
std::string blimpNameSpace = "GravyLongWayBlimp";
// std::string blimpNameSpace = "SuperBeefBlimp";

auto identity_msg = std_msgs::msg::String();
auto log_msg = std_msgs::msg::String();
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class blimp : public rclcpp::Node
{
  public:
    blimp()
    : Node(blimpNameSpace), count_(0)
    {
      // create publishers (7 right now)
      identity_publisher = this->create_publisher<std_msgs::msg::String>("/identify", 10);
      imu_publisher = this->create_publisher<sensor_msgs::msg::Imu>((blimpNameSpace + "/imu").c_str(), 10);
      debug_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>((blimpNameSpace + "/debug").c_str(), 10);
      height_publisher = this->create_publisher<std_msgs::msg::Float64>((blimpNameSpace + "/height").c_str(), 10);
      z_velocity_publisher = this->create_publisher<std_msgs::msg::Float64>((blimpNameSpace + "/z_velocity").c_str(), 10);
      state_machine_publisher = this->create_publisher<std_msgs::msg::Int64>((blimpNameSpace + "/state_machine").c_str(), 10);
      log_publisher = this->create_publisher<std_msgs::msg::String>((blimpNameSpace + "/log").c_str(), 10);

      //create subscribers (10 right now)
      //Base station
      auto_subscription = this->create_subscription<std_msgs::msg::Bool>((blimpNameSpace + "/auto").c_str(), 10, std::bind(&blimp::auto_subscription_callback, this, _1));
      baseBarometer_subscription = this->create_subscription<std_msgs::msg::Float64>((blimpNameSpace + "/baseBarometer").c_str(), 10, std::bind(&blimp::baro_subscription_callback, this, _1));
      calibrateBarometer_subscription = this->create_subscription<std_msgs::msg::Bool>((blimpNameSpace + "/calibrateBarometer").c_str(), 10, std::bind(&blimp::calibrateBarometer_subscription_callback, this, _1));
      grabber_subscription = this->create_subscription<std_msgs::msg::Bool>((blimpNameSpace + "/grabbing").c_str(), 10, std::bind(&blimp::grab_subscription_callback, this, _1));
      shooter_subscription = this->create_subscription<std_msgs::msg::Bool>((blimpNameSpace + "/shooting").c_str(), 10, std::bind(&blimp::shoot_subscription_callback, this, _1));
      motor_subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>((blimpNameSpace + "/motorCommands").c_str(), 10, std::bind(&blimp::kill_subscription_callback, this, _1));
      kill_subscription = this->create_subscription<std_msgs::msg::Bool>((blimpNameSpace + "/killed").c_str(), 10, std::bind(&blimp::motor_subscription_callback, this, _1));
      goal_color_subscription = this->create_subscription<std_msgs::msg::Int64>((blimpNameSpace + "/goal_color").c_str(), 10, std::bind(&blimp::goal_color_subscription_callback, this, _1));
      
      //Offboard ML
      targets_subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>((blimpNameSpace + "/targets").c_str(), 10, std::bind(&blimp::targets_subscription_callback, this, _1));
      pixels_subscription = this->create_subscription<std_msgs::msg::Int64MultiArray>((blimpNameSpace + "/pixels").c_str(), 10, std::bind(&blimp::pixels_subscription_callback, this, _1));
      avoidance_subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>((blimpNameSpace + "/avoidance").c_str(), 10, std::bind(&blimp::avoidance_subscription_callback, this, _1));

      timer = this->create_wall_timer(
      1000ms, std::bind(&blimp::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      identity_publisher->publish(identity_msg);
    }
    rclcpp::TimerBase::SharedPtr timer;

    void publish_log(const char *message) {
        snprintf(log_msg.data.data, BUFFER_LEN, "%s", message);
        log_msg.data.size = strlen(log_msg.data.data);
        log_msg.data.capacity = BUFFER_LEN;
        log_publisher->publish(log_msg)
    }

    void auto_subscription_callback(const std_msgs::msg::Bool & msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
        if (msg.data) {
            if (blimp_state == manual) {
                publish_log("Activating Auto Mode");
            }
            blimp_state = autonomous;
        } else {
            if (blimp_state == autonomous) {
                publish_log("Going Manual for a Bit...");
            }
            blimp_state = manual;
        }
    }


    void calibrateBarometer_subscription_callback(const std_msgs::msg::Bool & msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
        calibrateBaro = msg.data;
        const char * boolAsConstCharPtr = calibrateBaro ? "true" : "false";

        // Barometer Calibration
        if (calibrateBaro == true) {
            baroCalibrationOffset = BerryIMU.comp_press - baseBaro;

            std::string floatAsString = std::to_string(BerryIMU.comp_press);
            const char* floatAsConstCharPtr = floatAsString.c_str();
            publish_log(floatAsConstCharPtr);
            std::string floatAsString2 = std::to_string(baseBaro);
            const char* floatAsConstCharPtr2 = floatAsString2.c_str();
            publish_log(floatAsConstCharPtr2);
            std::string floatAsString3 = std::to_string(baroCalibrationOffset);
            const char* floatAsConstCharPtr3 = floatAsString3.c_str();
            publish_log(floatAsConstCharPtr3);
            publish_log("Calibrating Barometer");
        }
    }

    void baro_subscription_callback(const std_msgs::msg::Float64 & msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
        baseBaro = msg.data;

        //heartbeat
        //update last message time
        lastMsgTime = micros()/MICROS_TO_SEC;

        //If teensy comes out of lost blimp_state, put it in manual control mode
        if (blimp_state == lost) {
            blimp_state = manual;
        }

  
    }

    void grab_subscription_callback(const std_msgs::msg::Bool & msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());

        if (grabCom == 0 && msg.data) {
            grabCom = 1;
            publish_log("Going for a catch...");
        } else if (grabCom == 1 && !msg.data) {
            grabCom = 0;
            publish_log("Hopefully I got a balloon!");
        }
    }

    void kill_subscription_callback(const std_msgs::msg::Bool & msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());

        if (msg.data == true) {
            publish_log("I'm ded xD");
            motorControl.update(0,0,0,0,0);
            bool leftReady = leftGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, 0, 0);
            bool rightReady = rightGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, 0, 0);
            leftGimbal.updateGimbal(leftReady && rightReady);
            rightGimbal.updateGimbal(leftReady && rightReady);
        }
    }

    void shoot_subscription_callback(const std_msgs::msg::Bool & msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());

        if (shootCom == 0 && msg.data) {
            shootCom = 1;
            publish_log("I'm shooting my shot...");
        } else if (shootCom == 1 && !msg.data) {
            shootCom = 0;
            publish_log("What a shot!");
        }
    }

    void motor_subscription_callback(const std_msgs::msg::Float64MultiArray & msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());

        //commands from basestation
        forward_msg = msg.data.data[3];
        up_msg = msg.data.data[1];
        yaw_msg = msg.data.data[0];
        translation_msg = msg.data.data[2];

        // char motorCommands[100];  // Size depending on the expected maximum length of your combined string
        // sprintf(motorCommands, "Teensy Motor Commands\nYaw: %.2f\nUp: %.2f\nTranslation: %.2f\nForward: %.2f\n", yaw_msg, up_msg, translation_msg, forward_msg);
        // publish_log(motorCommands);
    }

    void goal_color_subscription_callback(const std_msgs::msg::Int64 & msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());

        int goal_color = msg.data;
        if (goalColor != orange && goal_color == 0) {
            goalColor = orange;
            publish_log("Goal Color changed to Orange");
        } else if (goalColor != yellow && goal_color == 1) {
            goalColor = yellow;
            publish_log("Goal Color changed to Yellow");
        }
    }

    void avoidance_subscription_callback(const std_msgs::msg::Float64MultiArray & msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());

        //3 objects with xyz (9 elements in total)
        for (size_t i = 0; i < 9; ++i) {
            avoidance[i] = msg.data.data[i];
        }
    }

    void targets_subscription_callback(const std_msgs::msg::Float64MultiArray & msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());

        // object of interest with xyz (3 elements in total)
        for (size_t i = 0; i < 3; ++i) {
            targets[i] = msg.data.data[i];
        }
    }

    void pixels_subscription_callback(const std_msgs::msg::Int64MultiArray & msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());

        //3 objects with xyz (9 elements in total)
        for (size_t i = 0; i < 9; ++i) {
            pixels[i] = pixels_msg->data.data[i];
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr identity_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr height_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr z_velocity_publisher;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr state_machine_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_publisher;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr auto_subscription;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr baseBarometer_subscription;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr calibrateBarometer_subscription;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr grabber_subscription;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shooter_subscription;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr motor_subscription;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr kill_subscription;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr goal_color_subscription;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr targets_subscription;
    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr pixels_subscription;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr avoidance_subscription;
    

    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<blimp>());
  rclcpp::shutdown();
  return 0;
}