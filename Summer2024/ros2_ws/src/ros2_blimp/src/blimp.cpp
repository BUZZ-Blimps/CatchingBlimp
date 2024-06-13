//C++ includes
#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include <string>
#include <stdio.h>
#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"

//Message type includes
#include <std_msgs/msg/string.hpp> //include the message type that needs to be published (teensy data)
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>

//Includes for main.cpp
#include "MotorControl.h"
#include "OPI_IMU.h"
#include "Madgwick_Filter.h"
#include "baro_acc_kf.h"
#include "accelGCorrection.h"
#include "PID.h"
#include "EMAFilter.h"
#include "Kalman_Filter_Tran_Vel_Est.h"
#include "BangBang.h"
#include "optical_ekf.h"
#include "gyro_ekf.h"
#include "tripleBallGrabber.h"
#include "Gimbal.h"
#include <wiringPi.h>

//blimp mode
#define BLIMP_COLOR               red      //either red or blue
#define GOAL_COLOR                orange    //either orange or yellow

#define CEIL_HEIGHT_FROM_START    4 //unused 

//debug mode
#define ZERO_MODE                 false
#define GIMBAL_DEBUG              false
#define MOTORS_OFF                false

//optional controllers
#define USE_EST_VELOCITY_IN_MANUAL  false    //use false to turn off the velosity control to see the blimp's behavior 
#define USE_OBJECT_AVOIDENCE      false     //use false to turn off the obstacle avoidance 

//catch search time after one
#define MAX_SEARCH_WAIT_AFTER_ONE     80.0    //max searching 
#define GAME_BALL_WAIT_TIME_PENALTY   0    //should be set to 20, every catch assumed to be 20 seconds long  

//number of catches attempted
#define TOTAL_ATTEMPTS            2    // attempts at catching 
#define MAX_ATTEMPTS              5    //should be set to 5

//flight area parameters
#define CEIL_HEIGHT               4      //m
#define FLOOR_HEIGHT              1    //m

#define MAX_HEIGHT                2    //m  (unused)
#define GOAL_HEIGHT               6.5   //m
#define GOAL_HEIGHT_DEADBAND      0.4   //m

//distance triggers
#define GOAL_DISTANCE_TRIGGER    1.3  //m distance for blimp to trigger goal score 	
#define BALL_GATE_OPEN_TRIGGER   2    //m distance for blimp to open the gate 	
#define BALL_CATCH_TRIGGER       1.2  //m distance for blimp to start the open-loop control
#define AVOID_TRIGGER       0.8  //m distance for blimp to start the open-loop control


//object avoidence motor coms
#define FORWARD_AVOID             125  // 25% throttle
#define YAW_AVOID                 10	 // deg/s
#define UP_AVOID                  20   // % throttle 

//autonomy tunning parameters
// the inputs are bounded from -2 to 2, yaw is maxed out at 120 deg/s
#define GAME_BALL_YAW_SEARCH      -7  // deg/s
#define GAME_BALL_FORWARD_SEARCH  130 // 30% throttle 
#define GAME_BALL_VERTICAL_SEARCH 120  // m/s


#define GAME_BALL_CLOSURE_COM     180  //approaching at 20% throttle cap
#define GAME_BALL_APPROACH_ANGLE  40  //approach magic number (TODO: reset)
#define GAME_BaLL_X_OFFSET        80   //offset magic number (TODO: reset)

#define CATCHING_FORWARD_COM      350  //catching at 50% throttle 
#define CATCHING_UP_COM           50  //damp out pitch

#define CAUGHT_FORWARD_COM        -300  //go back so that the game ball gets to the back 
#define CAUGHT_UP_COM             -40

#define GOAL_YAW_SEARCH           15   
#define GOAL_FORWARD_SEARCH       150  //200 40% throttle
#define GOAL_UP_VELOCITY          450

#define GOAL_CLOSURE_COM          125  //forward command 25% throttle
#define GOAL_X_OFFSET             80  
#define GOAL_APPROACH_ANGLE       70  //height alignment (approach down)

//goal alignment test
#define ALIGNING_YAW_COM           10 //test
#define ALIGNING_FORWARD_COM       100 //test
#define ALIGNING_UP_COM            100 //test
#define ALIGNING_TRANSLATION_COM   300 //test


#define SCORING_YAW_COM           0
#define SCORING_FORWARD_COM       400 //40% throttle
#define SCORING_UP_COM            80

#define SHOOTING_FORWARD_COM      400  //counter back motion 
#define SHOOTING_UP_COM           100
//counter moment (right now we do want to shoot up because ball sinks)

#define SCORED_FORWARD_COM        -250
#define SCORED_UP_COM             -50

//sensor and controller rates
#define FAST_SENSOR_LOOP_FREQ           100.0
#define BARO_LOOP_FREQ                  50.0
#define STATE_MACHINE_FREQ              30.0
#define OPTICAL_LOOP_FREQ               55.0

#define DIST_CONSTANT             0.002

#define GYRO_X_CONSTANT           480.0
#define GYRO_YAW_CONSTANT         0

#define GYRO_Y_CONSTANT           323.45

//motor timeout before entering lost state
#define TEENSY_WAIT_TIME          2.0

#define BALL_APPROACH_THRESHOLD   2500
#define BALL_CATCH_THRESHOLD      62000

//**************** TEENSY PINOUT ****************//
#define L_Pitch                   2                    
#define L_Yaw                     3              
#define R_Pitch                   4                
#define R_Yaw                     5              

#define L_Pitch_FB                23                    
#define L_Yaw_FB                  22                  
#define R_Pitch_FB                21                    
#define R_Yaw_FB                  20                  

#define GATE_S                    15                

#define PWM_R                     6              
#define PWM_G                     9              
#define PWM_L                     14              

#define OF_CS                     10              
//***********************************************//

//constants
#define MICROS_TO_SEC             1000000.0
#define BUFFER_LEN                512
#define MAX_TARGETS               100
#define MIN_MOTOR                 1000
#define MAX_MOTOR                 2000


enum autoState {
    searching,
    approach,
    catching,
    caught,
    goalSearch,
    approachGoal,  //To Do: add goal aligntment (PID) in if, or add another state "align"
    scoringStart,
    shooting,
    scored
};

enum blimpState {
    manual,
    autonomous,
    lost,
};

int blimp_state = lost;

enum grabberState {
    opened,
    closed,
};

enum blimpType {
    blue,
    red
};

enum goalType {
    orange,
    yellow
};

enum gameballType{
    green,
    pink
};

//global variables
int auto_state = searching;

//blimp game parameters
int blimpColor = BLIMP_COLOR;
int goalColor = GOAL_COLOR;

//timeout message time
double lastMsgTime = -1.0;

//msg for commands
float forward_msg = 0;
float yaw_msg = 0;
float up_msg = 0;
float translation_msg = 0;

//sensor data
float pitch = 0;
float yaw = 0;
float roll = 0;

float rotation = 0;
float ground_pressure = 0;

//timers for state machine
double searchingTimeStart = 0.0;
bool backingUp = false;
double searchTime = 30000;
double backupTime = 6000;

double approachTimeStart = 0;
double approachTimeMax = 10000;   //ms

double catchMemoryTimer = 0;
double catchTimeStart = 0;
double catchTime = 2300;        //ms

double caughtTimeStart = 0;
double caughtTime = 3000;       //ms

double scoreTimeStart = 0;
double scoreTime = 1000;        //ms

double shootingTimeStart = 0;
double shootingTime = 4500;//2500;     //ms

double scoredTimeStart = 0;
double scoredTime = 4500 ;//2500;       //ms

double firstMessageTime = 0.0;

double lastCatch = 0.0;

//grabber data
int catches = 0;
int shoot = 0;
int grab = 0;
int shootCom = 0;
int grabCom = 0;

//interupt pin setup at timing for long range ultrasonic
volatile unsigned long pulseInTimeBegin = micros();
volatile unsigned long pulseInTimeEnd = micros();
volatile bool newPulseDurationAvailable = false;
const u_int8_t interruptPin = 22;
float z_distance_m = 0;

//avoidence data
int quadrant = 10;

//base station baro
float baseBaro = 0.0;

//base station calibrate baro
bool calibrateBaro = false;

//base station baro offset
float baroCalibrationOffset = 0.0;

//direction from last ball search
bool wasUp = true;

//corrected baro
float actualBaro = 0.0;

float searchYawDirection = -1;

float goalYawDirection = -1;

// char log_buf[BUFFER_LEN];

//avoidance data (9 quadrants), targets data and pixel data (balloon, orange goal, yellow goal)
//1000 means object is not present
std::vector<double> avoidance = {1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0};
std::vector<double> targets = {1000.0, 1000.0, 1000.0};
std::vector<int64_t> pixels = {1000, 1000, 0, 1000, 1000, 0, 1000, 1000, 0};


//------------------MICRO ROS publishers/subscribers--------------
//ROS node


//message types: String Bool Float32 Float32 MultiArray
//message topics : /auto /baseBarometer /blimpID /grabbing /killed /motorCommands /shooting /identify /imu /goal_color /state_machine

//service response&request
// test_msgs__srv__BasicTypes_Response res;
// test_msgs__srv__BasicTypes_Request req;
bool check = false;

//boolean messages
// auto auto_msg = std_msgs::msg::Bool();
// auto grab_msg = std_msgs::msg::Bool();
// auto shoot_msg = std_msgs::msg::Bool();
// auto kill_msg = std_msgs::msg::Bool();
// auto calibration_msg = std_msgs::msg::Bool();

//int64 message
// auto goal_color_msg = std_msgs::msg::Int64();
// auto state_machine_msg = std_msgs::msg::Int64();

//float64 message
// auto baro_msg = std_msgs::msg::Float64();
// auto height_msg = std_msgs::msg::Float64();
// auto z_velocity_msg = std_msgs::msg::Float64();

//float64multiarray and int64multiarray messages
// auto motor_msg = std_msgs::msg::Float64MultiArray();
// auto debug_msg = std_msgs::msg::Float64MultiArray();
// auto avoidance_msg = std_msgs::msg::Float64MultiArray();
// auto targets_msg = std_msgs::msg::Float64MultiArray();
// auto pixels_msg = std_msgs::msg::Float64MultiArray();

//String messages
// auto identity_msg = std_msgs::msg::String();
// auto log_msg = std_msgs::msg::String();

//sensor message
// auto imu_msg = sensor_msgs::msg::Imu();

int counter = 0;
bool last_connected = false;
bool last_lost = true;

//Global variables
//sensor fusion objects
servo Servo_L;
servo Servo_R;
brushless Brushless_L;
brushless Brushless_R;
OPI_IMU BerryIMU;
Madgwick_Filter madgwick;
BaroAccKF kf;
AccelGCorrection accelGCorrection;
// Optical_Flow Flow;
Kalman_Filter_Tran_Vel_Est kal_vel;
// OpticalEKF xekf(DIST_CONSTANT, GYRO_X_CONSTANT, GYRO_YAW_CONSTANT);
// OpticalEKF yekf(DIST_CONSTANT, GYRO_Y_CONSTANT, 0);
GyroEKF gyroEKF;

//Gimbal leftGimbal(yawPin, pitchPin, motorPin, newDeadband, newTurnOnCom, newMinCom, newMaxCom);
MotorControl motorControl;
Gimbal leftGimbal;
Gimbal rightGimbal;

//Manual PID control
PID verticalPID(350, 0, 0);  //not used for now due to baro reading malfunction
PID yawPID(12.0, 0, 0);  //can also tune kd with a little overshoot induced
PID forwardPID(300, 0, 0);  //not used
PID translationPID(300, 0, 0); //not used

//Auto PID control (output fed into manual controller)
PID yPID(0.8,0,0);    //TODO:retune these (can also be in pixels depends on which one performs better) 0.0075 for pixel PID
PID xPID(0.035,0,0);       //TODO:retune these 0.162 for pixel PID

//Goal positioning controller
BangBang goalPositionHold(GOAL_HEIGHT_DEADBAND, GOAL_UP_VELOCITY); //Dead band, velocity to center itself

//pre process for accel before vertical kalman filter
EMAFilter verticalAccelFilter(0.05);

//filter on yaw gyro
EMAFilter yawRateFilter(0.2);
EMAFilter rollRateFilter(0.5);

//Low pass filter for computer vision parameters
EMAFilter xFilter(0.5);
EMAFilter yFilter(0.5);
EMAFilter zFilter(0.5);
EMAFilter areaFilter(0.5);

//baro offset computation from base station value
EMAFilter baroOffset(0.5);

//roll offset computation from imu
EMAFilter rollOffset(0.5);

//ball grabber object
TripleBallGrabber ballGrabber;

//timing global variables for each update loop
float lastSensorFastLoopTick = 0.0;
float lastStateLoopTick = 0.0;
float lastBaroLoopTick = 0.0;
float lastOpticalLoopTick = 0.0;

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

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class blimp:public rclcpp::Node
{
    public:
        blimp():Node(blimpNameSpace), count_(0){
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
            motor_subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>((blimpNameSpace + "/motorCommands").c_str(), 10, std::bind(&blimp::motor_subscription_callback, this, _1)); 
            kill_subscription = this->create_subscription<std_msgs::msg::Bool>((blimpNameSpace + "/killed").c_str(), 10, std::bind(&blimp::kill_subscription_callback, this, _1));
            goal_color_subscription = this->create_subscription<std_msgs::msg::Int64>((blimpNameSpace + "/goal_color").c_str(), 10, std::bind(&blimp::goal_color_subscription_callback, this, _1));
            
            // // Offboard ML
            targets_subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>((blimpNameSpace + "/targets").c_str(), 10, std::bind(&blimp::targets_subscription_callback, this, _1));
            pixels_subscription = this->create_subscription<std_msgs::msg::Int64MultiArray>((blimpNameSpace + "/pixels").c_str(), 10, std::bind(&blimp::pixels_subscription_callback, this, _1));
            avoidance_subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>((blimpNameSpace + "/avoidance").c_str(), 10, std::bind(&blimp::avoidance_subscription_callback, this, _1));

            timer_imu = this->create_wall_timer(
            10ms, std::bind(&blimp::imu_callback, this));

            timer_baro = this->create_wall_timer(
            20ms, std::bind(&blimp::baro_callback, this));

            timer_state_machine = this->create_wall_timer(
            33ms, std::bind(&blimp::state_machine_callback, this));

            timer_ = this->create_wall_timer(
                1000ms, std::bind(&blimp::timer_callback, this));

            BerryIMU.OPI_IMU_Setup();
            firstMessageTime = micros()/MICROS_TO_SEC;

            // Start Servos
            Servo_L.servo_setup(0);
            Servo_R.servo_setup(2);
            Servo_L.servo_angle(0);
            Servo_R.servo_angle(180);

            // // initialize
            ballGrabber.ballgrabber_init(GATE_S, PWM_G);
            leftGimbal.gimbal_init(L_Yaw, L_Pitch, PWM_L, 25, 30, MIN_MOTOR, MAX_MOTOR, 45, 0.5);
            rightGimbal.gimbal_init(R_Yaw, R_Pitch, PWM_R, 25, 30, MIN_MOTOR, MAX_MOTOR, 135, 0.5);

            //Start Brushless
            Brushless_L.brushless_setup(5);
            Brushless_R.brushless_setup(16);
            Brushless_L.brushless_thrust(1500);
            Brushless_R.brushless_thrust(1500);
        }
        
private: 
        //timer call back
    void timer_callback() {
        // RCLC_UNUSED(last_call_time);
        BerryIMU.IMU_read();
        auto identity_msg = std_msgs::msg::String();
        identity_msg.data = blimpNameSpace;
        identity_publisher->publish(identity_msg);
    }

    void imu_callback()
    {
        auto imu_msg = sensor_msgs::msg::Imu();
        //read sensor values and update madgwick
        BerryIMU.IMU_read();
        BerryIMU.IMU_ROTATION(rotation); // Rotate IMU
        madgwick.Madgwick_Update(BerryIMU.gyr_rateXraw, BerryIMU.gyr_rateYraw, BerryIMU.gyr_rateZraw, BerryIMU.AccXraw, BerryIMU.AccYraw, BerryIMU.AccZraw);

        //publish imu data
        // snprintf(imu_msg.header.frame_id.data, BUFFER_LEN, blimpNameSpace.c_str());
        // imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);
        // imu_msg.header.frame_id.capacity = BUFFER_LEN;

        unsigned int now = micros();
        imu_msg.header.stamp.sec = (unsigned int)((double)now/1000000);
        imu_msg.header.stamp.nanosec = (now % 1000000) * 1000;

        //Estimated body orentation (quaternion)
        imu_msg.orientation.w = madgwick.q1;
        imu_msg.orientation.x = madgwick.q2;
        imu_msg.orientation.y = madgwick.q3;
        imu_msg.orientation.z = madgwick.q4;

        //Estimated body frame angular velocity from gyro
        imu_msg.angular_velocity.x = BerryIMU.gyr_rateXraw;
        imu_msg.angular_velocity.y = BerryIMU.gyr_rateYraw;
        imu_msg.angular_velocity.z = BerryIMU.gyr_rateZraw;

        //Estimated body frame acceleration from accelerometer
        imu_msg.linear_acceleration.x = BerryIMU.AccXraw;
        imu_msg.linear_acceleration.y = BerryIMU.AccYraw;
        imu_msg.linear_acceleration.z = BerryIMU.AccZraw;

        imu_publisher->publish(imu_msg);

        //get orientation from madgwick
        pitch = madgwick.pitch_final;
        roll = madgwick.roll_final;
        yaw = madgwick.yaw_final;

        //compute the acceleration in the barometers vertical reference frame
        accelGCorrection.updateData(BerryIMU.AccXraw, BerryIMU.AccYraw, BerryIMU.AccZraw, pitch, roll);

        //run the prediction step of the vertical velecity kalman filter
        double dt = 10/1000;
        kf.predict(dt);
        // xekf.predict(dt);
        // yekf.predict(dt);
        gyroEKF.predict(dt);

        //pre filter accel before updating vertical velocity kalman filter
        verticalAccelFilter.filter(accelGCorrection.agz);

        //update vertical velocity kalman filter acceleration
        kf.updateAccel(verticalAccelFilter.last);

        //update filtered yaw rate
        yawRateFilter.filter(BerryIMU.gyr_rateZraw);

        //perform gyro update
        // gyroEKF.updateGyro(BerryIMU.gyr_rateXraw*3.14/180, BerryIMU.gyr_rateYraw*3.14/180, BerryIMU.gyr_rateZraw*3.14/180);
        // gyroEKF.updateAccel(BerryIMU.AccXraw, BerryIMU.AccYraw, BerryIMU.AccZraw);

        // xekf.updateAccelx(-accelGCorrection.agx);
        // xekf.updateGyroX(gyroEKF.pitchRate - gyroEKF.pitchRateB);
        // xekf.updateGyroZ(BerryIMU.gyr_rateZraw);

        // yekf.updateAccelx(-accelGCorrection.agy);
        // yekf.updateGyroX(gyroEKF.rollRate - gyroEKF.rollRateB);
        // yekf.updateGyroZ(BerryIMU.gyr_rateZraw);

        // Serial.print(">Z:");
        // Serial.println(xekf.z);

        // Serial.print(">Opt:");
        // Serial.println(xekf.opt);

        // Serial.print(">gyro x:");
        // Serial.println(xekf.gyrox);

        // Serial.print(">Ax:");
        // Serial.println(xekf.ax);

        // Serial.print(">Velocity:");
        // Serial.println(xekf.v);

        // Serial.print(">Yrate:");
        // Serial.println(yawRateFilter.last);

        // Serial.print(">zVel:");
        // Serial.println(kf.v);

        // kal_vel.predict_vel();
        // kal_vel.update_vel_acc(-accelGCorrection.agx/9.81, -accelGCorrection.agy/9.81);
    }


    void baro_callback()
    {
        auto height_msg = std_msgs::msg::Float64();
        auto z_velocity_msg = std_msgs::msg::Float64();
        // get most current imu values
        BerryIMU.IMU_read();

        //update kalman with uncorreced barometer data
        kf.updateBaro(BerryIMU.alt);

        //compute the corrected height with base station baro data and offset
        if (baseBaro != 0) {
            actualBaro = 44330 * (1 - pow(((BerryIMU.comp_press - baroCalibrationOffset)/baseBaro), (1/5.255))); // In meters Base Baro is the pressure

            //publish Height
            height_msg.data = actualBaro;
            // height_msg.data = BerryIMU.comp_press;  //only for debug
            height_publisher->publish(height_msg);

        } else {
            actualBaro = 1000;
        }

        // Add Z Velocity to a Message and Publish
        z_velocity_msg.data = kf.v;

        // if (check == false){
        //   z_velocity_msg.data = 0;
        // }else{
        //   z_velocity_msg.data = 0;
        // }


        // xekf.updateBaro(CEIL_HEIGHT_FROM_START-actualBaro);
        // yekf.updateBaro(CEIL_HEIGHT_FROM_START-actualBaro);
        z_velocity_publisher->publish(z_velocity_msg);
    }


    void state_machine_callback()
    {
        auto state_machine_msg = std_msgs::msg::Int64();
        auto debug_msg = std_msgs::msg::Float64MultiArray();
        double dt = 33/1000;
        //control inputs
        float forwardCom = 0.0;
        float upCom = 0.0;
        float yawCom = 0.0;
        float translationCom = 0.0;

        //object avoidence commands to overide search and computer vition
        float forwardA = 0.0;
        float upA = 0.0;
        float yawA = 0.0;

        //set avoidence command based on quadrant that contains object to avoid
        switch (quadrant) {
        case 1:
            forwardA = -FORWARD_AVOID;
            upA = -UP_AVOID;
            yawA = -YAW_AVOID;
            break;
        case 2:
            forwardA = -FORWARD_AVOID;
            upA = -UP_AVOID;
            yawA = 0;
            break;
        case 3:
            forwardA = -FORWARD_AVOID;
            upA = -UP_AVOID;
            yawA = YAW_AVOID;
            break;
        case 4:
            forwardA = -FORWARD_AVOID;
            upA = 0;
            yawA = -YAW_AVOID;
            break;
        case 5:
            forwardA = -FORWARD_AVOID;
            upA = 0;
            yawA = 0;
            break;
        case 6:
            forwardA = -FORWARD_AVOID;
            upA = 0;
            yawA = YAW_AVOID;
            break;
        case 7:
            forwardA = -FORWARD_AVOID;
            upA = UP_AVOID;
            yawA = -YAW_AVOID;
            break;
        case 8:
            forwardA = -FORWARD_AVOID;
            upA = UP_AVOID;
            yawA = 0;
            break;
        case 9:
            forwardA = -FORWARD_AVOID;
            upA = UP_AVOID;
            yawA = YAW_AVOID;
            break;
        default:
            break;
        }

        //from base station
        //compute blimp_state machine
        if (blimp_state == manual) {
            //get manual data
            //all motor commands are between -1 and 1
            //set max yaw command to 120 deg/s

            yawCom = -yaw_msg*120;

            if (USE_EST_VELOCITY_IN_MANUAL == true){
                //set max velocities 2 m/s
                upCom = up_msg*2.0;
                forwardCom = forward_msg*2.0;
                translationCom = translation_msg*2.0;
            } else {
                //normal mapping using max esc command 
                // upCom = up_msg*2.0; //PID used and maxed out at 2m/s
                upCom = -up_msg*500.0; //up is negative
                // upCom = -up_msg*500.0-0.5*pitch; //pitch correction? (pitch in degrees, conversion factor command/degree)
                forwardCom = forward_msg*500.0;
                translationCom = translation_msg*500.0;
            }

            //check if shooting should be engaged
            //this block switches the blimp_state to the oposite that it is currently in
            if (shoot != shootCom) {
                shoot = shootCom;

                //change shoot blimp_state
                if (ballGrabber.state == 2) {
                    //stop shooting
                    ballGrabber.closeGrabber(blimp_state);
                } else {
                    //reset catch counter
                    catches = 0;

                    //go back to searching
                    auto_state = searching;
                    // searching timer
                    searchingTimeStart = millis();
                    searchYawDirection = searchDirection();  //randomize the search direction

                    //start shooting
                    ballGrabber.shoot(blimp_state);
                }
            //check if grabbing should be engaged
            //this block switches the blimp_state to the oposite that it is currently in
            } else if (grab != grabCom) {
                grab = grabCom;

                //change grab blimp_state
                if (ballGrabber.state == 0) {
                    ballGrabber.openGrabber(blimp_state);
                } else {
                    ballGrabber.closeGrabber(blimp_state);

                    //increase catch counter
                    catches++;

                    //start catch timmer
                    if (catches >= 1) {
                        lastCatch = micros()/MICROS_TO_SEC;
                    }
                }
            }
        } else if (blimp_state == autonomous) {
            //filter target data
            // float tx = 0;
            // float ty = 0;
            float tx = 0;
            float ty = 0;
            float tz = 0;
            float temp_tx = 0;
            float temp_ty = 0;
            // float area = 0;

            //new target (empty target)
            std::vector<double> detected_target;

            //if a target is seen
            if (targets[2] != 1000){
                float rawZ = targets[2]; // distance
                tx = xFilter.filter(static_cast<float>(targets[0]));
                ty = yFilter.filter(static_cast<float>(targets[1]));
                tz = zFilter.filter(rawZ);
                
                // tx = static_cast<float>(targets[0]);
                // ty = static_cast<float>(targets[1]);
                // tz = rawZ;

                detected_target.push_back(tx);
                detected_target.push_back(ty);
                detected_target.push_back(tz);
                
            } else {
                // no target, set to default value
                xFilter.filter(0);
                yFilter.filter(0);
                zFilter.filter(4);
                // areaFilter.filter(0);
            }
            /*
            //update targets data if any target exists
            //TODO: add area to verify distance 
            //balloon 
            if (targets[2] != 1000 && (auto_state == searching || auto_state == approach || auto_state == catching)) {
                float rawZ = targets[2]; //balloon distance
                // update filtered target coordinates (3D space, with center of camera as (0,0,0))
                // tx = xFilter.filter(targets[0]); (3D)
                // ty = yFilter.filter(targets[1]);
                tx = xFilter.filter(static_cast<float>(pixels[0]));
                ty = yFilter.filter(static_cast<float>(pixels[1]));
                tz = zFilter.filter(rawZ);
                // area = areaFilter.filter(target[0][3]);
                detected_target.push_back(tx);
                detected_target.push_back(ty);
                detected_target.push_back(tz);
            } else {
                //no target, set to default value
                xFilter.filter(0);
                yFilter.filter(0);
                zFilter.filter(0);
                // areaFilter.filter(0);
            }

            //orange goal
            //in goal scoring stages 
            if (targets[5] != 1000 && goalColor == orange && (auto_state == goalSearch || auto_state == approachGoal || auto_state == scoringStart)) {
                float rawZ = targets[5]; //balloon distance
                //update filtered target coordinates (3D space, with center of camera as (0,0,0))
                // tx = xFilter.filter(targets[3]);
                // ty = yFilter.filter(targets[4]); 
                tx = xFilter.filter(static_cast<float>(pixels[3]));
                ty = yFilter.filter(static_cast<float>(pixels[4])); 
                tz = zFilter.filter(rawZ);
                // area = areaFilter.filter(target[0][3]);
                detected_target.push_back(tx);
                detected_target.push_back(ty);
                detected_target.push_back(tz);
            } else {
                //no target, set to default value
                xFilter.filter(0);
                yFilter.filter(0);
                zFilter.filter(0);
                // areaFilter.filter(0);
            }

            //yellow goal
            if (targets[8] != 1000 && goalColor == yellow && (auto_state == goalSearch || auto_state == approachGoal || auto_state == scoringStart)) {
                float rawZ = targets[8]; //balloon distance
                //update filtered target coordinates (3D space, with center of camera as (0,0,0))
                // tx = xFilter.filter(targets[6]);
                // ty = yFilter.filter(targets[7]);
                tx = xFilter.filter(static_cast<float>(pixels[6]));
                ty = yFilter.filter(static_cast<float>(pixels[7]));
                tz = zFilter.filter(rawZ);
                // area = areaFilter.filter(target[0][3]);
                detected_target.push_back(tx);
                detected_target.push_back(ty);
                detected_target.push_back(tz);
            } else {
                //no target, set to default value
                xFilter.filter(0);
                yFilter.filter(0);
                zFilter.filter(0);
                // areaFilter.filter(0);
            }
            */


            //test target message

            // if (target.size() != 0){
            // debug_msg.data.data[0] = target[0];
            // debug_msg.data.data[1] = target[1];
            // debug_msg.data.data[2] = target[2];
            // debug_msg.data.size = 3;
            // }

            // RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));

            //modes for autonomous behavior
            switch (auto_state) {
                //blimp_state machine
                case searching: {

                    //check if goal scoring should be attempted
                    if (catches >= 1 && ((micros()/MICROS_TO_SEC - lastCatch) >= (MAX_SEARCH_WAIT_AFTER_ONE - (catches-1)*GAME_BALL_WAIT_TIME_PENALTY))) {
                        catches = TOTAL_ATTEMPTS;
                        auto_state = goalSearch;
                        goalYawDirection = searchDirection();  //randomize search direction
                        break;
                    }

                    if (catches >= TOTAL_ATTEMPTS) {
                        auto_state = goalSearch;
                        goalYawDirection = searchDirection();  //randomize search direction
                        break;
                    }

                    //begin search pattern spinning around at different heights
                    if (detected_target.size() == 0) {

                        //keep ball grabber closed
                        // ballGrabber.closeGrabber();

                        //use object avoidence
                        double avoidanceMinVal = 1000.0; // Initialize 
                        int avoidanceMinIndex = 10;

                        // Iterate through the vector to find the minimum value and its index
                        // find the minimum distance and its corresponding quadrant number (1-9)
                        for (int i = 0; i < 9; ++i) {
                            if (avoidance[i] < avoidanceMinVal) {
                                avoidanceMinVal = avoidance[i]; //distance
                                avoidanceMinIndex = i+1; //quadrant number
                            }
                        }

                        //set the avoidance quadrant only when avoidance range is triggered
                        if (avoidanceMinVal < AVOID_TRIGGER){
                            //update quadrant
                            quadrant = avoidanceMinIndex;
                        } else {
                            //safe
                            //update quadrant
                            quadrant = 10;
                        }

                        //avoding obstacle
                        if (quadrant != 10 && USE_OBJECT_AVOIDENCE) {

                            //overide search commands
                            yawCom = yawA;
                            forwardCom = forwardA;
                            upCom = upA;

                        } else {
                            //search behavior (no detected_target)
                            //spin in a small circle looking for a game ball
                            //randomize the diretion selection
                            // yawCom = GAME_BALL_YAW_SEARCH*searchYawDirection;
                            //upCom = 0;    //is overriden later, defined here as a safety net
                            // forwardCom = GAME_BALL_FORWARD_SEARCH;
                            
                            // Timeline:
                            // SearchingStart -> +18 seconds -> +20 seconds -> restart searching
                            double elapsedSearchingTime = millis() - searchingTimeStart;
                            std::string message = "elapsedSearchTime=" + std::to_string(elapsedSearchingTime) + "s.";
                            if(elapsedSearchingTime < searchTime){
                                backingUp = false;
                                yawCom = GAME_BALL_YAW_SEARCH*searchYawDirection;
                                forwardCom = GAME_BALL_FORWARD_SEARCH;

                                if (actualBaro > CEIL_HEIGHT) {
                                    upCom = GAME_BALL_VERTICAL_SEARCH; //down
                                }else if (actualBaro < FLOOR_HEIGHT) {
                                    upCom = -GAME_BALL_VERTICAL_SEARCH;  //up
                                }else{
                                    upCom = 0;
                                }
                            }else if(elapsedSearchingTime < searchTime + backupTime){
                                if(!backingUp){
                                    backingUp = true;
                                    searchYawDirection = searchDirection();
                                }   
                                message += " Backup!";
                                
                                yawCom = 35*searchYawDirection;
                                forwardCom = -240;
                                upCom = -100;
                            }else{
                                message += " Reset!";
                                searchingTimeStart = millis();
                            }
                            publish_log(message.c_str());

                            // Prolly hit a net
                            // if (20000 < millis()- searchingTimeStart){
                            //     double backupTimer = millis();

                            //     std::string message = "20+ seconds.";

                            //     if (2200 > millis() - backupTimer) {
                            //         message += " first 2 seconds.";
                            //         searchYawDirection = searchDirection();
                            //         yawCom = 60*searchYawDirection;
                            //         upCom = -50;    //is overriden later, defined here as a safety net
                            //         forwardCom = -200;
                            //     }

                            //     searchingTimeStart = millis(); 
                            //     publish_log(message.c_str());
                            // }

                            //move up and down within the set boundry
                            // if (actualBaro > CEIL_HEIGHT) {
                            //     // if (wasUp) wasUp = false;
                            //     yawCom = GAME_BALL_YAW_SEARCH*searchYawDirection;
                            //     upCom = GAME_BALL_VERTICAL_SEARCH; //down
                            //     forwardCom = GAME_BALL_FORWARD_SEARCH;
                            // }

                            // if (actualBaro < FLOOR_HEIGHT) {
                            //     // if (!wasUp) wasUp = true;
                            //     yawCom = GAME_BALL_YAW_SEARCH*searchYawDirection;
                            //     upCom = -GAME_BALL_VERTICAL_SEARCH;  //up
                            //     forwardCom = GAME_BALL_FORWARD_SEARCH;
                            // }

                            // if (actualBaro <= CEIL_HEIGHT && actualBaro >=FLOOR_HEIGHT) {
                            //     // if (wasUp) wasUp = false;
                            //     yawCom = GAME_BALL_YAW_SEARCH*searchYawDirection;
                            //     upCom = 0;
                            //     forwardCom = GAME_BALL_FORWARD_SEARCH;
                            // }
                        }

                    } else {
                        //move to approaching game ball
                        auto_state = approach;
                        //start approaching timer
                        approachTimeStart = millis();
                    }
                    break;
                }case approach: {
                    // max time to approach
                    if (approachTimeMax < millis() - approachTimeStart) {
                        auto_state = searching;
                        // searching timer
                        searchingTimeStart = millis();
                    }

                    //check if target is still valid
                    if (detected_target.size() > 0) {
                        //seeing a target
                        //add memory
                        temp_tx = tx;
                        temp_ty = ty;
                        catchMemoryTimer = millis();

                        // if (catches >= 1 && micros()/MICROS_TO_SEC - lastCatch >= MAX_SEARCH_WAIT_AFTER_ONE - (catches-1)*(GAME_BALL_WAIT_TIME_PENALTY)) {
                        //   catches = TOTAL_ATTEMPTS;
                        //   mode = goalSearch;
                        //   goalYawDirection = searchDirection();  //randomize search direction
                        //   break;
                        // }

                        // if (catches >= TOTAL_ATTEMPTS) {
                        //   mode = goalSearch;
                        //   goalYawDirection = searchDirection();  //randomize search direction
                        // }

                        //move toward the balloon
                        yawCom = xPID.calculate(GAME_BaLL_X_OFFSET, tx, dt/1000); 
                        upCom = -yPID.calculate(GAME_BALL_APPROACH_ANGLE, ty, dt/1000);  
                        forwardCom = GAME_BALL_CLOSURE_COM;
                        translationCom = 0;

                        //check if the gate should be opened
                        if (tz < BALL_GATE_OPEN_TRIGGER) {
                            ballGrabber.openGrabber(blimp_state);

                            //check if the catching mode should be triggered
                            if (tz < BALL_CATCH_TRIGGER) {
                                auto_state = catching;

                                //start catching timer
                                catchTimeStart = millis();

                                //turn motors off
                                motorControl.update(0,0,0,0,0);
                            }
                        } else {
                            //make sure grabber is closed, no game ball is close enough to catch
                            ballGrabber.closeGrabber(blimp_state);
                        }

                        //if target is lost within 1 second
                        //remember the previous info about where the ball is 
                    }
                    else if((millis()-catchMemoryTimer) < 1000 && detected_target.size() == 0){
                            yawCom = xPID.calculate(GAME_BaLL_X_OFFSET, temp_tx, dt/1000); 
                            upCom = -yPID.calculate(GAME_BALL_APPROACH_ANGLE, temp_ty, dt/1000);  
                            forwardCom = GAME_BALL_CLOSURE_COM;
                            translationCom = 0;
                    } 
                    // after two seconds of losing the target, the target is still not detected
                    else {
                        //no target, look for another
                        //maybe add some memory
                        auto_state = searching;
                        // searching timer
                        searchingTimeStart = millis();
                        ballGrabber.closeGrabber(blimp_state);
                        searchYawDirection = searchDirection();  //randomize the search direction
                    }

                    break;
                }case catching: {
                    //wait for 0.3 second
                    // delay(300);

                    forwardCom = CATCHING_FORWARD_COM;
                    upCom = -CATCHING_UP_COM;
                    yawCom = 0;
                    translationCom = 0;

                    if (catchTimeStart < millis() - catchTime) {
                        //catching ended, start caught timer
                        auto_state = caught;
                        caughtTimeStart = millis();
                        ballGrabber.closeGrabber(blimp_state);

                        //increment number of catches
                        catches = catches + 1;

                        //start catch timmer
                        lastCatch = micros()/MICROS_TO_SEC;
                    }
                    break;
                } case caught: {
                    if (catches > 0) {
                        //if a target is seen right after the catch
                        if (detected_target.size() > 0) {
                            //approach next game ball if visible
                            if (catches < TOTAL_ATTEMPTS) {
                                auto_state = searching;
                                // searching timer
                                searchingTimeStart = millis();
                                searchYawDirection = searchDirection();  //randomize the search direction
                            }
                        }

                        //decide if the blimp is going to game ball search or goal search
                        if (caughtTimeStart < millis() - caughtTime) {
                            if (catches >= TOTAL_ATTEMPTS) {
                                auto_state = goalSearch;
                                goalYawDirection = searchDirection();  //randomize search direction
                            } else {
                                auto_state = searching;
                                // searching timer
                                searchingTimeStart = millis();
                                searchYawDirection = searchDirection();  //randomize the search direction
                            }
                        }

                        forwardCom = CAUGHT_FORWARD_COM;
                        upCom = -CAUGHT_UP_COM;
                        yawCom = 0;
                    } else {
                        auto_state = searching;
                        // searching timer
                        searchingTimeStart = millis();
                        searchYawDirection = searchDirection();  //randomize the search direction
                    }
                    break;
                } case goalSearch: {
                    if (catches >= TOTAL_ATTEMPTS) {

                        //keep ball grabber closed
                        // ballGrabber.closeGrabber();

                        //use object avoidence
                        double avoidanceMinVal = 1000.0; // Initialize 
                        int avoidanceMinIndex = 10;

                        // Iterate through the vector to find the minimum value and its index
                        // find the minimum distance and its corresponding quadrant number (1-9)
                        for (int i = 0; i < 9; ++i) {
                            if (avoidance[i] < avoidanceMinVal) {
                                avoidanceMinVal = avoidance[i]; //distance
                                avoidanceMinIndex = i+1; //quadrant number
                            }
                        }

                        //set the avoidance quadrant only in range
                        if (avoidanceMinVal < AVOID_TRIGGER) {
                            //update quadrant
                            quadrant = avoidanceMinIndex;
                        } else {
                            //update quadrant
                            //safe
                            quadrant = 10;
                        }

                        if (quadrant != 10 && USE_OBJECT_AVOIDENCE) {
                            //avoding obstacle
                            //overide search commands
                            yawCom = yawA;
                            forwardCom = forwardA;
                            upCom = upA;
                        } else {
                            //goal search behavior
                            //randomize the diretion selection
                            yawCom = GOAL_YAW_SEARCH*goalYawDirection;
                            upCom = -goalPositionHold.calculate(GOAL_HEIGHT, actualBaro);  //go up to the goal
                            // upCom = GOAL_UP_VELOCITY;
                            forwardCom = GOAL_FORWARD_SEARCH;
                        }

                        if (detected_target.size() > 0) {
                            auto_state = approachGoal;
                        }
                        
                    } else {
                        auto_state = searching;
                        // searching timer
                        searchingTimeStart = millis();
                        searchYawDirection = searchDirection();  //randomize the search direction
                        ballGrabber.closeGrabber(blimp_state);
                    }
                    break;
                } case approachGoal: {
                    if (detected_target.size() > 0 && catches >= TOTAL_ATTEMPTS) {
                        yawCom = xPID.calculate(GOAL_X_OFFSET, tx, dt);
                        upCom = -yPID.calculate(GOAL_APPROACH_ANGLE, ty, dt);
                        forwardCom = GOAL_CLOSURE_COM;

                        if ((tz < GOAL_DISTANCE_TRIGGER && goalColor == orange) || (tz < GOAL_DISTANCE_TRIGGER && goalColor == yellow)) {
                            scoreTimeStart = millis();
                            auto_state = scoringStart;
                        }
                    } else {
                        auto_state = goalSearch;
                        goalYawDirection = searchDirection();  //randomize search direction
                        ballGrabber.closeGrabber(blimp_state);
                    }
                    break;
                } case scoringStart: {
                    //after correction, we can do goal alignment with a yaw and a translation 
                    if (true) {
                        yawCom = SCORING_YAW_COM;
                        forwardCom = SCORING_FORWARD_COM;
                        upCom = -SCORING_UP_COM;

                        if (scoreTimeStart < millis() - scoreTime) {
                            auto_state = shooting;     
                            shootingTimeStart = millis();
                            break;
                        }
                    }
                    break;
                } case shooting: {
                    if (true) {
                        yawCom = 0;
                        forwardCom = SHOOTING_FORWARD_COM;
                        upCom = -SHOOTING_UP_COM;

                        ballGrabber.shoot(blimp_state);
                        catches = 0;

                        if (shootingTimeStart < millis() - shootingTime) {
                            ballGrabber.closeGrabber(blimp_state);
                            scoredTimeStart = millis();
                            auto_state = scored;
                            break;
                        }
                    }
                    break;
                } case scored: {
                    if (true) {

                        // ballGrabber.closeGrabber(blimp_state);

                        yawCom = 0;
                        forwardCom = SCORED_FORWARD_COM;
                        upCom = SCORED_UP_COM;

                        if (scoredTimeStart < millis() - scoredTime) {
                            auto_state = searching;
                            // searching timer
                            searchingTimeStart = millis();
                            searchYawDirection = searchDirection();  //randomize the search direction
                            break;
                        }
                    }
                    break;
                } default: {
                    //shouldn't get here
                    yawCom = 0;
                    forwardCom = 0;
                    upCom = 0;
                    break;
                }
            } //End switch
        } else {
            //Blimp is lost
            forwardCom = 0.0;
            upCom = 0.0;
            yawCom = 0.0;
        }

        //publish blimp_state machine info to Basestation
        state_machine_msg.data = auto_state;

        state_machine_publisher->publish(state_machine_msg);
        //safty height 
        // if (actualBaro > MAX_HEIGHT) {
        //     upCom = -1;
        // }
        //translation velocity and command
        // Serial.print(">z v:");
        // Serial.println(-yekf.v);
        // Serial.print(">z com:");
        // Serial.println(translationCom);

        //PID controllers
        float yawMotor = 0.0;
        float upMotor = 0.0;
        float forwardMotor = 0.0;
        float translationMotor = 0.0;

        //hyperbolic tan for yaw "filtering"
        float deadband = 5; // deadband for filteration
        yawMotor = yawPID.calculate(yawCom, yawRateFilter.last, dt);  

        if (abs(yawCom-yawRateFilter.last) < deadband) {
            
            yawMotor = 0;

        } else {

            yawMotor = tanh(yawMotor)*abs(yawMotor);

        }

        //TO DO: improve velocity control
        // upMotor = verticalPID.calculate(upCom, kf.v, dt); //up velocity from barometer
        // What's up motor? :)
        upMotor = upCom;

        if (USE_EST_VELOCITY_IN_MANUAL == true) {
            //using kalman filters for the current velosity feedback for full-blimp_state feeback PID controllers

            // forwardMotor = forwardPID.calculate(forwardCom, xekf.v, dt);  //extended filter
            // float forwardMotor = forwardPID.calculate(forwardCom, kal_vel.x_vel_est, dt);
            // translationMotor = translationPID.calculate(translationCom, yekf.v, dt); //extended filter
            // float translationMotor = translationPID.calculate(translationCom, kal_vel.y_vel_est, dt); 
        } else{
            //without PID
            forwardMotor = forwardCom;
            translationMotor = translationCom;
        }

        //motor debug
        // debug_msg.data.data[0] = yawCom;
        // debug_msg.data.data[1] = upCom;
        // debug_msg.data.data[2] = translationCom;
        // debug_msg.data.data[3] = forwardCom;
        // debug_msg.data.size = 4;

        // debug_msg.data[0] = yawCom;
        // debug_msg.data[1] = upCom;
        // debug_msg.data[2] = translationCom;
        // debug_msg.data[3] = forwardCom;

        debug_msg.data = {yawCom, upCom, translationCom, forwardCom};

        

        // debug_msg.data.data[0] = forward_msg;
        // debug_msg.data.data[1] = yaw_msg;
        // debug_msg.data.data[2] = up_msg;
        // debug_msg.data.data[3] = translation_msg;
        // debug_msg.data.size = 4;

        //test target messages
        // debug_msg.data.data[0] = targets[0];
        // debug_msg.data.data[1] = targets[1];
        // debug_msg.data.data[2] = targets[2];
        // debug_msg.data.data[3] = targets[3];
        // debug_msg.data.data[4] = targets[4];
        // debug_msg.data.data[5] = targets[5];
        // debug_msg.data.data[6] = targets[6];
        // debug_msg.data.data[7] = targets[7];
        // debug_msg.data.data[8] = targets[8];
        // debug_msg.data.size = 9;

        debug_publisher->publish(debug_msg);

        // //Serial.print(">up current:");
        // //Serial.println(kf.v);
        // //float yawCurrent =  (float)yawRateFilter.last;
        // //Serial.print(">yaw current");
        // //Serial.println(yawCurrent);
        // //Serial.print(">forward current:");
        // //Serial.println(xekf.v);
        // //Serial.print(">translation current:");
        // //Serial.println(yekf.v);

        //gimbal + motor updates
        ballGrabber.update();

        // neeed to verify this
        if (dt < 10 + firstMessageTime) {
            //filter base station data
            baroOffset.filter(baseBaro-BerryIMU.comp_press);
            rollOffset.filter(BerryIMU.gyr_rateXraw);

            //zero motors while filters converge and esc arms
            motorControl.update(0, 0, 0, 0, 0);
            bool leftReady = leftGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, motorControl.yawLeft, motorControl.upLeft, motorControl.forwardLeft);
            bool rightReady = rightGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, motorControl.yawRight, motorControl.upRight, motorControl.forwardRight);
            leftGimbal.updateGimbal(leftReady && rightReady);
            rightGimbal.updateGimbal(leftReady && rightReady);
        } else {
            if (blimp_state == manual && !MOTORS_OFF) {
                //forward, translation, up, yaw, roll
                if (!ZERO_MODE) motorControl.update(forwardMotor, -translationMotor, upMotor, yawMotor, 0);
                bool leftReady = leftGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, motorControl.upLeft, motorControl.forwardLeft);
                bool rightReady = rightGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, motorControl.upRight, motorControl.forwardRight);
                leftGimbal.updateGimbal(leftReady && rightReady);
                rightGimbal.updateGimbal(leftReady && rightReady);
            } else if (blimp_state == autonomous && !MOTORS_OFF) {
                motorControl.update(forwardMotor, -translationMotor, upMotor, yawMotor, 0);
                bool leftReady = leftGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, motorControl.yawLeft, motorControl.upLeft, motorControl.forwardLeft);
                bool rightReady = rightGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, motorControl.yawRight, motorControl.upRight, motorControl.forwardRight); 
                leftGimbal.updateGimbal(leftReady && rightReady);
                rightGimbal.updateGimbal(leftReady && rightReady);
            } else if(MOTORS_OFF){
                motorControl.update(forwardMotor, -translationMotor, upMotor, yawMotor, 0);
                bool leftReady = leftGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, motorControl.yawLeft, motorControl.upLeft, motorControl.forwardLeft);
                bool rightReady = rightGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, motorControl.yawRight, motorControl.upRight, motorControl.forwardRight); 
                leftGimbal.updateGimbal(leftReady && rightReady);
                rightGimbal.updateGimbal(leftReady && rightReady);
            } else {
                motorControl.update(0,0,0,0,0);
                bool leftReady = leftGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, 0, 0);
                bool rightReady = rightGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, 0, 0);
                leftGimbal.updateGimbal(leftReady && rightReady);
                rightGimbal.updateGimbal(leftReady && rightReady);
            }
        }
    }

    void publish_log(std::string message) const {
        auto log_msg = std_msgs::msg::String();

        log_msg.data = message;
        log_publisher->publish(log_msg);
        // RCSOFTCHECK(rcl_publish(&log_publisher, &log_msg, NULL));
    }

    void auto_subscription_callback(const std_msgs::msg::Bool & msg) const
    {
        // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
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
        // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
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
        // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
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
        // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());

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
        // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());

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
        // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());

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
        // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());

        //commands from basestation
        // forward_msg = msg.data.data[3];
        // up_msg = msg.data.data[1];
        // yaw_msg = msg.data.data[0];
        // translation_msg = msg.data.data[2];

        forward_msg = msg.data[3];
        up_msg = msg.data[1];
        yaw_msg = msg.data[0];
        translation_msg = msg.data[2];

        // char motorCommands[100];  // Size depending on the expected maximum length of your combined string
        // sprintf(motorCommands, "Teensy Motor Commands\nYaw: %.2f\nUp: %.2f\nTranslation: %.2f\nForward: %.2f\n", yaw_msg, up_msg, translation_msg, forward_msg);
        // publish_log(motorCommands);
    }

    void goal_color_subscription_callback(const std_msgs::msg::Int64 & msg) const
    {
        // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());

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
        // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());

        //3 objects with xyz (9 elements in total)
        for (size_t i = 0; i < 9; ++i) {
            // avoidance[i] = msg.data.data[i];
            avoidance[i] = msg.data[i];
        }
    }

    void targets_subscription_callback(const std_msgs::msg::Float64MultiArray & msg) const
    {
        // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());

        // object of interest with xyz (3 elements in total)
        for (size_t i = 0; i < 3; ++i) {
            // targets[i] = msg.data.data[i];
            targets[i] = msg.data[i];
        }
    }

    void pixels_subscription_callback(const std_msgs::msg::Int64MultiArray & msg) const
    {
        auto pixels_msg = msg;
        // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());

        //3 objects with xyz (9 elements in total)
        for (size_t i = 0; i < 9; ++i) {
            //pixels[i] = pixels_msg.data.data[i];
            pixels[i] = pixels_msg.data[i];
        }
    }

    // //pulse function for adding ultrasonic
    void Pulse() {
        if (digitalRead(interruptPin) == HIGH) {
            // start measuring
            pulseInTimeBegin = micros();
        }
        else {
            // stop measuring
            pulseInTimeEnd = micros();
            newPulseDurationAvailable = true;
        }
    }

    float searchDirection() {
        int ran = rand()%10; //need to check bounds
        float binary = 1.0;
        if (ran <= 4){
            binary = 1.0;
        }else if (ran > 4){
            binary = -1.0;
        }
        return binary;
    }

    rclcpp::TimerBase::SharedPtr timer_imu;
    rclcpp::TimerBase::SharedPtr timer_baro;
    rclcpp::TimerBase::SharedPtr timer_state_machine;
    rclcpp::TimerBase::SharedPtr timer_;


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