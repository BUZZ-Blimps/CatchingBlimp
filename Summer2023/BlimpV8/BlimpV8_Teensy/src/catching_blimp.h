#ifndef CATCHING_BLIMP_H
#define CATCHING_BLIMP_H

#include <Arduino.h>

//C++ includes
#include <chrono>
#include <cstdlib>
#include <memory>
#include <vector>
#include <string>

//Micro ROS/ROS2 includes
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

//Message type includes
#include <std_msgs/msg/string.h> //include the message type that needs to be published (teensy data)
#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/vector3.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/int64.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <std_msgs/msg/int64_multi_array.h>

//include service
// #include <test_msgs/srv/basic_types.h>

//Includes for main.cpp
#include "MotorControl.h"
#include "BerryIMU_v3.h"
#include "Madgwick_Filter.h"
#include "baro_acc_kf.h"
#include "accelGCorrection.h"
#include "PID.h"
#include "EMAFilter.h"
#include "Optical_Flow.h"
#include "Kalman_Filter_Tran_Vel_Est.h"
#include "BangBang.h"
#include "optical_ekf.h"
#include "gyro_ekf.h"
#include "tripleBallGrabber.h"
#include "Gimbal.h"


#define EXECUTE_EVERY_N_MS(MS, X)  do { \
static volatile int64_t init = -1; \
if (init == -1) { init = uxr_millis();} \
if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

#define RCCHECK(fn) {rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

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
#define TOTAL_ATTEMPTS            4    // attempts at catching 
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


#define GAME_BALL_CLOSURE_COM     110  //approaching at 20% throttle cap
#define GAME_BALL_APPROACH_ANGLE  40  //approach magic number (TODO: reset)
#define GAME_BaLL_X_OFFSET        40   //offset magic number (TODO: reset)

#define CATCHING_FORWARD_COM      300  //catching at 50% throttle 
#define CATCHING_UP_COM           50  //damp out pitch

#define CAUGHT_FORWARD_COM        -220  //go back so that the game ball gets to the back 
#define CAUGHT_UP_COM             -40

#define GOAL_YAW_SEARCH           20   
#define GOAL_FORWARD_SEARCH       150  //200 40% throttle
#define GOAL_UP_VELOCITY          450

#define GOAL_CLOSURE_COM          90  //forward command 25% throttle
#define GOAL_X_OFFSET             80  
#define GOAL_APPROACH_ANGLE       45  //height alignment (approach down)

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

//-----States for blimp and grabber-----
enum agentState {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} agent_state_;

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

//Function prototypes
void error_loop();
void publish_log(const char *message);
// void client_callback(const void * msgin);
// void send_request();
// void receive_response();
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
// void id_subscription_callback(const void *msgin);
void auto_subscription_callback(const void *msgin);
void baro_subscription_callback(const void *msgin);
void calibrateBarometer_subscription_callback(const void *msgin);
void kill_subscription_callback(const void *msgin);
void shoot_subscription_callback(const void *msgin);
void motor_subscription_callback(const void *msgin);
void goal_color_subscription_callback(const void *msgin);
void avoidance_subscription_callback(const void *msgin);
void targets_subscription_callback(const void *msgin);
void pixels_subscription_callback(const void *msgin);
bool create_entities();
void destroy_entities();
void Pulse();
void update_agent_state();


//Global variables
//sensor fusion objects
BerryIMU_v3 BerryIMU;
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

//global variables
int blimp_state = lost;
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
const byte interruptPin = 22;
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
//executors
rclc_executor_t executor_pub;
rclc_executor_t executor_sub;
// rclc_executor_t executor_srv;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

//ROS client
// rcl_client_t client;
// rmw_request_id_t req_id;

//ROS publishers
rcl_publisher_t identity_publisher; //string
rcl_publisher_t imu_publisher;    //imu
rcl_publisher_t debug_publisher;  //float64_multi_array
rcl_publisher_t height_publisher; //float64
rcl_publisher_t z_velocity_publisher; //float64
rcl_publisher_t state_machine_publisher; //int64

//Teensy log message publisher
rcl_publisher_t log_publisher;

//ROS subscribers
rcl_subscription_t auto_subscription; //boolean
rcl_subscription_t baseBarometer_subscription; //float64
rcl_subscription_t calibrateBarometer_subscription; //boolean
rcl_subscription_t grabber_subscription; //boolean
rcl_subscription_t shooter_subscription; //boolean
rcl_subscription_t motor_subscription; //float64_multi_array
rcl_subscription_t kill_subscription; //boolean
rcl_subscription_t goal_color_subscription; //int64
rcl_subscription_t avoidance_subscription; //float64_multi_array
rcl_subscription_t targets_subscription; //float64_multi_array
rcl_subscription_t pixels_subscription; //int64_multi_array

//The following names can be commented/uncommented based on the blimp that is used
// Define the name of the blimp/robot
//  std::string blimpNameSpace = "BurnCreamBlimp";
//std::string blimpNameSpace = "SillyAhBlimp";
//std::string blimpNameSpace = "TurboBlimp";
//std::string blimpNameSpace = "GameChamberBlimp";
std::string blimpNameSpace = "GravyLongWayBlimp";
// std::string blimpNameSpace = "SuperBeefBlimp";

//message types: String Bool Float32 Float32 MultiArray
//message topics : /auto /baseBarometer /blimpID /grabbing /killed /motorCommands /shooting /identify /imu /goal_color /state_machine

//service response&request
// test_msgs__srv__BasicTypes_Response res;
// test_msgs__srv__BasicTypes_Request req;
bool check = false;

//boolean messages
std_msgs__msg__Bool auto_msg;
std_msgs__msg__Bool grab_msg;
std_msgs__msg__Bool shoot_msg;
std_msgs__msg__Bool kill_msg;
std_msgs__msg__Bool calibration_msg;

//int64 message
std_msgs__msg__Int64 goal_color_msg;
std_msgs__msg__Int64 state_machine_msg;

//float64 message
std_msgs__msg__Float64  baro_msg;
std_msgs__msg__Float64  height_msg;
std_msgs__msg__Float64  z_velocity_msg;

//float64multiarray and int64multiarray messages
std_msgs__msg__Float64MultiArray motor_msg;
std_msgs__msg__Float64MultiArray debug_msg;
std_msgs__msg__Float64MultiArray avoidance_msg;
std_msgs__msg__Float64MultiArray targets_msg;
std_msgs__msg__Int64MultiArray pixels_msg;

//String messages
std_msgs__msg__String identity_msg;
std_msgs__msg__String log_msg;

//sensor message
sensor_msgs__msg__Imu imu_msg;

int counter = 0;
bool last_connected = false;
bool last_lost = true;

#endif
