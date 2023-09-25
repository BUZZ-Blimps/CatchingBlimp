#include <Arduino.h>

#include <micro_ros_platformio.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <vector>
#include <string>

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

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

//include all message types needed
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
#include <test_msgs/srv/basic_types.h>

#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

#define RCCHECK(fn) {rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(1000);
  }
}

//blimp mode
#define BLIMP_COLOR               red      //either red or blue
#define GOAL_COLOR                orange    //either orange or yellow

#define CEIL_HEIGHT_FROM_START    4

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
#define TOTAL_ATTEMPTS            5    // attempts at catching 
#define MAX_ATTEMPTS              5    //should be set to 5

//flight area parameters
#define CEIL_HEIGHT               8      //m
#define FLOOR_HEIGHT              2.5    //m

#define MAX_HEIGHT                2    //m
#define GOAL_HEIGHT               5.0    //m
#define GOAL_HEIGHT_DEADBAND      0.3       //m

//distance triggers
#define GOAL_DISTANCE_TRIGGER    1.3 //m distance for blimp to trigger goal score 	
#define BALL_GATE_OPEN_TRIGGER   2 //m distance for blimp to open the gate 	
#define BALL_CATCH_TRIGGER       1.2  //m distance for blimp to start the open-loop control

//object avoidence motor coms
#define FORWARD_AVOID             125  //25% throttle
#define YAW_AVOID                 10	 //deg/s
#define UP_AVOID                  0.4  //m/s

//autonomy tunning parameters
// the inputs are bounded from -2 to 2, yaw is maxed out at 120 deg/s
#define GAME_BALL_YAW_SEARCH      -15  //deg/s
#define GAME_BALL_FORWARD_SEARCH  130 //30% throttle 
#define GAME_BALL_VERTICAL_SEARCH 10  //m/s


#define GAME_BALL_CLOSURE_COM     130  //approaching at 20% throttle cap
#define GAME_BALL_APPROACH_ANGLE  40  //approach magic number (TODO: reset)
#define GAME_BaLL_X_OFFSET        0   //offset magic number (TODO: reset)

#define CATCHING_FORWARD_COM      280  //catching at 50% throttle 
#define CATCHING_UP_COM           10  //damp out pitch

#define CAUGHT_FORWARD_COM        -220  //go back so that the game ball gets to the back 
#define CAUGHT_UP_COM             -40

#define GOAL_YAW_SEARCH           20   
#define GOAL_FORWARD_SEARCH       150  //200 40% throttle
#define GOAL_UP_VELOCITY          30

#define GOAL_CLOSURE_COM          90  //forward command 25% throttle
#define GOAL_X_OFFSET             80  
#define GOAL_APPROACH_ANGLE       45  //height alignment (approach down)

//goal alignment test
#define ALIGNING_YAW_COM           10 //test
#define ALIGNING_FORWARD_COM        100 //test
#define ALIGNING_UP_COM            100 //test
#define ALIGNING_TRANSLATION_COM   300 //test


#define SCORING_YAW_COM           0
#define SCORING_FORWARD_COM       400 //40% throttle
#define SCORING_UP_COM            180

#define SHOOTING_FORWARD_COM      400  //counter back motion 
#define SHOOTING_UP_COM           150
//counter moment (right now we do want to shoot up because ball sinks)

#define SCORED_FORWARD_COM        -250
#define SCORED_UP_COM             -50

//sensor and controller rates
#define FAST_SENSOR_LOOP_FREQ           100.0
#define BARO_LOOP_FREQ                  50.0
#define STATE_MACHINE_FREQ              20.0
#define OPTICAL_LOOP_FREQ               55.0

#define DIST_CONSTANT             0.002

#define GYRO_X_CONSTANT           480.0
#define GYRO_YAW_CONSTANT         0

#define GYRO_Y_CONSTANT           323.45

//constants
#define MICROS_TO_SEC             1000000.0
#define BUFFER_LEN                500

//motor timeout before entering lost state
#define TEENSY_WAIT_TIME          0.5

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
Gimbal leftGimbal(L_Yaw, L_Pitch, PWM_L, 25, 30, 1000, 2000, 45, 0.5);
Gimbal rightGimbal(R_Yaw, R_Pitch, PWM_R, 25, 30, 1000, 2000, 135, 0.5);

//Manual PID control
PID verticalPID(350, 0, 0);  //not used for now due to baro reading malfunction
PID yawPID(12.0, 0, 0);  //can also tune kd with a little overshoot induced
PID forwardPID(300, 0, 0);  //not used
PID translationPID(300, 0, 0); //not used

//Auto PID control (output fed into manual controller)
PID yPID(0.5,0,0);    //TODO:retune these (can also be in pixels depends on which one performs better) 0.0075 for pixel PID
PID xPID(0.036,0,0);       //TODO:retune these 0.162 for pixel PID

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
TripleBallGrabber ballGrabber(GATE_S, PWM_G);

//-----States for blimp and grabber-----
enum agentState {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
}agent_state_;

enum autoState {
  searching,
  approach,
  catching,
  caught,
  goalSearch,
  approachGoal,  //To Do: add goal aligntment (PID) in if, or add another state "align"
  scoringStart,
  shooting,
  scored,
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

//timing global variables for each update loop
float lastSensorFastLoopTick = 0.0;
float lastStateLoopTick = 0.0;
float lastBaroLoopTick = 0.0;
float lastOpticalLoopTick = 0.0;

//global variables
int state = lost;
int mode = searching;

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
double approachTimeStart = 0;
double approachTimeMax = 10000;   //ms

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
int quad = 10;

//base station baro
float baseBaro = 0.0;

//direction from last ball search
bool wasUp = true;

//corrected baro
float actualBaro = 0.0;

float searchYawDirection = -1;

float goalYawDirection = -1;

//targets data and pixel data (balloon, orange goal, yellow goal)
//1000 means object is not present
std::vector<double> targets = {1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0};
std::vector<int64_t> pixels = {1000, 1000, 0, 1000, 1000, 0, 1000, 1000, 0};
//------------------MICRO ROS publishers/subscribers--------------
//ROS node
//executors
rclc_executor_t executor_pub;
rclc_executor_t executor_sub;
rclc_executor_t executor_srv;


rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;


//ROS client
rcl_client_t client;
rmw_request_id_t req_id;

//ROS publishers
rcl_publisher_t blimpid_publisher; //string
rcl_publisher_t imu_publisher;    //imu
rcl_publisher_t debug_publisher;  //float64_multi_array
rcl_publisher_t height_publisher; //float64
rcl_publisher_t z_velocity_publisher; //float64
rcl_publisher_t state_machine_publisher; //int64

//ROS subscribers
rcl_subscription_t identity_subscription; //boolean
rcl_subscription_t auto_subscription; //boolean
rcl_subscription_t baseBarometer_subscription; //float64
rcl_subscription_t grabber_subscription; //boolean
rcl_subscription_t shooter_subscription; //boolean
rcl_subscription_t motor_subscription; //float64_multi_array
rcl_subscription_t kill_subscription; //boolean
rcl_subscription_t goal_color_subscription; //int64
rcl_subscription_t targets_subscription; //float64_multi_array
rcl_subscription_t pixels_subscription; //int64_multi_array

//The following names can be commented/uncommented based on the blimp that is used
// Define the name of the blimp/robot
std::string blimpNameSpace = "BurnCreamBlimp";   
// std::string blimpNameSpace = "SillyAhBlimp";
// std::string blimpNameSpace = "TurboBlimp";
// std::string blimpNameSpace = "GameChamberBlimp";
// std::string blimpNameSpace = "FiveGuysBlimp";


//message types: String Bool Float32 Float32 MultiArray
//message topics : /auto /baseBarometer /blimpID /grabbing /killed /motorCommands /shooting /identify /imu /goal_color /state_machine

//service response&request
test_msgs__srv__BasicTypes_Response res;
test_msgs__srv__BasicTypes_Request req;
bool check = false;

//bolean message
std_msgs__msg__Bool identity_msg;
std_msgs__msg__Bool auto_msg;
std_msgs__msg__Bool grab_msg;
std_msgs__msg__Bool shoot_msg;
std_msgs__msg__Bool kill_msg;

//int64 message
std_msgs__msg__Int64 goal_color_msg;
std_msgs__msg__Int64 state_machine_msg;

//float64 message
std_msgs__msg__Float64  baro_msg;
std_msgs__msg__Float64  height_msg;
std_msgs__msg__Float64  z_velocity_msg;

//float64multiarray message
std_msgs__msg__Float64MultiArray motor_msg;
std_msgs__msg__Float64MultiArray debug_msg;
std_msgs__msg__Float64MultiArray targets_msg;
std_msgs__msg__Int64MultiArray pixels_msg;

//String message
std_msgs__msg__String blimpid_msg;
const char* blimp_name_str = blimpNameSpace.c_str(); //const char message
int counter = 0;

//sensor message
sensor_msgs__msg__Imu imu_msg;


//client functions
//TODO: fix these to make them work
void client_callback(const void * msgin){
  // Cast response message to expected type
  test_msgs__srv__BasicTypes_Response *id_msg = (test_msgs__srv__BasicTypes_Response *) msgin;
  check = id_msg->bool_value;
}

//send request
void send_request(){
  // request
  int64_t seq;
  test_msgs__srv__BasicTypes_Request__init(&req);
  snprintf(req.string_value.data, BUFFER_LEN,"%s",blimp_name_str);
  req.string_value.size = strlen(req.string_value.data);
  req.string_value.capacity = BUFFER_LEN;
  delay(2000);  //Sleep a while to ensure DDS matching before sending request
  RCSOFTCHECK(rcl_send_request(&client, &req, &seq));
}

//receive response
void receive_response(){
  //response
  delay(2000);
  RCSOFTCHECK(rcl_take_response(&client,&req_id, &res));
}

//timer call back
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
  // snprintf(msg.data.data, BUFFER_LEN, "%s for the %dth time!", blimp_name_str, counter++);
  snprintf(blimpid_msg.data.data, BUFFER_LEN,"%s", blimp_name_str);
  blimpid_msg.data.size = strlen(blimpid_msg.data.data);
  blimpid_msg.data.capacity = BUFFER_LEN;
  RCSOFTCHECK(rcl_publish(&blimpid_publisher, &blimpid_msg, NULL));
  }
}


//subscription massage callbacks
//message topics : /auto /baseBarometer/grabbing /killed /motorCommands /shooting /

void id_subscription_callback(const void *msgin)
{
  const std_msgs__msg__Bool *identity_msg = (const std_msgs__msg__Bool *)msgin;
  //Do nothing
}

void auto_subscription_callback(const void *msgin)
{

  const std_msgs__msg__Bool *auto_msg = (const std_msgs__msg__Bool *)msgin;
  if (auto_msg->data == false){
    state = manual;
    //update last message time
    lastMsgTime = micros()/MICROS_TO_SEC;
  } else if (auto_msg->data == true){
    state = autonomous;
    //update last message time
    lastMsgTime = micros()/MICROS_TO_SEC;
  } else {
    state = lost;
  }

}

void baro_subscription_callback(const void *msgin)
{
  const std_msgs__msg__Float64 *baro_msg = (const std_msgs__msg__Float64 *)msgin;
  baseBaro = baro_msg->data;
}

void grab_subscription_callback(const void *msgin)
{
  const std_msgs__msg__Bool *grab_msg = (const std_msgs__msg__Bool *)msgin;
  if (grab_msg->data == false){
    grabCom = 0;
  } else if (grab_msg->data == true){
    grabCom = 1;
  }
}

void kill_subscription_callback(const void *msgin)
{
  const std_msgs__msg__Bool *kill_msg = (const std_msgs__msg__Bool *)msgin;

 if (kill_msg->data == true){
        motorControl.update(0,0,0,0,0);
        bool leftReady = leftGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, 0, 0);
        bool rightReady = rightGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, 0, 0);
        leftGimbal.updateGimbal(leftReady && rightReady);
        rightGimbal.updateGimbal(leftReady && rightReady);
  } 
}

void shoot_subscription_callback(const void *msgin)
{
  const std_msgs__msg__Bool *shoot_msg = (const std_msgs__msg__Bool *)msgin;

   if (shoot_msg->data == false){
    shootCom = 0;
  } else if (shoot_msg->data == true){
    shootCom = 1;
  }
}

void motor_subscription_callback(const void *msgin)
{
  const std_msgs__msg__Float64MultiArray *motor_msg = (const std_msgs__msg__Float64MultiArray *)msgin;
  //commands from basestation
  forward_msg = motor_msg->data.data[3];
  up_msg = motor_msg->data.data[1];
  yaw_msg = motor_msg->data.data[0];
  translation_msg = motor_msg->data.data[2];
}

void goal_color_subscription_callback(const void *msgin)
{
  const std_msgs__msg__Int64 *goal_color_msg = (const std_msgs__msg__Int64 *)msgin;
  int goal_color = goal_color_msg->data;
  if (goal_color == 0){
    goalColor = orange;
  } else if (goal_color == 1){
    goalColor = yellow;
  }
}

void targets_subscription_callback(const void *msgin)
{
  const std_msgs__msg__Float64MultiArray *targets_msg = (const std_msgs__msg__Float64MultiArray *)msgin;
  //3 objects with xyz (9 elements in total)
    for (size_t i = 0; i < 9; ++i) {
      targets[i] = targets_msg->data.data[i];
    }
}

void pixels_subscription_callback(const void *msgin)
{
  const std_msgs__msg__Int64MultiArray *pixels_msg = (const std_msgs__msg__Int64MultiArray *)msgin;
  //3 objects with xyz (9 elements in total)
    for (size_t i = 0; i < 9; ++i) {
      pixels[i] = pixels_msg->data.data[i];
    }
}


bool create_entities() {
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, blimp_name_str, "", &support)); //name the robot

  //create client
  RCCHECK(rclc_client_init_default(&client, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(test_msgs, srv, BasicTypes), "/BlimpID"));  


  // create publishers (6 right now)
  RCCHECK(rclc_publisher_init_default(&blimpid_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), (blimpNameSpace + "/blimpID").c_str()));
  RCCHECK(rclc_publisher_init_default(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), (blimpNameSpace + "/imu").c_str()));
  RCCHECK(rclc_publisher_init_default(&debug_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg,Float64MultiArray ), (blimpNameSpace + "/debug").c_str()));
  RCCHECK(rclc_publisher_init_default(&height_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg,Float64), (blimpNameSpace + "/height").c_str()));
  RCCHECK(rclc_publisher_init_default(&z_velocity_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg,Float64), (blimpNameSpace + "/z_velocity").c_str()));
  RCCHECK(rclc_publisher_init_default(&state_machine_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg,Int64), (blimpNameSpace + "/state_machine").c_str()));

  //create subscribers (9 right now)
  RCCHECK(rclc_subscription_init_default(&identity_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/identify"));
  RCCHECK(rclc_subscription_init_default(&auto_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), (blimpNameSpace + "/auto").c_str()));
  RCCHECK(rclc_subscription_init_default(&baseBarometer_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), (blimpNameSpace + "/baseBarometer").c_str()));
  RCCHECK(rclc_subscription_init_default(&grabber_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), (blimpNameSpace + "/grabbing").c_str()));
  RCCHECK(rclc_subscription_init_default(&shooter_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), (blimpNameSpace + "/shooting").c_str()));
  RCCHECK(rclc_subscription_init_default(&motor_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray), (blimpNameSpace + "/motorCommands").c_str()));
  RCCHECK(rclc_subscription_init_default(&kill_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), (blimpNameSpace + "/killed").c_str()));
  RCCHECK(rclc_subscription_init_default(&goal_color_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64), (blimpNameSpace + "/goal_color").c_str()));
  RCCHECK(rclc_subscription_init_default(&targets_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray), (blimpNameSpace + "/targets").c_str()));
  RCCHECK(rclc_subscription_init_default(&pixels_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64MultiArray), (blimpNameSpace + "/pixels").c_str()));


  // create timer
  const unsigned int timer_period = 10;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_period), timer_callback));

  // create executor
  executor_pub = rclc_executor_get_zero_initialized_executor();
  executor_sub = rclc_executor_get_zero_initialized_executor();
  // executor_srv = rclc_executor_get_zero_initialized_executor();

  //init executors
  // RCCHECK(rclc_executor_init(&executor_srv, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 6, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 10, &allocator));

  //add client  
  // RCCHECK(rclc_executor_add_client(&executor_srv, &client, &res, client_callback));
  // send_request();
  // receive_response();
  
  //add all subscriptions (9) 
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &identity_subscription, &identity_msg, &id_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &auto_subscription, &auto_msg, &auto_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &baseBarometer_subscription, &baro_msg, &baro_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &grabber_subscription, &grab_msg, &grab_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &shooter_subscription, &shoot_msg, &shoot_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &kill_subscription, &kill_msg, &kill_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &motor_subscription, &motor_msg, &motor_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &goal_color_subscription, &goal_color_msg, &goal_color_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &targets_subscription, &targets_msg, &targets_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &pixels_subscription, &pixels_msg, &pixels_subscription_callback, ON_NEW_DATA));
  return true;
}

void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  //finish publishers
  rcl_publisher_fini(&blimpid_publisher, &node);
  rcl_publisher_fini(&imu_publisher, &node);
  rcl_publisher_fini(&debug_publisher, &node);
  rcl_publisher_fini(&height_publisher, &node);
  rcl_publisher_fini(&z_velocity_publisher, &node);
  rcl_publisher_fini(&state_machine_publisher, &node);

  //finnish subscriptions
  rcl_subscription_fini(&identity_subscription, &node);
  rcl_subscription_fini(&auto_subscription, &node);
  rcl_subscription_fini(&baseBarometer_subscription, &node);
  rcl_subscription_fini(&grabber_subscription, &node);
  rcl_subscription_fini(&shooter_subscription, &node);
  rcl_subscription_fini(&motor_subscription, &node);
  rcl_subscription_fini(&kill_subscription, &node);
  rcl_subscription_fini(&goal_color_subscription, &node);
  rcl_subscription_fini(&targets_subscription, &node);
  rcl_subscription_fini(&pixels_subscription, &node);
  
  //finish client
  rcl_client_fini(&client, &node);

  rcl_timer_fini(&timer);
  // rclc_executor_fini(&executor_srv);
  rclc_executor_fini(&executor_pub);
  rclc_executor_fini(&executor_sub);
 
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// Functions create_entities and destroy_entities can take several seconds.
// In order to reduce this rebuild the library wif
// - UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS=3


//pulse function for adding ultrasonic
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

float searchDirection(){
  int rand = random(0,10);
  float binary = 1.0;
   if (rand <= 4){
    binary = 1.0;
   }else if (rand > 4){
    binary = -1.0;
   }
   return binary;
}

//set up

void setup() {
  //start serial connection
  Serial.begin(115200);
  set_microros_serial_transports(Serial); //to pi
  agent_state_ = WAITING_AGENT; //wait for connection
  Serial1.begin(115200);
  //Start IMU
  BerryIMU.BerryIMU_v3_Setup();

  
  //initialize messages

  //service message
  req.string_value.data = (char *)malloc(BUFFER_LEN * sizeof(char));
  
  //publisher/subscriber messages
  imu_msg.header.frame_id.data = (char *) malloc(BUFFER_LEN*sizeof(char));
  imu_msg.header.frame_id.size = 0;
  imu_msg.header.frame_id.capacity = BUFFER_LEN;


  blimpid_msg.data.data = (char *) malloc(BUFFER_LEN*sizeof(char));
  blimpid_msg.data.size = strlen(blimpid_msg.data.data);
  blimpid_msg.data.capacity = BUFFER_LEN;

  motor_msg.data.data = (double *) malloc(BUFFER_LEN*sizeof(double));
  motor_msg.data.size = sizeof(motor_msg.data.data);
  motor_msg.data.capacity = BUFFER_LEN;

  debug_msg.data.data = (double *) malloc(BUFFER_LEN*sizeof(double));
  debug_msg.data.size = 0; 
  debug_msg.data.capacity = BUFFER_LEN;

  targets_msg.data.data = (double *) malloc(BUFFER_LEN*sizeof(double));
  targets_msg.data.size = sizeof(targets_msg.data.data);
  targets_msg.data.capacity = BUFFER_LEN;

  pixels_msg.data.data = (int64_t *) malloc(BUFFER_LEN*sizeof(int64_t));
  pixels_msg.data.size = sizeof(pixels_msg.data.data);
  pixels_msg.data.capacity = BUFFER_LEN;


  delay(2000);

  //time out
  firstMessageTime = micros()/MICROS_TO_SEC;
}


//loop

void loop() {
  //-------------------------MICRO ROS PUBLISHER and SUBSCRIBERS--------------------------------------
  // publisher state machine
  // checking if the agent is available 

  switch (agent_state_) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(100, agent_state_ = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      agent_state_ = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (agent_state_ == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED: //the microros agent is connected to teensy
      EXECUTE_EVERY_N_MS(10, agent_state_ = (RMW_RET_OK == rmw_uros_ping_agent(10, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (agent_state_ == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor_srv, RCL_MS_TO_NS(10));
        rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(10));
        rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(10));
      }
      break;
    case AGENT_DISCONNECTED:
        destroy_entities();
      agent_state_ = WAITING_AGENT;
      break;
    default:
      break;
  }

  //----------------------------------LOST---------------------------------------
  //lost state
  if (micros()/MICROS_TO_SEC - lastMsgTime > TEENSY_WAIT_TIME) {
      state = lost;
  }

  //----------------------------IMU LOOP-----------------------------------------
  //compute accel, gyro, madwick loop time at the set freqency
  float dt = micros()/MICROS_TO_SEC-lastSensorFastLoopTick;
  if (dt >= 1.0/FAST_SENSOR_LOOP_FREQ) {
    lastSensorFastLoopTick = micros()/MICROS_TO_SEC;

    //read sensor values and update madgwick
    BerryIMU.IMU_read();
    BerryIMU.IMU_ROTATION(rotation); // Rotate IMU
    madgwick.Madgwick_Update(BerryIMU.gyr_rateXraw,
                             BerryIMU.gyr_rateYraw,
                             BerryIMU.gyr_rateZraw,
                             BerryIMU.AccXraw,
                             BerryIMU.AccYraw,
                             BerryIMU.AccZraw);

    

    //publish imu data
    snprintf(imu_msg.header.frame_id.data, BUFFER_LEN, blimp_name_str);
    imu_msg.header.frame_id.size = strlen(blimpid_msg.data.data);
    imu_msg.header.frame_id.capacity = BUFFER_LEN;

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

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));


    //get orientation from madgwick
    pitch = madgwick.pitch_final;
    roll = madgwick.roll_final;
    yaw = madgwick.yaw_final;

    //compute the acceleration in the barometers vertical reference frame
    accelGCorrection.updateData(BerryIMU.AccXraw, BerryIMU.AccYraw, BerryIMU.AccZraw, pitch, roll);

    //run the prediction step of the vertical velecity kalman filter
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

  //-----------------------------BARO LOOP-----------------------------------------------
  //update barometere at set barometere frequency
  dt = micros()/MICROS_TO_SEC-lastBaroLoopTick;
  if (dt >= 1.0/BARO_LOOP_FREQ) {
    lastBaroLoopTick = micros()/MICROS_TO_SEC;

    //get most current imu values
    BerryIMU.IMU_read();
    
    //update kalman with uncorreced barometer data
    kf.updateBaro(BerryIMU.alt);


    //compute the corrected height with base station baro data and offset
    if (baseBaro != 0){
      actualBaro = 44330 * (1 - pow((BerryIMU.comp_press/baseBaro), (1/5.255))); //In meters Base Baro is the pressure

        //publish Height
        height_msg.data = actualBaro;
        RCSOFTCHECK(rcl_publish(&height_publisher, &height_msg, NULL));
    }
    else{
      actualBaro = 1000;
    }

    // Add Z Velocity to a Message and Publish
    z_velocity_msg.data = kf.v;

    // if (check == false){
    //   z_velocity_msg.data = 0;
    // }else{
    //   z_velocity_msg.data = 0;
    // }

    RCSOFTCHECK(rcl_publish(&z_velocity_publisher, &z_velocity_msg, NULL));

    // xekf.updateBaro(CEIL_HEIGHT_FROM_START-actualBaro);
    // yekf.updateBaro(CEIL_HEIGHT_FROM_START-actualBaro);
  }

  //---------------------OPTICAL FLOW LOOP-----------------------------------------
  // //Optical flow update
  //read buffer for optical flow
  // Flow.read_buffer();
  // dt = micros()/MICROS_TO_SEC-lastOpticalLoopTick;
  // if (dt > 1.0/OPTICAL_LOOP_FREQ) {
  //   lastOpticalLoopTick = micros()/MICROS_TO_SEC;
  
  //   Flow.update_flow(BerryIMU.gyr_rateXraw, BerryIMU.gyr_rateYraw, 1);

  //   //change to blimp coordinates
  //   float x_opt = (float)Flow.y_motion/dt;
  //   float y_opt = (float)Flow.x_motion/dt;

  //   xekf.updateOptical(x_opt);
  //   yekf.updateOptical(y_opt);

  //   // Serial.print(">X Velocity:");
  //   // Serial.println(Flow.x_motion_comp);

  //   // accelGCorrection.updateData(BerryIMU.AccXraw, BerryIMU.AccYraw, BerryIMU.AccZraw, pitch, roll);

  //   // kal_vel.update_vel_optical(Flow.x_motion_comp, Flow.y_motion_comp);

  //   // Serial.print(">X Velocity est:");
  //   // Serial.println(kal_vel.x_vel_est);

  // }
  

  //compute slower sensors (if any)
  //use same if statement structure

  //----------------------------STATE MACHINE LOOP-------------------------------
  //compute state machine and motor control at specified frequency
  dt = micros()/MICROS_TO_SEC-lastStateLoopTick;
  if (dt >= 1.0/STATE_MACHINE_FREQ) {
    lastStateLoopTick = micros()/MICROS_TO_SEC;

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
    switch (quad) {
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

    //compute state machine
    if (state == manual) {
      
      //get manual data
      
      //all motor commands are between -1 and 1

        //set max yaw command to 120 deg/s
        

        yawCom = -yaw_msg*120;

        if (USE_EST_VELOCITY_IN_MANUAL == true){
          //set max velocities 2 m/s
          upCom = up_msg*2.0;
          forwardCom = forward_msg*2.0;
          translationCom = translation_msg*2.0;
        }else{
          //normal mapping using max esc command 
          // upCom = up_msg*2.0; //PID used and maxed out at 2m/s
          upCom = -up_msg*500.0;
          // upCom = -up_msg*500.0-0.5*pitch; //pitch correction? (pitch in degrees, conversion factor command/degree)
          forwardCom = forward_msg*500.0;
          translationCom = translation_msg*500.0;
        }


      //check if shooting should be engaged
      //this block switches the state to the oposite that it is currently in

          if (shoot != shootCom) {
          shoot = shootCom;
          //change shoot state
          if (ballGrabber.state == 2) {
            //stop shooting
            ballGrabber.closeGrabber();
          } else {
            //reset catch counter
            catches = 0;
            //go back to searching
            mode = searching;
            searchYawDirection = searchDirection();  //randomize the search direction
            
            //start shooting
            ballGrabber.shoot();
          }

     //check if grabbing should be engaged
     //this block switches the state to the oposite that it is currently in
        } else if (grab != grabCom) {
          grab = grabCom;

          //change grab state
          if (ballGrabber.state == 0) {
            ballGrabber.openGrabber();
          } else {
            ballGrabber.closeGrabber();

            //increase catch counter
            catches++;

            //start catch timmer
            if (catches >= 1) {
              lastCatch = micros()/MICROS_TO_SEC;
            }
          }
        }
      
    } else if (state == autonomous){

      //filter target data
      // float tx = 0;
      // float ty = 0;
      float tx = 0;
      float ty = 0;
      float tz = 0;
      // float area = 0;
      
      //new target (empty target)
      std::vector<double> target;

      //update targets data if any target exists
      //TODO: add area to verify distance 
      //balloon 
      if (targets[2] != 1000 && (mode == searching || mode == approach || mode == catching)) {
        float rawZ = targets[2]; //balloon distance
        //update filtered target coordinates (3D space, with center of camera as (0,0,0))
        // tx = xFilter.filter(targets[0]); (3D)
        // ty = yFilter.filter(targets[1]);
        tx = xFilter.filter(static_cast<float>(pixels[0]));
        ty = yFilter.filter(static_cast<float>(pixels[1]));
        tz = zFilter.filter(rawZ);
        // area = areaFilter.filter(target[0][3]);
        target.push_back(tx);
        target.push_back(ty);
        target.push_back(tz);
      } else {
        //no target, set to default value
        xFilter.filter(0);
        yFilter.filter(0);
        zFilter.filter(0);
        // areaFilter.filter(0);
      }

      //orange goal
      //in goal scoring stages 
      if (targets[5] != 1000 && goalColor == orange && (mode == goalSearch || mode == approachGoal || mode == scoringStart)) {
        float rawZ = targets[5]; //balloon distance
        //update filtered target coordinates (3D space, with center of camera as (0,0,0))
        // tx = xFilter.filter(targets[3]);
        // ty = yFilter.filter(targets[4]); 
        tx = xFilter.filter(static_cast<float>(pixels[3]));
        ty = yFilter.filter(static_cast<float>(pixels[4])); 
        tz = zFilter.filter(rawZ);
        // area = areaFilter.filter(target[0][3]);
        target.push_back(tx);
        target.push_back(ty);
        target.push_back(tz);
      } else {
        //no target, set to default value
        xFilter.filter(0);
        yFilter.filter(0);
        zFilter.filter(0);
        // areaFilter.filter(0);
      }

      //yellow goal
      if (targets[8] != 1000 && goalColor == yellow && (mode == goalSearch || mode == approachGoal || mode == scoringStart)) {
        float rawZ = targets[8]; //balloon distance
        //update filtered target coordinates (3D space, with center of camera as (0,0,0))
        // tx = xFilter.filter(targets[6]);
        // ty = yFilter.filter(targets[7]);
        tx = xFilter.filter(static_cast<float>(pixels[6]));
        ty = yFilter.filter(static_cast<float>(pixels[7]));
        tz = zFilter.filter(rawZ);
        // area = areaFilter.filter(target[0][3]);
        target.push_back(tx);
        target.push_back(ty);
        target.push_back(tz);
      } else {
        //no target, set to default value
        xFilter.filter(0);
        yFilter.filter(0);
        zFilter.filter(0);
        // areaFilter.filter(0);
      }
        //test target message

        // if (target.size() != 0){
        // debug_msg.data.data[0] = target[0];
        // debug_msg.data.data[1] = target[1];
        // debug_msg.data.data[2] = target[2];
        // debug_msg.data.size = 3;
        // }

        // RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));

      //modes for autonomous behavior
      switch (mode) {
        //state machine
        case searching:
          //check if goal scoring should be attempted

          if (catches >= 1 && ((micros()/MICROS_TO_SEC - lastCatch) >= (MAX_SEARCH_WAIT_AFTER_ONE - (catches-1)*GAME_BALL_WAIT_TIME_PENALTY))) {
            catches = TOTAL_ATTEMPTS;
            mode = goalSearch;
            goalYawDirection = searchDirection();  //randomize search direction
            break;
          }


          if (catches >= TOTAL_ATTEMPTS) {
            mode = goalSearch;
            goalYawDirection = searchDirection();  //randomize search direction
            break;
          }


          //begin search pattern spinning around at different heights
          if (target.size() == 0) {
            //search behavoir (no target)
            //spin in a small circle looking for a game ball
            //randomize the diretion selection

            yawCom = GAME_BALL_YAW_SEARCH*searchYawDirection;
            upCom = 0;    //is overriden later, defined here as a safety net
            forwardCom = GAME_BALL_FORWARD_SEARCH;

            //keep ball grabber closed
            ballGrabber.closeGrabber();

            //use object avoidence
            if (quad != 10 && USE_OBJECT_AVOIDENCE) {
              //overide search commands
              //yawCom = yawA;
              forwardCom = forwardA;
            }

            //move up and down within the set boundry
            if (actualBaro > CEIL_HEIGHT || !wasUp) {
              if (wasUp) wasUp = false;
              upCom = -GAME_BALL_VERTICAL_SEARCH;
            }
            if (actualBaro < FLOOR_HEIGHT || wasUp) {
              if (!wasUp) wasUp = true;
              upCom = GAME_BALL_VERTICAL_SEARCH;
            }

          } else {
            //move to approaching game ball
            mode = approach;
            //start approaching timer
                approachTimeStart = millis();
          }
          break;

        case approach:
          //check if target is still valid
          if (target.size() > 0) {

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
              ballGrabber.openGrabber();

              //check if the catching mode should be triggered
              if (tz < BALL_CATCH_TRIGGER && pixels[2] > 85000) {
                mode = catching;
                
                //start catching timer
                catchTimeStart = millis();

                //turn motors off
                motorControl.update(0,0,0,0,0);
              }
            } else {
              //make sure grabber is closed, no game ball is close enough to catch
              ballGrabber.closeGrabber();
            }
            
          } else {
            //no target, look for another
            mode = searching;
            searchYawDirection = searchDirection();  //randomize the search direction
          }
          break;

        case catching:
          if (true) {
            //wait for 0.3 second
            // delay(300);

            forwardCom = CATCHING_FORWARD_COM;
            upCom = CATCHING_UP_COM;
            yawCom = 0;
            translationCom = 0;
    
            if (catchTimeStart < millis() - catchTime) {
              //catching ended, start caught timer

              mode = caught;
              caughtTimeStart = millis();
              ballGrabber.closeGrabber();

              //increment number of catches
              catches = catches + 1;

              //start catch timmer
              lastCatch = micros()/MICROS_TO_SEC;
            }
          }
          break;

        case caught:
          if (catches > 0) {

            //if a target is seen right after the catch
            if (target.size() > 0) {
              //approach next game ball if visible
              if (catches < TOTAL_ATTEMPTS) {
                mode = searching;
                searchYawDirection = searchDirection();  //randomize the search direction
              }
            }

            //decide if the blimp is going to game ball search or goal search
            if (caughtTimeStart < millis() - caughtTime) {
              if (catches >= TOTAL_ATTEMPTS) {
                mode = goalSearch;
                goalYawDirection = searchDirection();  //randomize search direction
              } else {
                mode = searching;
                searchYawDirection = searchDirection();  //randomize the search direction
              }
            }
          
            forwardCom = CAUGHT_FORWARD_COM;
            upCom = CAUGHT_UP_COM;
            yawCom = 0;
            
          } else {
            mode = searching;
            searchYawDirection = searchDirection();  //randomize the search direction
          }
          break;

        case goalSearch:
          if (catches >= TOTAL_ATTEMPTS) {
            //randomize the diretion selection
            yawCom = GOAL_YAW_SEARCH*goalYawDirection;
            // upCom = goalPositionHold.calculate(GOAL_HEIGHT, actualBaro);
            upCom = GOAL_UP_VELOCITY;
            forwardCom = GOAL_FORWARD_SEARCH;
            ballGrabber.closeGrabber();

            //implement avoidence here
            if (quad != 10 && USE_OBJECT_AVOIDENCE) {
              yawCom = yawA;
              forwardCom = forwardA;
              goalYawDirection = random(0, 9);
              if (yawCom > 0) {
                goalYawDirection = 1;
              } else if (yawA < 0) {
                goalYawDirection = -1;
              }
            }
            
            if (target.size() > 0) {
              mode = approachGoal;
            }
          } else {
            mode = searching;
            searchYawDirection = searchDirection();  //randomize the search direction
          }
          break;

        case approachGoal:
          if (target.size() > 0 && catches >= TOTAL_ATTEMPTS) {
            yawCom = xPID.calculate(GOAL_X_OFFSET, tx, dt);
            upCom = -yPID.calculate(GOAL_APPROACH_ANGLE, ty, dt);
            forwardCom = GOAL_CLOSURE_COM;

            if ((tz < GOAL_DISTANCE_TRIGGER && goalColor == orange && pixels[5] > 203000) || (tz < GOAL_DISTANCE_TRIGGER && goalColor == yellow && pixels[8] > 203000)) {
              scoreTimeStart = millis();
              mode = scoringStart;
            }
          } else {
            mode = goalSearch;
            goalYawDirection = searchDirection();  //randomize search direction
          }
          break;
          //after correction, we can do goal alignment with a yaw and a translation 

        case scoringStart:
        if (true) {
          yawCom = SCORING_YAW_COM;
          forwardCom = SCORING_FORWARD_COM;
          upCom = -SCORING_UP_COM;
  
          if (scoreTimeStart < millis() - scoreTime) {
            mode = shooting;     
            shootingTimeStart = millis();
            break;
          }
        }
          break;

        case shooting:
        if (true) {
          yawCom = 0;
          forwardCom = SHOOTING_FORWARD_COM;
          upCom = -SHOOTING_UP_COM;

          ballGrabber.shoot();
          catches = 0;
  
          if (shootingTimeStart < millis() - shootingTime) {
            ballGrabber.closeGrabber();
            scoredTimeStart = millis();
            mode = scored;
            break;
          }
        }
        break;

        case scored:
        if (true) {

          ballGrabber.closeGrabber();

          yawCom = 0;
          forwardCom = SCORED_FORWARD_COM;
          upCom = SCORED_UP_COM;
  
          if (scoredTimeStart < millis() - scoredTime) {
            mode = searching;
            searchYawDirection = searchDirection();  //randomize the search direction
            break;
          }
        }
        //shouldn't get here
        default:
          yawCom = 0;
          forwardCom = 0;
          upCom = 0;
          break;
      }

    }else {
        //not a valid state
        forwardCom = 0.0;
        upCom = 0.0;
        yawCom = 0.0;
    }


    //publish state machine info to Basestation
    state_machine_msg.data = mode;
    RCSOFTCHECK(rcl_publish(&state_machine_publisher, &state_machine_msg, NULL));


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
         float deadband = 1.0; //deadband for filteration
         yawMotor = yawPID.calculate(yawCom, yawRateFilter.last, dt);  

         if (abs(yawCom-yawRateFilter.last) < deadband) {
             yawMotor = 0;
         } else {
             yawMotor = tanh(yawMotor)*abs(yawMotor);
         }

    //TO DO: im prove velocity control
    // upMotor = verticalPID.calculate(upCom, kf.v, dt); //up velocity from barometer
    upMotor = upCom;

    if (USE_EST_VELOCITY_IN_MANUAL == true){
    //using kalman filters for the current velosity feedback for full-state feeback PID controllers
    // forwardMotor = forwardPID.calculate(forwardCom, xekf.v, dt);  //extended filter
    // float forwardMotor = forwardPID.calculate(forwardCom, kal_vel.x_vel_est, dt);
    // translationMotor = translationPID.calculate(translationCom, yekf.v, dt); //extended filter
    // float translationMotor = translationPID.calculate(translationCom, kal_vel.y_vel_est, dt); 
    }else{
      //without PID
      forwardMotor = forwardCom;
      translationMotor = translationCom;
    }

        //motor debug
        
        debug_msg.data.data[0] = yawMotor;
        debug_msg.data.data[1] = upMotor;
        debug_msg.data.data[2] = translationMotor;
        debug_msg.data.data[3] = forwardMotor;
        debug_msg.data.size = 4;

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

        RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));

    //Serial.print(">up current:");
    //Serial.println(kf.v);
    //float yawCurrent =  (float)yawRateFilter.last;
    //Serial.print(">yaw current");
    //Serial.println(yawCurrent);
    //Serial.print(">forward current:");
    //Serial.println(xekf.v);
    // Serial.print(">translation current:");
    //Serial.println(yekf.v);

    //gimbal + motor updates

    if (micros()/MICROS_TO_SEC < 10 + firstMessageTime) {
      //filter base station data
      baroOffset.filter(baseBaro-BerryIMU.alt);
      rollOffset.filter(BerryIMU.gyr_rateXraw);

      //zero motors while filters converge and esc arms
      motorControl.update(0, 0, 0, 0, 0);
      bool leftReady = leftGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, motorControl.yawLeft, motorControl.upLeft, motorControl.forwardLeft);
      bool rightReady = rightGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, motorControl.yawRight, motorControl.upRight, motorControl.forwardRight);
      leftGimbal.updateGimbal(leftReady && rightReady);
      rightGimbal.updateGimbal(leftReady && rightReady);

    } else {
      if (state == manual && !MOTORS_OFF){
        //forward, translation, up, yaw, roll
        if (!ZERO_MODE) motorControl.update(forwardMotor, -translationMotor, upMotor, yawMotor, 0);
        bool leftReady = leftGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, motorControl.upLeft, motorControl.forwardLeft);
        bool rightReady = rightGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, motorControl.upRight, motorControl.forwardRight);
        leftGimbal.updateGimbal(leftReady && rightReady);
        rightGimbal.updateGimbal(leftReady && rightReady);
      } else if (state == autonomous && !MOTORS_OFF) {
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
      }else {
        motorControl.update(0,0,0,0,0);
        bool leftReady = leftGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, 0, 0);
        bool rightReady = rightGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, 0, 0);
        leftGimbal.updateGimbal(leftReady && rightReady);
        rightGimbal.updateGimbal(leftReady && rightReady);
      }
    }
  }
}
