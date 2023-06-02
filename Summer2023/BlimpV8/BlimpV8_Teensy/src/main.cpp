#include <Arduino.h>

#include <micro_ros_platformio.h>

#include <vector>
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
#include <std_msgs/msg/float64_multi_array.h>



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
#define MAX_SEARCH_WAIT_AFTER_ONE     80    //max searching 
#define GAME_BALL_WAIT_TIME_PENALTY   0    //should be set to 20, every catch assumed to be 20 seconds long  

//number of catches attempted
#define TOTAL_ATTEMPTS            100    // attempts at catching 
#define MAX_ATTEMPTS              100    //should be set to 5

//flight area parameters
#define CEIL_HEIGHT               8      //m
#define FLOOR_HEIGHT              2.5    //m

#define MAX_HEIGHT                2    //m
#define GOAL_HEIGHT               5.0    //m
#define GOAL_HEIGHT_DEADBAND      0.3       //m

//distance triggers
#define GOAL_DISTANCE_TRIGGER    1.5 //m distance for blimp to trigger goal score 	
#define BALL_GATE_OPEN_TRIGGER   2.5 //m distance for blimp to open the gate 	
#define BALL_CATCH_TRIGGER       1  //m distance for blimp to start the open-loop control

//object avoidence motor coms
#define FORWARD_AVOID             250  //25% throttle
#define YAW_AVOID                 10	 //deg/s
#define UP_AVOID                  0.4  //m/s

//autonomy tunning parameters
// the inputs are bounded from -2 to 2, yaw is maxed out at 120 deg/s
#define GAME_BALL_YAW_SEARCH      -15  //deg/s
#define GAME_BALL_FORWARD_SEARCH  250 //50% throttle 
#define GAME_BALL_VERTICAL_SEARCH 0.7  //m/s


#define GAME_BALL_CLOSURE_COM     200  //approaching at 20% throttle cap
#define GAME_BALL_APPROACH_ANGLE  -80  //0.2 approach magic number
#define GAME_BaLL_X_OFFSET        0   //10 offset magic number

#define CATCHING_FORWARD_COM      450  //catching at 90% throttle 
#define CATCHING_UP_COM           1  //damp out pitch

#define CAUGHT_FORWARD_COM        -420  //go back so that the game ball gets to the back 
#define CAUGHT_UP_COM             -0.5

#define GOAL_YAW_SEARCH           20   
#define GOAL_FORWARD_SEARCH       300  //200 40% throttle
#define GOAL_UP_VELOCITY          2.5

#define GOAL_CLOSURE_COM          400  //forward command 25% throttle
#define GOAL_X_OFFSET             -10
#define GOAL_APPROACH_ANGLE       -30  //height alignment (approach down)

//goal alignment test
#define ALIGNING_YAW_COM           10 //test
#define ALIGNING_FORWARD_COM        100 //test
#define ALIGNING_UP_COM            0.4 //test
#define ALIGNING_TRANSLATION_COM   300 //test


#define SCORING_YAW_COM           0
#define SCORING_FORWARD_COM       500 //40% throttle
#define SCORING_UP_COM            1.5

#define SHOOTING_FORWARD_COM      700  //counter back motion 
#define SHOOTING_UP_COM           0.3
//counter moment (right now we do want to shoot up because ball sinks)

#define SCORED_FORWARD_COM        -700
#define SCORED_UP_COM             -0.2

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
#define BUFFER_LEN                300

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
PID verticalPID(-320, 0, 0);  
PID yawPID(5.0, 0, 0);
PID forwardPID(300, 0, 0);  //not used
PID translationPID(300, 0, 0); //not used

//Auto PID control (output fed into manual controller)
PID yPixelPID(0.0075,0,0);
PID xPixelPID(0.2,0,0);

//Goal positioning controller
BangBang goalPositionHold(GOAL_HEIGHT_DEADBAND, GOAL_UP_VELOCITY); //Dead band, velocity to center itself

//pre process for accel before vertical kalman filter
EMAFilter verticalAccelFilter(0.05);

//filter on yaw gyro
EMAFilter yawRateFilter(0.2);

EMAFilter rollRateFilter(0.5);

//Low pass filter for computer vision parameters
EMAFilter xPixFilter(0.5);
EMAFilter yPixFilter(0.5);
EMAFilter zPixFilter(0.5);
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

//to Do: make two type of game ball color tracking
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

float rotation = 180;

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

float goalYawDirection = -1;

//------------------MICRO ROS publishers/subscribers--------------
//ROS node
rclc_executor_t executor_pub;
rclc_executor_t executor_sub;


rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

//ROS publishers
rcl_publisher_t blimpid_publisher;
rcl_publisher_t imu_publisher;    
rcl_publisher_t motor_publisher; 
rcl_publisher_t height_publisher;
rcl_publisher_t z_velocity_publisher;

//ROS subscribers
rcl_subscription_t identity_subscription; //boolean
rcl_subscription_t auto_subscription; //boolean
rcl_subscription_t baseBarometer_subscription; //float64
rcl_subscription_t grabber_subscription; //boolean
rcl_subscription_t shooter_subscription; //boolean
rcl_subscription_t motor_subscription; //float64_multi_array
rcl_subscription_t kill_subscription; //boolean


//message types: String Bool Float32 Float32 MultiArray
//message topics : /auto /baseBarometer /blimpID /grabbing /killed /motorCommands /shooting /identify /imu

//bolean message
std_msgs__msg__Bool identity_msg;
std_msgs__msg__Bool auto_msg;
std_msgs__msg__Bool grab_msg;
std_msgs__msg__Bool shoot_msg;
std_msgs__msg__Bool kill_msg;

//float64 message
std_msgs__msg__Float64  baro_msg;
std_msgs__msg__Float64  height_msg;
std_msgs__msg__Float64  z_velocity_msg;

//float64multiarray message
std_msgs__msg__Float64MultiArray motor_msg;
std_msgs__msg__Float64MultiArray motor_debug_msg;

//String message
std_msgs__msg__String blimpid_msg;
const char* burn_cream_str = "BurnCreamBlimp"; //const char message
int counter = 0;

//sensor message
sensor_msgs__msg__Imu imu_msg;

//timer call back
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
  // snprintf(msg.data.data, BUFFER_LEN, "%s for the %dth time!", burn_cream_str, counter++);
  snprintf(blimpid_msg.data.data, BUFFER_LEN, burn_cream_str, "BurnCreamBlimp");
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
    ballGrabber.closeGrabber();

    //start catching timer if catching was attempted
    if (catches >= 1) {
      lastCatch = micros()/MICROS_TO_SEC;
    }

  } else if (grab_msg->data == true){
    ballGrabber.openGrabber();
    //increase catch counter
    catches++;
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
    ballGrabber.closeGrabber();

  } else if (shoot_msg->data == true){
    //startshooting
    ballGrabber.shoot();
    //reset cathes counter
    catches = 0;
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


bool create_entities() {
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "BurnCreamBlimp", "", &support)); //name the robot

  // create publishers
  RCCHECK(rclc_publisher_init_default(&blimpid_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/BurnCreamBlimp/blimpID"));
  RCCHECK(rclc_publisher_init_default(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/BurnCreamBlimp/imu"));
  RCCHECK(rclc_publisher_init_default(&motor_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg,Float64MultiArray ), "/BurnCreamBlimp/motorDebug"));
  RCCHECK(rclc_publisher_init_default(&height_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg,Float64), "/BurnCreamBlimp/height"));
  RCCHECK(rclc_publisher_init_default(&z_velocity_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg,Float64), "/BurnCreamBlimp/z_velocity"));

  //create subscribers
  RCCHECK(rclc_subscription_init_default(&identity_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/identify"));
  RCCHECK(rclc_subscription_init_default(&auto_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/BurnCreamBlimp/auto"));
  RCCHECK(rclc_subscription_init_default(&baseBarometer_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "/BurnCreamBlimp/baseBarometer"));
  RCCHECK(rclc_subscription_init_default(&grabber_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/BurnCreamBlimp/grabbing"));
  RCCHECK(rclc_subscription_init_default(&shooter_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/BurnCreamBlimp/shooting"));
  RCCHECK(rclc_subscription_init_default(&motor_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray), "/BurnCreamBlimp/motorCommands"));
  RCCHECK(rclc_subscription_init_default(&kill_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/BurnCreamBlimp/killed"));

  // create timer
  const unsigned int timer_period = 10;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_period), timer_callback));

  // create executor
  executor_pub = rclc_executor_get_zero_initialized_executor();
  executor_sub = rclc_executor_get_zero_initialized_executor();

  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));

  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 8, &allocator));

  //add all subscriptions
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &identity_subscription, &identity_msg, &id_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &auto_subscription, &auto_msg, &auto_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &baseBarometer_subscription, &baro_msg, &baro_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &grabber_subscription, &grab_msg, &grab_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &shooter_subscription, &shoot_msg, &shoot_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &kill_subscription, &kill_msg, &kill_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &motor_subscription, &motor_msg, &motor_subscription_callback, ON_NEW_DATA));

  return true;
}

void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  //finish publishers
  rcl_publisher_fini(&blimpid_publisher, &node);
  rcl_publisher_fini(&imu_publisher, &node);
  rcl_publisher_fini(&motor_publisher, &node);
  rcl_publisher_fini(&height_publisher, &node);
  rcl_publisher_fini(&z_velocity_publisher, &node);

  //finnish subscriptions
  rcl_subscription_fini(&identity_subscription, &node);
  rcl_subscription_fini(&auto_subscription, &node);
  rcl_subscription_fini(&baseBarometer_subscription, &node);
  rcl_subscription_fini(&grabber_subscription, &node);
  rcl_subscription_fini(&shooter_subscription, &node);
  rcl_subscription_fini(&motor_subscription, &node);
  rcl_subscription_fini(&kill_subscription, &node);

  rcl_timer_fini(&timer);
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

  imu_msg.header.frame_id.data = (char *) malloc(BUFFER_LEN*sizeof(char));
  imu_msg.header.frame_id.size = 0;
  imu_msg.header.frame_id.capacity = BUFFER_LEN;


  blimpid_msg.data.data = (char *) malloc(BUFFER_LEN*sizeof(char));
  blimpid_msg.data.size = strlen(blimpid_msg.data.data);
  blimpid_msg.data.capacity = BUFFER_LEN;

  motor_msg.data.data = (double *) malloc(BUFFER_LEN*sizeof(double));
  motor_msg.data.size = sizeof(motor_msg.data.data);
  motor_msg.data.capacity = BUFFER_LEN;

  motor_debug_msg.data.data = (double *) malloc(BUFFER_LEN*sizeof(double));
  motor_debug_msg.data.size = 0; 
  motor_debug_msg.data.capacity = BUFFER_LEN;


  delay(2000);

  //time out
  firstMessageTime = micros()/MICROS_TO_SEC;
}


//loop

void loop() {
  //-----------------------MICRO ROS PUBLISHER and SUBSCRIBERS--------------------------------------
  // publisher state machine
  // checking if the agent is available 
  switch (agent_state_) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, agent_state_ = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      agent_state_ = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (agent_state_ == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(100, agent_state_ = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (agent_state_ == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100));
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
    snprintf(imu_msg.header.frame_id.data, BUFFER_LEN, "BurnCreamBlimp");
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
    verticalAccelFilter.filter(-accelGCorrection.agz);

    //update vertical velocity kalman filter acceleration
    kf.updateAccel(verticalAccelFilter.last);

    //update filtered yaw rate
    yawRateFilter.filter(BerryIMU.gyr_rateZraw);

    //perform gyro update
    gyroEKF.updateGyro(BerryIMU.gyr_rateXraw*3.14/180, BerryIMU.gyr_rateYraw*3.14/180, BerryIMU.gyr_rateZraw*3.14/180);
    gyroEKF.updateAccel(BerryIMU.AccXraw, BerryIMU.AccYraw, BerryIMU.AccZraw);
    

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
        height_msg.data = BerryIMU.alt;
        RCSOFTCHECK(rcl_publish(&height_publisher, &height_msg, NULL));
    }
    else{
      actualBaro = 1000;
    }

    // Add Z Velocity to a Message and Publish
    z_velocity_msg.data = kf.v;
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
          upCom = up_msg*2.0; //PID used and maxed out at 2m/s
          forwardCom = forward_msg*500.0;
          translationCom = translation_msg*500.0;
        }

        //motor debug
        motor_debug_msg.data.data[0] = yaw_msg;
        motor_debug_msg.data.data[1] = up_msg;
        motor_debug_msg.data.data[2] = translation_msg;
        motor_debug_msg.data.data[3] = forward_msg;
        motor_debug_msg.data.size = 4;
        RCSOFTCHECK(rcl_publish(&motor_publisher, &motor_debug_msg, NULL));
      
    } else if (state == autonomous){
      /*
        //get auto data
      std::vector<std::vector<double>> target = piData.target;

      //filter target data
      float tx = 0;
      float ty = 0;
      float tz = 200;
      float area = 0;

      //update target data if target exists
      if (target.size() > 0 && target[0].size() == 4) {
        float rawZ = target[0][2];
        //if distance is too large, cap distance value
        if (rawZ > 300) {
          rawZ = 300;
        }

        //update filtered target coordinates (3D space, with center of camera as (0,0,0))
        tx = xPixFilter.filter(-target[0][0]);
        ty = yPixFilter.filter(-target[0][1]);
        tz = zPixFilter.filter(rawZ);
        area = areaFilter.filter(target[0][3]);
        
      } else {
        //no target, set to default value
        xPixFilter.filter(0);
        yPixFilter.filter(0);
        zPixFilter.filter(300);
        areaFilter.filter(0);
      }

      //modes for autonomous behavior
      switch (mode) {
        case searching:
          //check if goal scoring should be attempted

          if (catches >= 1 && micros()/MICROS_TO_SEC - lastCatch >= MAX_SEARCH_WAIT_AFTER_ONE - (catches-1)*(GAME_BALL_WAIT_TIME_PENALTY)) {
            catches = TOTAL_ATTEMPTS;
            mode = goalSearch;
            break;
          }


          if (catches >= TOTAL_ATTEMPTS) {
            mode = goalSearch;
            break;
          }

          //begin search pattern spinning around at different heights
          if (target.size() == 0) {
            //search behavoir
            //spin in a small circle looking for a game ball
            yawCom = GAME_BALL_YAW_SEARCH;
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
          if (target.size() > 0 && target[0].size() == 4) {

            if (catches >= 1 && micros()/MICROS_TO_SEC - lastCatch >= MAX_SEARCH_WAIT_AFTER_ONE - (catches-1)*(GAME_BALL_WAIT_TIME_PENALTY)) {
              catches = TOTAL_ATTEMPTS;
              mode = goalSearch;
              break;
            }

            if (catches >= TOTAL_ATTEMPTS) {
              mode = goalSearch;
            }

            //move toward the balloon
            yawCom = xPixelPID.calculate(tx, GAME_BaLL_X_OFFSET, dt/1000);
            upCom = yPixelPID.calculate(ty, GAME_BALL_APPROACH_ANGLE, dt/1000);
            forwardCom = GAME_BALL_CLOSURE_COM;
            translationCom = 0;
            
            //check if the gate should be opened
            if (tz < BALL_GATE_OPEN_TRIGGER) {
              ballGrabber.openGrabber();

              //check if the catching mode should be triggered
              if (tz < BALL_CATCH_TRIGGER) {
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
          }

          break;
        case catching:
          if (true) {
            //wait for 0.3 second
            delay(300);

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

            if (target.size() > 0 && target[0].size() == 4) {
              //approach next game ball if visible
              if (catches < TOTAL_ATTEMPTS) {
                mode = searching;
              }
            }
    
            if (caughtTimeStart < millis() - caughtTime) {
              if (catches >= TOTAL_ATTEMPTS) {
                mode = goalSearch;
              } else {
                mode = searching;
              }
            }
          
            forwardCom = CAUGHT_FORWARD_COM;
            upCom = CAUGHT_UP_COM;
            yawCom = 0;
            
          } else {
            mode = searching;
          }

          break;
        case goalSearch:
          if (catches >= TOTAL_ATTEMPTS) {
            yawCom = GOAL_YAW_SEARCH*goalYawDirection;
            upCom = goalPositionHold.calculate(GOAL_HEIGHT, actualBaro);
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
          }
          break;
        case approachGoal:
          if (target.size() > 0 && catches >= TOTAL_ATTEMPTS) {
            yawCom = xPixelPID.calculate(tx, 0, dt);
            upCom = yPixelPID.calculate(ty, GOAL_APPROACH_ANGLE, dt);
            forwardCom = GOAL_CLOSURE_COM;

            Serial.println(tz);
            print("tz", tz);

            if (tz < GOAL_DISTANCE_TRIGGER) {
              mode = scoringStart;
              scoreTimeStart = millis();
            }
          } else {
            mode = goalSearch;
          }
          break;
          //after correction, we can do goal alignment with a yaw and a translation 

        case scoringStart:
        if (true) {
          yawCom = SCORING_YAW_COM;
          forwardCom = SCORING_FORWARD_COM;
          upCom = SCORING_UP_COM;
  
          if (scoreTimeStart < millis() - scoreTime) {
            mode = shooting;     //change this to add shooting mode
            shootingTimeStart = millis();
            break;
          }
        }
          break;
        case shooting:
        if (true) {
          yawCom = 0;
          forwardCom = SHOOTING_FORWARD_COM;
          upCom = SHOOTING_UP_COM;

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
            break;
          }
        }
        default:
          yawCom = 0;
          forwardCom = 0;
          upCom = 0;
          break;
      }
      */
    }else {
        //not a valid state
        forwardCom = 0.0;
        upCom = 0.0;
        yawCom = 0.0;
    }

     //safty height 
    if (actualBaro > MAX_HEIGHT) {
        upCom = -1;
    }

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
    upMotor = verticalPID.calculate(upCom, kf.v, dt); //up velocity from barometer

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
    motor_debug_msg.data.data[0] = yawMotor;
    motor_debug_msg.data.data[1] = upMotor;
    motor_debug_msg.data.data[2] = translationMotor;
    motor_debug_msg.data.data[3] = forwardMotor;
    motor_debug_msg.data.size = 4;
    RCSOFTCHECK(rcl_publish(&motor_publisher, &motor_debug_msg, NULL));

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
