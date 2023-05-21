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

#include <std_msgs/msg/string.h> //include the message type that needs to be published (teensy data)
#include <micro_ros_utilities/string_utilities.h>

#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/vector3.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <std_msgs/msg/bool.h>

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

#define MAX_HEIGHT                8    //m
#define GOAL_HEIGHT               5.0    //m
#define GOAL_HEIGHT_DEADBAND      0.3       //m

//distance triggers
#define GOAL_DISTANCE_TRIGGER     65  //unscaled distance for blimp to trigger goal score 
#define BALL_GATE_OPEN_TRIGGER    22 //30 unscaled distance for blimp to open the gate
#define BALL_CATCH_TRIGGER        19 //27 unscaled distance for blimp to start the open-loop control 

//object avoidence motor coms
#define FORWARD_AVOID             0.5 
#define YAW_AVOID                 10
#define UP_AVOID                  0.1

//autonomy tunning parameters
// the inputs are bounded from -2 to 2, yaw is maxed out at 120 deg/s
#define GAME_BALL_YAW_SEARCH      -20  
#define GAME_BALL_FORWARD_SEARCH  0.4  //0.4 = 20% 
#define GAME_BALL_VERTICAL_SEARCH 0.15

#define GAME_BALL_CLOSURE_COM     0.2  //approaching at 20% throttle
#define GAME_BALL_APPROACH_ANGLE  0.1  //descend or ascend at 5% throttle
#define GAME_BaLL_X_OFFSET        10   //adjusting yaw at 10 deg/s

#define CATCHING_FORWARD_COM      0.8  //catching at 40% throttle 
#define CATCHING_UP_COM           0.2

#define CAUGHT_FORWARD_COM        -0.82  //go back so that the game ball gets to the back 
#define CAUGHT_UP_COM             -0.2

#define GOAL_YAW_SEARCH           15   
#define GOAL_FORWARD_SEARCH       0.4  //20% throttle
#define GOAL_UP_VELOCITY          0.1  

#define GOAL_CLOSURE_COM          0.3 
#define GOAL_APPROACH_ANGLE       0

#define SCORING_YAW_COM           0
#define SCORING_FORWARD_COM       0.18
#define SCORING_UP_COM            0.1

#define SHOOTING_FORWARD_COM      0.6
#define SHOOTING_UP_COM           0.2

#define SCORED_FORWARD_COM        -0.6
#define SCORED_UP_COM             -0.0015

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

//init pins

#define LYSPIN                     7
#define LPSPIN                     6
#define LMPIN                     2
#define RYSPIN                     10
#define RPSPIN                     9
#define RMPIN                     5

#define GRABBER_PIN               8
#define SHOOTER_PIN               4


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
Gimbal leftGimbal(7, 6, 2, 25, 30, 1000, 2000, 45, 0.2);
Gimbal rightGimbal(10, 9, 5, 25, 30, 1000, 2000, 135, 0.2);

//Manual PID control
PID verticalPID(500, 0, 0);  
PID yawPID(20.0, 0, 0);
PID forwardPID(500, 0, 0);  
PID translationPID(500, 0, 0);

//Auto PID control (output fed into manual controller)
PID yPixelPID(0.0075,0,0);
PID xPixelPID(0.162,0,0);

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
TripleBallGrabber ballGrabber(GRABBER_PIN,SHOOTER_PIN);

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

//sensor data
float pitch = 0;
float yaw = 0;
float roll = 0;

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
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

//ROS publishers
rcl_publisher_t burn_publisher;
rcl_publisher_t imu_publisher;    
// rcl_publisher_t tf_publisher;

//ROS subscribers
rcl_subscription_t identity_subscription;

//boolean messgae
std_msgs__msg__Bool identity_msg;

//String message
std_msgs__msg__String msg;
const char* burn_cream_str = "Got'Em, get the burn cream!"; //const char message
int counter = 0;

//sensor message
sensor_msgs__msg__Imu imu_msg;

//transform message
geometry_msgs__msg__TransformStamped tf_msg;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
  snprintf(msg.data.data, BUFFER_LEN, "%s for the %dth time!", burn_cream_str, counter++);
  msg.data.size = strlen(msg.data.data);
  msg.data.capacity = BUFFER_LEN;
  RCSOFTCHECK(rcl_publish(&burn_publisher, &msg, NULL));
  }
}

// Functions create_entities and destroy_entities can take several seconds.
// In order to reduce this rebuild the library wif
// - UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS=3

bool create_entities() {
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "burn_cream_blimp", "", &support)); //name the robot

  // create publishers
  RCCHECK(rclc_publisher_init_default(&burn_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/burn_cream"));
  RCCHECK(rclc_publisher_init_default(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu"));
  // RCCHECK(rclc_publisher_init_default(&tf_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TransformStamped), "/transform"));

  //create subscribers
  RCCHECK(rclc_subscription_init_default(&identity_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/identity"));

  // create timer
  const unsigned int timer_period = 10;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_period), timer_callback));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  return true;
}

void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&burn_publisher, &node);
  rcl_publisher_fini(&imu_publisher, &node);
  // rcl_publisher_fini(&tf_publisher, &node);
  rcl_subscription_fini(&identity_subscription, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

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

void setup() {
  //start serial connection
  Serial1.begin(115200);
  Serial.begin(115200);
  set_microros_serial_transports(Serial); //to pi
  agent_state_ = WAITING_AGENT; //wait for connection

  //initialize
  imu_msg.header.frame_id.data = (char *) malloc(BUFFER_LEN*sizeof(char));
  imu_msg.header.frame_id.size = 0;
  imu_msg.header.frame_id.capacity = BUFFER_LEN;

  // tf_msg.header.frame_id.data = (char *) malloc(BUFFER_LEN*sizeof(char));
  // tf_msg.header.frame_id.size = 0;
  // tf_msg.header.frame_id.capacity = BUFFER_LEN;

  msg.data.data = (char *) malloc(BUFFER_LEN*sizeof(char));
  msg.data.size = strlen(msg.data.data);
  msg.data.capacity = BUFFER_LEN;

  delay(2000);
}

void loop() {
  //-----------------------MICRO ROS PUBLISHER--------------------------------------
  //publisher state machine
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
      EXECUTE_EVERY_N_MS(200, agent_state_ = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (agent_state_ == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      agent_state_ = WAITING_AGENT;
      break;
    default:
      break;
  }

  //----------------------------IMU LOOP-----------------------------------------
  //compute accel, gyro, madwick loop time at the set freqency
  float dt = micros()/MICROS_TO_SEC-lastSensorFastLoopTick;
  if (dt >= 1.0/FAST_SENSOR_LOOP_FREQ) {
    lastSensorFastLoopTick = micros()/MICROS_TO_SEC;

    //read sensor values and update madgwick
    BerryIMU.IMU_read();
    madgwick.Madgwick_Update(BerryIMU.gyr_rateXraw,
                             BerryIMU.gyr_rateYraw,
                             BerryIMU.gyr_rateZraw,
                             BerryIMU.AccXraw,
                             BerryIMU.AccYraw,
                             BerryIMU.AccZraw);

        //publish imu data
    snprintf(imu_msg.header.frame_id.data, BUFFER_LEN, "Burn Cream Blimp");
    imu_msg.header.frame_id.size = strlen(msg.data.data);
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

    // //publish tranform data
    // snprintf(tf_msg.header.frame_id.data, BUFFER_LEN, "Burn Cream Blimp");
    // tf_msg.header.frame_id.size = strlen(msg.data.data);
    // tf_msg.header.frame_id.capacity = BUFFER_LEN;

    // tf_msg.header.stamp.sec = (unsigned int)((double)now/1000000);
    // tf_msg.header.stamp.nanosec = (now % 1000000) * 1000;

    //     // x,y,z translation
    //     tf_msg.transform.translation.x = 0.0;
    //     tf_msg.transform.translation.y = 0.0;
    //     tf_msg.transform.translation.z = 0.0;

    //     // rotation in quaternion
    //     tf_msg.transform.rotation.w = madgwick.q1;
    //     tf_msg.transform.rotation.x = madgwick.q2;
    //     tf_msg.transform.rotation.y = madgwick.q3;
    //     tf_msg.transform.rotation.z = madgwick.q4;
        
    // RCSOFTCHECK(rcl_publish(&tf_publisher, &tf_msg, NULL));

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

    kal_vel.predict_vel();
    kal_vel.update_vel_acc(-accelGCorrection.agx/9.81, -accelGCorrection.agy/9.81);
  }

  //-----------------------------BARO LOOP----------------------
  //update barometere at set barometere frequency
  dt = micros()/MICROS_TO_SEC-lastBaroLoopTick;
  if (dt >= 1.0/BARO_LOOP_FREQ) {
    lastBaroLoopTick = micros()/MICROS_TO_SEC;

    //get most current imu values
    BerryIMU.IMU_read();
    
    //update kalman with uncorreced barometer data
    kf.updateBaro(BerryIMU.alt);


    //compute the corrected height with base station baro data and offset
    actualBaro = BerryIMU.alt - baseBaro + baroOffset.last;

    // xekf.updateBaro(CEIL_HEIGHT_FROM_START-actualBaro);
    // yekf.updateBaro(CEIL_HEIGHT_FROM_START-actualBaro);
  }

  //---------------------OPTICAL FLOW LOOP----------------------------
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

    //compute state machine
    if (state == manual) {
      /*
      //Serial.println("Manual");
      //get manual data
      //TO DO: new communication
      // std::vector<double> manualComs = piData.getManualComs();
      
      //all motor commands are between -1 and 1

      //check to make sure 4 motor commands and two grabber commands are recieved
      if (manualComs.size() == 6) {
        //Serial.println("Set Commands");
        //set max yaw command to 120 deg/s
        

        yawCom = -manualComs[0]*120;

        if (USE_EST_VELOCITY_IN_MANUAL == true){
          //set max velocities 2 m/s
          upCom = manualComs[1]*2.0;
          forwardCom = manualComs[3]*2.0;
          translationCom = manualComs[2]*2.0;
        }else{
          //normal mapping using max esc command 
          upCom = manualComs[1]*2.0; //PID used and maxed out at 2m/s
          forwardCom = manualComs[3]*500.0;
          translationCom = manualComs[2]*500.0;
        }
      
      } else {
        Serial.println(manualComs.size());
        //data is not valid, set motors to zero
        yawCom = 0;
        forwardCom = 0;
        upCom = 0;
        translationCom = 0;
      }

      //Serial.print(">up:");
      //Serial.println(upCom);


        //check if shooting should be engaged
        //this block switches the state to the oposite that it is currently in
          if (shoot != manualComs[5]) {
          shoot = manualComs[5];
          //change shoot state
          if (ballGrabber.state == 2) {
            //stop shooting
            ballGrabber.closeGrabber();
          } else {
            //start shooting
            ballGrabber.shoot();

            //reset catch counter
            catches=0;
          }

        //check if grabbing should be engaged
        //this block switches the state to the oposite that it is currently in
        } else if (grab != manualComs[4]) {
          grab = manualComs[4];

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
        */
      
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

    //  safty height 
    if (actualBaro > MAX_HEIGHT) {
        upCom = -0.5;
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

    upMotor = verticalPID.calculate(upCom, kf.v, dt); //up velocity from barometer

    if (USE_EST_VELOCITY_IN_MANUAL == true){
    //using kalman filters for the current velosity feedback for full-state feeback PID controllers
    //forwardMotor = forwardPID.calculate(forwardCom, xekf.v, dt);  //extended filter
    //float forwardMotor = forwardPID.calculate(forwardCom, kal_vel.x_vel_est, dt);
    //translationMotor = translationPID.calculate(translationCom, yekf.v, dt); //extended filter
    //float translationMotor = translationPID.calculate(translationCom, kal_vel.y_vel_est, dt); 
    }else{
      //without PID
      forwardMotor = forwardCom;
      translationMotor = translationCom;
    }

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
        bool leftReady = leftGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, motorControl.yawLeft, motorControl.upLeft, motorControl.forwardLeft);
        bool rightReady = rightGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, motorControl.yawRight, motorControl.upRight, motorControl.forwardRight);
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
