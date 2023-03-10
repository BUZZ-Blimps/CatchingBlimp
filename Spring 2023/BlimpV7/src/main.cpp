#include <Arduino.h>
#include <vector>
#include <SerialData.h>
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

//motor timeout before entering lost state
#define TEENSY_WAIT_TIME          0.5

//init pins

// #define LYSPIN                     7
// #define LPSPIN                     6
// #define LMPIN                     2
// #define RYSPIN                     10
// #define RPSPIN                     9
// #define RMPIN                     5

#define GRABBER_PIN               8
#define SHOOTER_PIN               4

//Print info (for sending variable data to the rasberry pi)
#define MAX_NUM_VARIABLES         4
#define MAX_VARIABLE_NAME_SIZE    6

//objects
SerialData piData;

//sensor fusion objects
BerryIMU_v3 BerryIMU;
Madgwick_Filter madgwick;
BaroAccKF kf;
AccelGCorrection accelGCorrection;
Optical_Flow Flow;
Kalman_Filter_Tran_Vel_Est kal_vel;
OpticalEKF xekf(DIST_CONSTANT, GYRO_X_CONSTANT, GYRO_YAW_CONSTANT);
OpticalEKF yekf(DIST_CONSTANT, GYRO_Y_CONSTANT, 0);
GyroEKF gyroEKF;

//Gimbal leftGimbal(yawPin, pitchPin, motorPin, newDeadband, newTurnOnCom, newMinCom, newMaxCom);
MotorControl motorControl;
Gimbal leftGimbal(7, 6, 2, 25, 30, 1000, 2000, 45, 0.2);
Gimbal rightGimbal(10, 9, 5, 25, 30, 1000, 2000, 135, 0.2);

//Manual PID control
PID verticalPID(500, 0, 0);  //can be tuned down 
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

//msg variables
String msgTemp;
double lastMsgTime = -1.0;
bool outMsgRequest = false;

double handShakeTimeOut = 750; //ms


String outMsg = "hello world";

double count = 0;
int mult = 4;

std::vector<uint8_t> ofBuffer;
double startTime = 0;

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

//telemetry data to pi
std::vector<String> piVariables;
std::vector<float> piValues;

//function prototypes
std::vector<String> piMessage();
bool piListen();
bool processSerial(String msg);
void Pulse();
void print(String key, float value);

void setup() {
  //start serial connection
  Serial1.begin(115200);
  Serial.begin(115200);

  bool start = true;
  int msgCount = 0;
  while (!piListen) {
    Serial.println("Waiting");
    delay(100);
  }

  firstMessageTime = micros()/MICROS_TO_SEC;

  Serial.println("Starting Program");
}

void loop() {
  //check for a message from the pi, done as fast as possible
  piListen();

  //read buffer for optical flow
  Flow.read_buffer();

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

    //get orientation from madgwick
    pitch = madgwick.pitch_final;
    roll = madgwick.roll_final;
    yaw = madgwick.yaw_final;

    //compute the acceleration in the barometers vertical reference frame
    accelGCorrection.updateData(BerryIMU.AccXraw, BerryIMU.AccYraw, BerryIMU.AccZraw, pitch, roll);

    //run the prediction step of the vertical velecity kalman filter
    kf.predict(dt);
    xekf.predict(dt);
    yekf.predict(dt);
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
    

    xekf.updateAccelx(-accelGCorrection.agx);
    xekf.updateGyroX(gyroEKF.pitchRate - gyroEKF.pitchRateB);
    xekf.updateGyroZ(BerryIMU.gyr_rateZraw);

    yekf.updateAccelx(-accelGCorrection.agy);
    yekf.updateGyroX(gyroEKF.rollRate - gyroEKF.rollRateB);
    yekf.updateGyroZ(BerryIMU.gyr_rateZraw);

    
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
    
    

    //print("Yrate", yawRateFilter.last);

    //print("zVel", kf.v);

    
    kal_vel.predict_vel();
    kal_vel.update_vel_acc(-accelGCorrection.agx/9.81, -accelGCorrection.agy/9.81);
  }


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

    xekf.updateBaro(CEIL_HEIGHT_FROM_START-actualBaro);
    yekf.updateBaro(CEIL_HEIGHT_FROM_START-actualBaro);
  }

  
  //Optical flow update
  dt = micros()/MICROS_TO_SEC-lastOpticalLoopTick;
  if (dt > 1.0/OPTICAL_LOOP_FREQ) {
    lastOpticalLoopTick = micros()/MICROS_TO_SEC;
  
    Flow.update_flow(BerryIMU.gyr_rateXraw, BerryIMU.gyr_rateYraw, 1);

    //change to blimp coordinates
    float x_opt = (float)Flow.y_motion/dt;
    float y_opt = (float)Flow.x_motion/dt;

    xekf.updateOptical(x_opt);
    yekf.updateOptical(y_opt);

    // Serial.print(">X Velocity:");
    // Serial.println(Flow.x_motion_comp);

    // accelGCorrection.updateData(BerryIMU.AccXraw, BerryIMU.AccYraw, BerryIMU.AccZraw, pitch, roll);

    // kal_vel.update_vel_optical(Flow.x_motion_comp, Flow.y_motion_comp);

    // Serial.print(">X Velocity est:");
    // Serial.println(kal_vel.x_vel_est);

  }
  

  //compute slower sensors (if any)
  //use same if statement structure


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
      //Serial.println("Manual");
      //get manual data
      std::vector<double> manualComs = piData.getManualComs();
      
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
      
    } else if (state == autonomous){
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
    }else {
        //not a valid state
        forwardCom = 0.0;
        upCom = 0.0;
        yawCom = 0.0;
    }

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
    forwardMotor = forwardPID.calculate(forwardCom, xekf.v, dt);  //extended filter
    //float forwardMotor = forwardPID.calculate(forwardCom, kal_vel.x_vel_est, dt);
    translationMotor = translationPID.calculate(translationCom, yekf.v, dt); //extended filter
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
        
        //Serial.println("Controlable");
        //if (ZERO_MODE) motorControl.update(10, 0, 0, 0, 0);
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

bool piListen() {

  bool retVal = false;

  //check for debug mode
  if (!ZERO_MODE) {
    if (Serial1.available() > 0) {
      char c = Serial1.read();
      //Serial.print(c);
      if (c == '#') {
        //process message
        Serial.println(msgTemp);
        retVal = processSerial(msgTemp);

        //update last message time
        lastMsgTime = micros()/MICROS_TO_SEC;

        //send back state of blimp
        String variablesToPi = "";
        for (int i = 0; i < piVariables.size(); i++) {
          variablesToPi = variablesToPi + piVariables[i] + ":";
          variablesToPi = variablesToPi + String(piValues[i]) + ":";
        }

        Serial1.println(String(mode)+":"+String(blimpColor)+":"+String(goalColor)+ ":" + variablesToPi+"#");
        //Serial.println(String(mode)+":"+String(blimpColor)+":"+String(goalColor)+ ":" + variablesToPi+"#");

        //clear message
        msgTemp = "";
      } else {
        msgTemp = msgTemp + String(c);
      }
    }

    //if a message has not been recieved after 0.25s change to lost state (turn off motors)
    if (micros()/MICROS_TO_SEC - lastMsgTime > TEENSY_WAIT_TIME) {
      state = lost;
    }
  }

  return retVal;
}

bool processSerial(String msg) {
  //set up output vectors
  std::vector<double> mData;
  std::vector<String> splitData;
  std::vector<String> object;
  std::vector<double> objectFinal;
  std::vector<std::vector<double> > balloons;
  std::vector<std::vector<double> > dGoals;
  std::vector<std::vector<double> > aGoals;
  std::vector<std::vector<double> > eBlimps;
  std::vector<std::vector<double> > fBlimps;

  //clear old target
  piData.target.clear();

  //clear manual data
  mData.clear();

  //Serial.println(msg);
  msg.trim();

  if (msg.length() > 0 && msg[0] == 'L') {
    state = lost;
    return false;
  }

  //split string by objects
  //decompose data
  if (msg.length() > 1) {
    int first = 0;
    int index = 1;
    while (index < msg.length()) {
      if (msg[index] == '&') {
        splitData.push_back((msg.substring(first, index)).trim());
        first = index+1;
        index = first+1;
      } else {
        index += 1;
      }
    }
  } else {
    //invalid message
    return false;
  }
  
  if (splitData.size() > 2 && splitData[0].equals("M")) {
    //add motor commands to mData array
    state = manual;
    if (splitData.size() == 9) {
      for (unsigned int i = 3; i < splitData.size(); i++) {
        mData.push_back((double)splitData[i].toFloat());
      }
    }

    quad = (int)splitData[1].toFloat();

    if (splitData[2].toFloat() < -1000) {
      Serial.println("Baro Data Not Current");
    } else {
      baseBaro = splitData[2].toFloat();
      //Serial.println("Baro Data is Current");
    }

    //Serial.print("Manual size: ");
    //Serial.println(mData.size());
    piData.update(mData);
    return true;

  } else if (splitData.size() > 2 && splitData[0].equals("A")) {
    state = autonomous;

    quad = (int)splitData[1].toFloat();
    
    
    if (splitData[2].toFloat() < -1000) {
      Serial.println("Baro Data Not Current");
    } else {
      baseBaro = splitData[2].toFloat();
      //Serial.println("Baro Data is Current");
    }
    
    for (unsigned int i = 3; i < 4 && splitData.size() >= 4; i++) {
        Serial.println("Target Data");
        Serial.println(splitData[i]);
        String msg = splitData[i];
        int index2 = 0;
        int first2 = 0;
        while (index2 < msg.length()) {
          if (msg[index2] == ':') {
            object.push_back((msg.substring(first2, index2)).trim());
            first2 = index2+1;
            index2 = first2+1;
          }
          index2 += 1;
        }

        for (unsigned int j = 0; j < object.size(); j++) {
            if (j == 0) {
              objectFinal.push_back((double)(object[j].toFloat()-320.0/2.0));
            } else if (j == 1) {
              objectFinal.push_back((double)(object[j].toFloat()-240.0/2.0));
            } else {
              objectFinal.push_back((double)(object[j].toFloat()));
            }
         }
         //clear old target
         piData.target.clear();
         
         //add updated target
         if (objectFinal.size() == 4) {
          piData.target.push_back(objectFinal);
         }

         //clear old variables
         object.clear();
         objectFinal.clear();
     } 
  } else {
    return false;
  }
  return true;
}

void print(String key, float value) {
  if (key.length() > MAX_VARIABLE_NAME_SIZE) {
    Serial.print("Variable name is to long: ");
    Serial.println(key);
    return;
  }

  bool keyWasFound = false;
  for (int i = 0; i < piVariables.size(); i++) {
    if (piVariables[i].equals(key)) {
      //update variable in values
      piValues[i] = value;
      keyWasFound = true;
    }
  }

  if (!keyWasFound) {
    if (piVariables.size() < MAX_NUM_VARIABLES) {
      piVariables.push_back(key);
      piValues.push_back(value);
    } else {
      Serial.println("To Many Variables to print to pi");
    }
  }
}

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
