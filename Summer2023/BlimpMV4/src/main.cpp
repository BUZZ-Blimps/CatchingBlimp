#include <tgmath.h>

#include "BerryIMU_v3.h"
#include "EMAFilter.h"
#include "PID.h"
#include "BlimpClock.h"
#include "Servo.h"
#include "MotorMapping.h"
#include "accelGCorrection.h"
#include "BangBang.h"
#include "baro_acc_kf.h"
#include "gyro_ekf.h"
#include "UDPComm.h"
#include "Madgwick_Filter.h"



//this is the 3 feedbacks
#define FEEDBACK_BUF_SIZE 3

#define MOTORS_OFF false //used for debugging

#define OUTERLOOP 10  //Hz

#define HWSERIAL Serial2

#define IDNAME(name) #name

#define DIST_CONSTANT             0.002

#define GYRO_X_CONSTANT           480.0
#define GYRO_YAW_CONSTANT         0

#define GYRO_Y_CONSTANT           323.45

//Define ceiling height from where we plug in battery in meters
#define CEIL_HEIGHT_FROM_START    4 

//sensor and controller rates
#define FAST_SENSOR_LOOP_FREQ           100.0
#define BARO_LOOP_FREQ                  50.0

//constants
#define MICROS_TO_SEC             1000000.0

enum states {
  searching,
  approach,
}state;

const char* stateNames[] = {IDNAME(searching), IDNAME(approach)};
const char* stateStr[] = {"searching", "approach"};

enum autonomousStates {
  autonomous,
  manual,
  lost,
}autonomousState;

const char* autonomousStatesNames[] = {IDNAME(autonomous), IDNAME(manual), IDNAME(lost)};
const char* autonomousStatesStr[] = {"autonomous", "manual", "lost"};

int targetColor = 2; 
//r 0, g 1, b 2

//motor pins
const int LMPIN = 9; // 26 is pin for Left motor object
const int RMPIN =  6; // 27 is pin for Right motor object
const int LSPIN = 2; //14 is pin for Left servo object
const int RSPIN = 4; //12 is pin for Right servo object 

// Pinout

//msg variables
String msgTemp;
double lastMsgTime = -1.0;
bool outMsgRequest = false;

//servo objects/motor objects

//motor->(pin,deadband,turn on,min,max)
MotorMapping motors(LSPIN, RSPIN, LMPIN, RMPIN, 5, 50, 1000, 2000,0.3);

//objects
BerryIMU_v3 BerryIMU;
Madgwick_Filter madgwick;
BaroAccKF kf;
AccelGCorrection accelGCorrection;

// Kalman_Filter_Tran_Vel_Est kal_vel;
// OpticalEKF xekf(DIST_CONSTANT, GYRO_X_CONSTANT, GYRO_YAW_CONSTANT);
// OpticalEKF yekf(DIST_CONSTANT, GYRO_Y_CONSTANT, 0);

GyroEKF gyroEKF;

EMAFilter yawRateFilter(0.2);
EMAFilter pitchRateFilter(0.1);

EMAFilter pitchAngleFilter(0.2);
EMAFilter rollAngleFilter(0.2);

//pre process for accel before vertical kalman filter
EMAFilter verticalAccelFilter(0.05);

//baro offset computation from base station value
EMAFilter baroOffset(0.5);
//roll offset computation from imu
EMAFilter rollOffset(0.5);

PID yawRatePID(3,0,0);  
PID pitchRatePID(2.4,0,0);


//adjust  these for Openmv dont change the middle zeros
PID xPos(0.25,0,0);

PID yPos(0.4,0,0);

//WIFI objects
BlimpClock udpClock;
BlimpClock heartbeat;
BlimpClock motorClock;
UDPComm udp;

//variables
double feedbackData[FEEDBACK_BUF_SIZE];
bool autoTransition = false;
bool motorsOff = false; //used for safegaurd

double forwardInput = 0;
double yawInput = 0;
double upInput = 0;

double lastOuterLoopTime = 0;

double ceilHeight = 500;

//IMU orentation
float rotation = -90;

//timing global variables for each update loop
float lastSensorFastLoopTick = 0.0;
float lastBaroLoopTick = 0.0;

//base station baro
float baseBaro = 0.0;

//corrected baro
float actualBaro = 0.0;

//sensor data
float pitch = 0;
float yaw = 0;
float roll = 0;

String s = "";


std::vector<std::vector<double>> detections;
void processSerial(String msg);


void setup() {
  
  // put your setup code here, to run once:
  Serial.begin(115200);

  //UART Comm (OpenMV)
  HWSERIAL.begin(115200);
  Serial.println("Color tracking program started");
  delay(2000);
    //initializations
    udp.init();
    udpClock.setFrequency(100);
    heartbeat.setFrequency(20);

    //initialize the feedback data as 0s
    for (int i=0; i < FEEDBACK_BUF_SIZE; i++) {
        feedbackData[i] = 0;
    }

    motorClock.setFrequency(400);    
  
  //wait 2 seconds
  delay(2000);
}

void loop() {
  //reading Serial2 color coordinates (OpenMV) and pass them to PID
  /*
  if (HWSERIAL.available()) {
    char c = HWSERIAL.read();
    Serial.print(c);
    if (c !='!'){
      s += c; 
    } else {
      //Serial.println(s);
      //parse string s
      processSerial(s);
      //clear string s
      s = "";
    }
  }
  */

  // Funky serial reading protocol, this is neccessary due to some weird bugs with serial1 and serial2 clashing
  String incomingString = "";  // initialize an empty string to hold the incoming data
  char startDelimiter = '@';   // set the start delimiter to '@'
  char endDelimiter = '!';     // set the end delimiter to '!'

  // Clear Serial Buffers
  Serial2.read();
  Serial1.read();

  while (Serial2.available()) {
    if (Serial2.find(startDelimiter)) {
      s = Serial2.readStringUntil(endDelimiter);
      Serial.println(s);
      Serial.println(s.length());

      // Ensure no invalid characters or overlapping packets
      if (s.indexOf('@') == -1 && s.indexOf('!') == -1) {
        // Ensure the packet is of the right size
        if (s.length() == 96){
          processSerial(s);
        }
      }

      // Reset s
      s = "";
    }
  }

  //reading data from base station
  if (udpClock.isReady() && udp.readPackets()) {

    // Retrieve inputs from packet
    std::vector<String> inputs = udp.packetMoveGetInput();


    //Check UDP packet is legit
    if (udp.packetMove.indexOf('@') == -1 && udp.packetMove.indexOf('!') == -1 && udp.packetMove != "") {
      
      // ************************** IMU LOOP ************************** //
      float dt = micros()/MICROS_TO_SEC-lastSensorFastLoopTick;
      if (dt >= 1.0/FAST_SENSOR_LOOP_FREQ) {
        lastSensorFastLoopTick = micros()/MICROS_TO_SEC;

        //read sensor values and update madgwick
        BerryIMU.IMU_read();
        BerryIMU.IMU_ROTATION(rotation);

        madgwick.Madgwick_Update(BerryIMU.gyr_rateXraw,
                                BerryIMU.gyr_rateYraw,
                                BerryIMU.gyr_rateZraw,
                                BerryIMU.AccXraw,
                                BerryIMU.AccYraw,
                                BerryIMU.AccZraw);

        //get orientation from madgwick
        pitch = madgwick.pitch_final;
        //Serial.println("PITCH:");
        //Serial.println (pitch);
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

        //Serial.println(BerryIMU.AccXraw);
        
      } 

      // ************************** BARO LOOP ************************** //
      dt = micros()/MICROS_TO_SEC-lastBaroLoopTick;
      if (dt >= 1.0/BARO_LOOP_FREQ) {
        lastBaroLoopTick = micros()/MICROS_TO_SEC;

        //get most current imu values
        BerryIMU.IMU_read();
        BerryIMU.IMU_ROTATION(rotation);
        
        //update kalman with uncorreced barometer data
        kf.updateBaro(BerryIMU.alt);


        //compute the corrected height with base station baro data and offset
        actualBaro = BerryIMU.alt - baseBaro + baroOffset.last;

        // xekf.updateBaro(CEIL_HEIGHT_FROM_START-actualBaro);
        // yekf.updateBaro(CEIL_HEIGHT_FROM_START-actualBaro);
      }

      // ******************* PACKET RELATED LOGIC ******************* //

      /*
      //DEBUG SEE INPUTS
      Serial.println("Inputs: ");
      for (int i = 0; i < inputs.size(); i++)
      {
        Serial.print(inputs[i]);
        Serial.print(" ");
      }
      Serial.print("\n");

      Serial.println(atof(inputs[1].c_str()));
      */

      // State Logic
      String t = inputs[0];
      // Serial.println("State " + t);
      String targetEnemy = "";

      if (t == "A") {
        autonomousState = autonomous;
        targetEnemy = inputs[1];
      } else if (t == "M"){
        autonomousState = manual;
        targetEnemy = inputs[5];
      } else {
        autonomousState = lost;
      }

      // Serial.println(autonomousState);

      // Target Enemy Logic
      if(targetEnemy == "R"){
        targetColor = 0;
      }else if(targetEnemy == "G"){
        targetColor = 1;
      }else if(targetEnemy == "B"){
        targetColor = 2;
      }

      // Serial.print("Target Color: ");
      // Serial.println(targetColor);

      // ******************* STATE MACHINE ******************* //
      // Manual
      if (autonomousState == manual){

        if (motorClock.isReady()) {
          //State Machine
          //MANUAL
          //Default

          //Serial.println(udp.packetMoveGetInput()[2].c_str());
          
          //stod() was toDouble()
          forwardInput = atof(inputs[4].c_str());
          yawInput = atof(inputs[1].c_str());
          upInput = atof(inputs[2].c_str());

          //safegaurd: if motor reads any command that is greater than 1, shut the motor off!!!
          if (abs(forwardInput) >1.0 || abs(yawInput)>1.0 || abs(upInput)>1.0){
            motorsOff = true;
            Serial.println("Invalid motor input!!!!!");
          }
            
          //Serial.println(forwardInput);
          //Serial.println(yawInput);
          //Serial.println(upInput);
           
          //map controller input to yaw rate
          //Serial.println(yawInput);
          upInput = 500*upInput;
          forwardInput = 500*forwardInput;
          yawInput = -yawInput*120;    //120 degrees per second

          //Serial.println(yawInput);
        }

      // Autonomous
      } else if (autonomousState == autonomous) {

        //AUTONOMOUS
        //Serial.println("auto");
        double outerLoopTime = millis() - lastOuterLoopTime;

        if (outerLoopTime > (1.0/OUTERLOOP)*1000) {
          lastOuterLoopTime = millis();

          //ultrasonic
          // Serial.print("Ceiling Height: ");
          // Serial.println(ceilHeight);
        

          //perform decisions
          switch (state) {

            //Search
            case searching:
              if (true) {
                
                yawInput = -20;   //turning rate while searching


                //check if the height of the blimp is within this range (ft), adjust accordingly to fall in the zone 
                if (ceilHeight < 110) {
                  //Serial.println("up");
                  upInput = 100*cos(pitch*3.1415/180.0); //or just 100 (without pitch control)
                  forwardInput = 100*sin(pitch*3.1415/180.0);
                  
                } else if (ceilHeight > 200) {
                  //Serial.println("down");
                  upInput = -100*cos(pitch*3.1415/180.0);
                  forwardInput = -100*sin(pitch*3.1415/180.0);
                } else {
                  upInput = 0;
                  forwardInput = 0;
                }
                
                //we see something o_O
                if (detections.size() == 3 && detections[targetColor].size() == 2 && detections[targetColor][0] < 500) {
                  state = approach;
                }
                
              }

            break;

            // Approach
            case approach:
              if (detections.size() == 3 && detections[targetColor].size() == 2 && detections[targetColor][0] < 500) {
                  yawInput = xPos.calculate(-detections[targetColor][0], 0, 100);
                  upInput = yPos.calculate(-detections[targetColor][1], 0, 100);
                  forwardInput = 350; //approaching thrust (300 as default)
              } else {
                state = searching;
              }

            break;

            // Default Case
            default:
            Serial.println("Invalid State");
            break;
          }
        }
      }
      
      // ******************* MOTOR INPUTS ******************* //
      double yawPIDInput = 0.0;
      double deadband = 2.0; //To do

      Serial.println(state);
      
      yawPIDInput = yawRatePID.calculate(yawInput, yawRateFilter.last, 100);   
      if (abs(yawInput-yawRateFilter.last) < deadband) {
          yawPIDInput = 0;
      } else {
          yawPIDInput = tanh(yawPIDInput)*abs(yawPIDInput);
      }
      
      // If lost, give zero command
      if (autonomousState == lost) {
        motors.update(0,0,0,0);
      }

      //turing the motors off for debugging for second case
      if (autonomousState == lost)
      {
        motors.update(0,0,0,0);
      }
      else if (MOTORS_OFF == false && motorsOff == false) {
        //Serial.println("after");
        //Serial.println(yawInput);
        //Serial.print(",");
        //Serial.print(upInput);
        //Serial.print(",");
        //Serial.println(forwardInput);

        motors.update(0, forwardInput, upInput, yawPIDInput);
      } else {
        motors.update(0,0,0,0);
      }

    // End Message Check
    }

    // End UDP Clock
  }

  //Clear Packet
  udp.packetClear();

  // End Main Loop
}

// process Serial message from the camera
void processSerial(String msg) {
    std::vector<double> green;
    std::vector<double> blue;
    std::vector<double> red;

    std::vector<String> splitData;
    std::vector<String> object;
    //Serial.print("Message");
    //split data
    if (msg.length() > 2) {
    int first = 0;
    int index = 1;
    while (index < msg.length()) {
      if (msg[index] == ';') {
        String k = msg.substring(first, index);
        k.trim();
        splitData.push_back(k);
        first = index+1;
        index = first+1;
      }
      index += 1;
    }
  } else {
    //invalid message
    Serial.println("Invalid Message From Open MV");
    return;
  }

  if (splitData.size() != 4) {
    return;
  }

  for (int i = 0; i < 3; i++) {
    //Serial.println(splitData[i]);
    
    if (splitData[i].length() > 2) {
      int first = 0;
      int index = 1;
      while (index < splitData[i].length()) {
        if (splitData[i][index] == ',') {
          String k = splitData[i].substring(first, index);
          k.trim();
          object.push_back(k);
          first = index+1;
          index = first+1;
        }
        index += 1;
      }
    } else {
      //invalid message
      Serial.println("Invalid Message From Open MV");
      return;
    }

    //get x,y coordinates
    double x = object[0].toFloat()-320/2;
    double y = -(object[1].toFloat()-240/2);

    if (object.size() == 3) {
      switch (object[2][0]) {
        case 'r':
        red.push_back(x);
        red.push_back(y);
        break;
        case 'g':
        green.push_back(x);
        green.push_back(y);
        break;
        case 'b':
        blue.push_back(x);
        blue.push_back(y);
        break;
        default:
        //do noting, invalid color option
        Serial.println("Invalid Color Option");
        break;
      }
    }

    object.clear();
  }

  ceilHeight = (double)splitData[3].toFloat();
  //Serial.println(ceilHeight);

  detections.clear();
  detections.push_back(red);
  detections.push_back(green);
  detections.push_back(blue);
}

