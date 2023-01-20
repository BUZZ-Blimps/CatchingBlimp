 //import libraries
#include "Servo.h"
#include "MotorControl.h"
#include "EMAFilter.h"
#include "IMU_Control.h"
#include "PID.h"
#include "vector"
#include "SerialData.h"
#include "String.h"
#include "tripleBallGrabber.h"

//rates
#define OUTERLOOP 50  //Hz
#define OFSERIAL Serial4

//-----States for blimp and grabber-----
enum autoState {
  searching,
  yAlign,
  approach,
  catching,
  caught,
  goalSearch,
  approachGoal,
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

//globals for states
int state = manual;
int mode = searching;

//serial object
SerialData piData;


//set up global varibles (state machine states)
double mainLoopRefresh = 100; //ms or 10Hz

double lastOuterLoopTime = 0;

//init pins
const int LSPin = 6;
const int RSPin = 7;
const int LMPin = 2;
const int RMPin = 5;

const int grabberPin = 8;

//define imu object
IMU_Control imu;

//define motors and servos object
MotorControl motors(LSPin, RSPin, LMPin, RMPin, 5, 50, 1000, 2000);

//ball Grabber startup here
TripleBallGrabber ballGrabber(8,4);
int shoot=0;
int grab=0;

//define PID controllers
PID yawRatePID(4,0,0);
PID heightRatePID(2.4,0,0);

PID yPixelPID(0.3,0,0);
PID xPixelPID(0.2,0,0);

//define Filters
EMAFilter pitchFilter(0.6);
EMAFilter yawRateFilter(0.2);
EMAFilter heightFilter(0.07);

EMAFilter xPixFilter(0.5);
EMAFilter yPixFilter(0.5);
EMAFilter zPixFilter(0.3);

EMAFilter xgPixFilter(0.5);
EMAFilter ygPixFilter(0.5);
EMAFilter agPixFilter(0.1);

//Serial messaging data handling
String msgTemp;
double lastMsgTime = 0;
bool outMsgRequest = false;

double handShakeTimeOut = 750; //ms


String outMsg = "hello world";

double zPixV = 0;

double count = 0;
int mult = 4;

std::vector<uint8_t> ofBuffer;
double startTime = 0;

double catchTimeStart = 0;
double catchTime = 1700;        //ms

double caughtTimeStart = 0;
double caughtTime = 6500;       //ms

double scoreTimeStart = 0;
double scoreTime = 1200;        //ms

double shootingTimeStart = 0;
double shootingTime = 1800;        //ms

double scoredTimeStart = 0;
double scoredTime = 2500;        //ms


int catches = 0;

void setup() {
  //Set up Serial ports
  OFSERIAL.begin(19200);
  Serial1.begin(115200);
  
  while (!OFSERIAL) {
    delay(100);
  }
  
  
  //Begin communication with pi or any other wifi module

  //init motors and servos
  delay(10000);

    //start handshake process
  bool tryHandShake = true;
  
  
  while (tryHandShake) {
    if (millis() % 1000 == 0) {
      //send data to pi
      Serial1.println("?");
      Serial.println("Waiting for Pi");
    }
    char c = 'a';
    
    while (Serial1.available() > 0) {
      //read a single character
      c = Serial1.read();
      if (c == '#') {
        //end handshake
        tryHandShake = false;
        break;
      }
    }
  }

  Serial.println("hand shake complete");

  //updates timeout counter
  lastMsgTime = millis();
  
  startTime = millis();
}



//constructs messages to send to the pi during message transmission
std::vector<String> piMessage() {
  
  String messageStr = "";

  if (state == manual) {
    //if manual, send manual mode and height from pin sensor
    messageStr += "Manual Mode\n";
    
  } else if (state == autonomous) {
    //if autonomous, confirm state and also send the current mode 
    messageStr += "Autonomous Mode: ";
    switch (mode) {
      case searching:
      messageStr += "Searching\n";
      break;
      case approach:
      messageStr += "Approach\n";
      break;
      case catching:
      messageStr += "Catching\n";
      break;
      case caught:
      messageStr += "Caught\n";
      break;
      case goalSearch:
      messageStr += "Goal Search\n";
      break;
      case approachGoal:
      messageStr += "Approach Goal\n";
      break;
      case scoringStart:
      messageStr += "Start Scoring\n";
      break;
      case shooting:
      messageStr += "Shooting\n";
      break;
      case scored:
      messageStr += "Scored\n";
      break;
    }
  } else {
    messageStr += "Lost\n";
  }

  //split message into 8 byte packets
  std::vector<String> message;
  
  for (unsigned int i = 0; i < messageStr.length(); i = i + 8) {

    int endChar = i+8;

    //check to see if the length is exceeded
    if (messageStr.length() < endChar) {
      endChar = messageStr.length();
    }
    
    message.push_back(messageStr.substring(i,endChar));
  }
  
  /*
   * debugging
  for (unsigned int i = 0; i < message.size(); i++ ) {
    Serial.println(message[i]);
  }
  */
  return message;
}

void loop() {
  //control directions
  double yaw = 0;     //yaw rate (deg/s)
  double up = 0;      //motor command
  double forward = 0; //motor command
  
  //read in data from pi
  bool serialStart = true;
  if (Serial1.available() <= 0) {
    serialStart = false;
  }

  //create message to send to pi
  std::vector<String> sendMessage;
  sendMessage = piMessage();
  
  int index = 0;
  
  while (serialStart) {

    //check if serial is empty
    while (Serial1.available() <= 0) {
      //wait
      if (millis() - lastMsgTime > handShakeTimeOut) {
        //turn off motors, messages are not being recieved
        break;
      }
    }
    
    while (true) {
      char c = 0;
      
      while (Serial1.available() > 0) {
        //read charaters
        c = Serial1.read();
        if (c != '@' && c != '#') {
          //if not start or end character, add character to string
          msgTemp = msgTemp+String(c);
        }
      }
      
      //if end, restart message recieving
      if (c == '#') {
        serialStart = false;
        lastMsgTime = millis();
        break;
      }

      //next message needs to be sent
      if (c == '@' || c == '*') {
        lastMsgTime = millis();
        break;
      }

      //timeout for no messages after a period of time
      if (millis() - lastMsgTime > handShakeTimeOut) {
        break;
      }
    }

    

    if (!serialStart) {
      processSerial(msgTemp);
      //Serial.println(msgTemp);
      msgTemp = "";
      break;
    }
    
    

    //send response
    if (index < sendMessage.size()) {
      Serial1.print(sendMessage[index]+"@");
      //Serial.print("Response sent: ");
      //Serial.println(sendMessage[index]+"@");
      index += 1;
    } else {
      Serial1.print("*");
      //Serial.print("Response sent: ");
      //Serial.println("*");
    }
    
    
    if (millis() - lastMsgTime > handShakeTimeOut) {
      break;
    }
    
  }
  
  
  //check if a new handshake needs to be completed
  if (millis() - lastMsgTime > handShakeTimeOut) {
    //redo handshake
    Serial1.clear();
    bool tryHandShake = true;
  
    while (tryHandShake) {

      //shut off motors
      motors.update(0,0,0,0);

      //look for handshake from pi
      if (millis() % 1000 == 0) {
        Serial1.println("?");
        Serial.println("Waiting for Pi");
      }
      char c = 'a';
      while (Serial1.available() > 0) {
        c = Serial1.read();
        if (c == '#') {
          tryHandShake = false;
          lastMsgTime = millis();
          break;
        }
      }
    }
    Serial1.clear();
  }
  

  

  //read data from pi as fast as possible
  double outerLoopTime = millis() - lastOuterLoopTime;
  if (outerLoopTime > (1.0/OUTERLOOP)*1000) {
    count += mult;
    //debug update rate functionality for outer loop
    lastOuterLoopTime = millis();
    //Serial.println(count);

    //get imu data
    imu.update();
    double pitch = 0; //pitchFilter.filter(imu.getPitch());
    double yawRate = yawRateFilter.filter(imu.getYawRate());

    //read height sensor
    double ping = analogRead(A6);
    //Serial.print(ping);
    //Serial.print("\t");
    ping = heightFilter.filter(ping)/1024.0*5*(512/5)/12;
    //Serial.println(ping);

    
    //calculate command for testing
    /*
    double forward = 80*sin((count/2)*3.1415/180);
    double yaw = 50*sin((count/3)*3.1415/180);
    double up = 250*sin((count/8)*3.1415/180);
    */
    
    /*
    if ((int)count % 5 == 0) {
      Serial.print(forward);
      Serial.print(",");
      Serial.print(yaw);
      Serial.print(",");
      Serial.println(up);
    */

    //Serial.println(state);

    //maual vs auto
    if (state == manual) {
      //get manual data
      std::vector<double> manualComs = piData.getManualComs();
  
      //Serial.println(manualComs.size());
  
      if (manualComs.size() == 6) {
        //Serial.println("Manual");
        
        yaw = -manualComs[0]*120;    //gives the desired rotarion rate
        up = manualComs[1]*500;
        forward = manualComs[3]*500;

        if (shoot != manualComs[5]) {
          shoot = manualComs[5];
          //change shoot state
          if (ballGrabber.state == 2) {
            ballGrabber.closeGrabber();
          } else {
            ballGrabber.shoot();
            catches=0;
          }

          
        } else if (grab != manualComs[4]) {
          grab = manualComs[4];
          //change grabber state

          if (ballGrabber.state == 0) {
            ballGrabber.openGrabber();
          } else {
            ballGrabber.closeGrabber();
            catches++;
          }
          
        }
        /*
        Serial.print(yaw);
        Serial.print("\t");
        Serial.print(up);
        Serial.print("\t");
        Serial.println(forward);
        */
        
      } else {
        //data is not valid, set motors to zero
        yaw = 0;
        forward = 0;
        up = 0;
      }
      
    } else if (state == autonomous) {
      //print balloon data if a balloon is present
      Serial.println("get balloon data");
      std::vector<std::vector<double> > balloons = piData.getBalloonData();
      Serial.println("balloon data retrieved");

      std::vector<std::vector<double> > goalsA = piData.getAGoalData();
      
      Serial.println("Autonomous");
      
      if (balloons.size() > 0) {
        for (int i = 0; i < balloons.size(); i++) {
          if (balloons[i].size() == 4) {
            Serial.print("Balloon-> x: ");
            Serial.print(balloons[i][1]);
            Serial.print("\ty: ");
            Serial.print(balloons[i][0]);
            Serial.print("\tz: ");
            Serial.print(balloons[i][2]);
            Serial.print("\tArea: ");
            Serial.println(balloons[i][3]);
          }
        }
      } else {
        Serial.println("No Balloon Found");
      }
      
      
      double area = 0;
      int bIndex = 0;
      double bx = 0;
      double by = 0;
      double bz = 0;
      if (balloons.size() > 0) {
        for (int i = 0; i < balloons.size(); i++) {
          if (balloons[i].size() == 4) {
            if (area < balloons[i][3]) {
              area = balloons[i][3];
              bIndex = i;
            }
          }
        }

        bx=xPixFilter.filter(-balloons[bIndex][1]);
        by=yPixFilter.filter(balloons[bIndex][0]);
        if (abs(balloons[bIndex][2]) > 0) {
          bz=zPixFilter.filter(abs(balloons[bIndex][2]));
        } else {
          bz=zPixFilter.filter(100000);
        }
      } else {
        bx=xPixFilter.filter(0);
        by=yPixFilter.filter(0);
        bz=zPixFilter.filter(100000);
      }

      if (balloons.size() > 0 && balloons[bIndex].size() == 4) {
        Serial.println(bz);
      }

      double areaG = 0;
      int gIndex = 0;
      double gx = 0;
      double gy = 0;
      double gA = 0;
      if (goalsA.size() > 0) {
        for (int i = 0; i < balloons.size(); i++) {
          if (goalsA[i].size() == 4) {
            if (areaG < goalsA[i][3]) {
              areaG = goalsA[i][3];
              gIndex = i;
            }
          }
        }

        gx=xgPixFilter.filter(-goalsA[gIndex][1]);
        gy=ygPixFilter.filter(goalsA[gIndex][0]);
        gA=agPixFilter.filter(goalsA[gIndex][3]);
      } else {
        gx=xgPixFilter.filter(0);
        gy=ygPixFilter.filter(0);
        gA=agPixFilter.filter(0);
      }

      Serial.println(bz);

      switch (mode) {
        case searching:
        if (true) {
          yaw = -10;
          up = 0;
          forward = 0;

          //check for a balloon in frame
          if (balloons.size() > 0) {
            mode = yAlign;
          }
        }
        break;
        case yAlign:
        if (balloons.size() > 0) {
          yaw = xPixelPID.calculate(bx, 0, outerLoopTime/1000);
          up = yPixelPID.calculate(by, -10, outerLoopTime/1000);
          forward = 5;

          if (abs(by)-10 < 100) {
            mode = approach;
          }
        } else {
          mode = searching;
        }
        break;
        case approach:
        if (balloons.size() > 0) {

          yaw = xPixelPID.calculate(bx, 0, outerLoopTime/1000);
          up = yPixelPID.calculate(by, -10, outerLoopTime/1000);
          forward = 30;
          
          if (bz < 80) {
            ballGrabber.openGrabber();
            if (bz < 60) {
              mode = catching;
            }
          } else {
            ballGrabber.closeGrabber();
          }
        } else {
          mode = searching;
        }
        break;
        case catching:
        if (true) {
          forward = 40;
          up = 120;
          yaw = 0;

          //start catching timer
          if (catchTimeStart < millis() - catchTime-1000) {
            catchTimeStart = millis();
          }
  
          if (catchTimeStart < millis() - catchTime) {
            mode = caught;
            ballGrabber.closeGrabber();
            catches++;
          }
        }
        
        break;
        case caught:
        if (catches > 0) {

          if (caughtTimeStart < millis() - caughtTime-1000) {
            caughtTimeStart = millis();
          }
  
          if (caughtTimeStart < millis() - caughtTime) {
            if (catches > 0) {
              mode = goalSearch;
            } else {
              mode = searching;
            }
          }
        
          forward = -150;
          up = 0;
          yaw = 0;
          
        } else {
          mode = searching;
        }
        break;
        case goalSearch:
        if (catches > 0) {
          yaw = -20;
          up = 0;
          forward = 0;
          
          if (goalsA.size() > 0) {
            mode = approachGoal;
          }
        } else {
          mode = searching;
        }
        break;
        case approachGoal:
        if (goalsA.size() > 0 && catches > 0) {
          yaw = xPixelPID.calculate(gx, 0, outerLoopTime/1000);
          up = yPixelPID.calculate(gy, -20, outerLoopTime/1000);
          forward = 30;

          if (gA > 35000) {
            mode = scoringStart;
          }
        } else if (catches > 0) {
          mode = goalSearch;
        } else {
          mode = searching;
        }
        break;
        case scoringStart:
        if (true) {
          yaw = 0;
          forward = 30;
          up = 20;

          if (scoreTimeStart < millis() - scoreTime-1000) {
            scoreTimeStart = millis();
          }
  
          if (scoreTimeStart < millis() - scoreTime) {
            mode = shooting;
            break;
          }

          
        }
        break;
        case shooting:
        if (true) {
          yaw = 0;
          forward = 100;
          up = 75;

          ballGrabber.shoot();
          catches = 0;

          if (shootingTimeStart < millis() - shootingTime-1000) {
            shootingTimeStart = millis();
          }
  
          if (shootingTimeStart < millis() - shootingTime) {
            ballGrabber.closeGrabber();
            mode = scored;
            break;
          }
        }
        break;
        case scored:
        if (true) {

          ballGrabber.closeGrabber();

          yaw = 0;
          forward = -40;
          up = -80;
          
          if (scoredTimeStart < millis() - scoredTime-1000) {
            scoredTimeStart = millis();
          }
  
          if (scoredTimeStart < millis() - scoredTime) {
            mode = searching;
            break;
          }
        }
        default:
          mode=searching;
          break;
      }
    } else {
      //lost
      yaw = 0;
      up = 0;
      forward = 0;
    }

    //PID controllers
    yaw = yawRatePID.calculate(yaw,yawRate, outerLoopTime/1000);
    //up = heightRatePID.calculate(9,ping, outerLoopTime/1000);
    
    //update all motors
   
    motors.update(pitch, forward, up, yaw);
    //motors.update(pitch, 0, 0, 0);
  }
}


void processSerial(String msg) {
  //set up output vectors
  std::vector<std::vector<std::vector<double> > > aData;
  std::vector<double> mData;
  std::vector<String> splitData;
  std::vector<String> object;
  std::vector<double> objectFinal;
  std::vector<std::vector<double> > balloons;
  std::vector<std::vector<double> > dGoals;
  std::vector<std::vector<double> > aGoals;
  std::vector<std::vector<double> > eBlimps;
  std::vector<std::vector<double> > fBlimps;

  //clear manual data
  mData.clear();

  //Serial.println(msg);
  msg.trim();

  if (msg.length() > 0 && msg[0] == 'L') {
    state = lost;
    return;
  }

  //split string by objects
  //decompose data
  if (msg.length() > 2) {
    int first = 0;
    int index = 1;
    while (index < msg.length()) {
      if (msg[index] == '&') {
        splitData.push_back((msg.substring(first, index)).trim());
        first = index+1;
        index = first+1;
      }
      index += 1;
    }
  } else {
    //invalid message
    return;
  }
  
  if (splitData[0].equals("M")) {
    //add motor commands to mData array
    state = manual;
    if (splitData.size() == 8) {
      //optical flow can be throw nan or large numbers, use this to threshold
      if (!splitData[1].equals("nan")) {
        if (splitData[1].toFloat() < 10 || splitData[1].toFloat() > -10) {
           zPixV = splitData[1].toFloat();
        }
      }
      for (unsigned int i = 2; i < splitData.size(); i++) {
        mData.push_back((double)splitData[i].toFloat());
      }
    }
  } else if (splitData[0].equals("A")) {
    state = autonomous;
    //optical flow can be throw nan or large numbers, use this to threshold
    if (!splitData[1].equals("nan")) {
      if (splitData[1].toFloat() < 10 || splitData[1].toFloat() > -10) {
         zPixV = splitData[1].toFloat();
      }
    }
    //split objects up and add them to correct array
    //Serial.println("Objects"); 
    
    //for (unsigned int i = 0; i < splitData.size(); i++) {
    //  Serial.println(splitData[i]);
    //}
    

    
    for (unsigned int i = 2; i < splitData.size(); i++) {
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

        for (unsigned int j = 1; j < object.size(); j++) {
            if (j == 1) {
              objectFinal.push_back((double)(object[j].toFloat()-320.0/2.0));
            } else if (j == 2) {
              objectFinal.push_back((double)(object[j].toFloat()-240.0/2.0));
            } else {
              objectFinal.push_back((double)(object[j].toFloat()));
            }
         }
         
         //what object type is it?
         if (object[0].equals("B")) {
            balloons.push_back(objectFinal);
         } else if (object[0].equals("GA")) {
            aGoals.push_back(objectFinal);
         } else if (object[0].equals("GD")) {
            dGoals.push_back(objectFinal);
         } else if (object[0].equals("BF")) {
            fBlimps.push_back(objectFinal);
         } else if (object[0].equals("BE")) {
            eBlimps.push_back(objectFinal);
         }
          
         object.clear();
         objectFinal.clear();
     } 
  } else {
    return;
  }

  //clear old data
  aData.clear();

  //push found data to data array

  
  aData.push_back(balloons);
  aData.push_back(dGoals);
  aData.push_back(aGoals);
  aData.push_back(eBlimps);
  aData.push_back(fBlimps);

  piData.update(mData, aData);
}
