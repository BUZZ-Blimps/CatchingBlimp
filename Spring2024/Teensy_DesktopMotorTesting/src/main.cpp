#include "Arduino.h"
#include "Servo.h"

#define L_Pitch                   2                    
#define L_Yaw                     3              
#define R_Pitch                   4                
#define R_Yaw                     5 

#define PWM_R                     6              
#define PWM_G                     9              
#define PWM_L                     14

Servo motorLPitch;
Servo motorRPitch;
Servo motorL;
Servo motorR;
Servo motorG;

const int servoForward = 2000;
const int servoBackward = 1000;
const int servoCenter = 1500;

const int motorForward = 1700;
const int motorBackward = 1300;
const int motorCenter = 1500;

void test_simple();
void test_fancy();

// ws = write servo
// motorStr: "L"->left, "R"->right, "B"->both
// up_down: 1->up, 0->center, -1->down
void ws(String motorStr, float up_down){
  // left
  if(motorStr == "L" || motorStr == "B"){
    int leftUp = 700;
    int leftDown = 2300;
    motorLPitch.writeMicroseconds((up_down+1)/2.0 * (leftUp-leftDown) + leftDown);
  }
  // right
  if(motorStr == "R" || motorStr == "B"){
    int rightUp = 2300;
    int rightDown = 700;
    motorRPitch.writeMicroseconds((up_down+1)/2.0 * (rightUp-rightDown) + rightDown);
  }
}

void ws_sweep(String motorStr, float up_down_from, float up_down_to, int delayms){
  for(int i=0; i<delayms; i++){
    float up_down = (i/(float)delayms)*(up_down_to - up_down_from) + up_down_from;
    ws(motorStr, up_down);
    delay(1);
  }
}

// wm = write motor
// motorStr: "L"->left, "C"->center, "R"->right, "A"->all
void wm(String motorStr, float forward_back){
  if(forward_back < -1) forward_back = -1;
  if(forward_back > 1) forward_back = 1;

  // left
  if(motorStr == "L" || motorStr == "A"){
    int forward = 2000;
    int backward = 1000;
    motorL.writeMicroseconds((forward_back+1)/2.0 * (forward-backward) + backward);
  }
  // right
  if(motorStr == "R" || motorStr == "A"){
    int forward = 2000;
    int backward = 1000;
    motorR.writeMicroseconds((forward_back+1)/2.0 * (forward-backward) + backward);
  }
  // center
  if(motorStr == "C" || motorStr == "A"){
    int forward = 2000;
    int backward = 1000;
    motorG.writeMicroseconds((forward_back+1)/2.0 * (forward-backward) + backward);
  }
}

void wm_sweep(String motorStr, float forward_back_from, float forward_back_to, int delayms){
  for(int i=0; i<delayms; i++){
    float forward_back = (i/(float)delayms)*(forward_back_to - forward_back_from) + forward_back_from;
    wm(motorStr, forward_back);
    delay(1);
  }
}

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  // delay(10000);
  motorLPitch.attach(L_Pitch);
  motorRPitch.attach(R_Pitch);
  motorL.attach(PWM_L);
  motorR.attach(PWM_R);
  motorG.attach(PWM_G);
  
  motorLPitch.write(1500);
  motorRPitch.write(1500);
  motorL.write(1500);
  motorR.write(1500);
  motorG.write(1500);

  Serial.println("Set to 0. Waiting 5 seconds... ");
  delay(5000);
  Serial.println("Done!");
  delay(1000);
}

void loop(){
  test_fancy();
}

void test_simple(){
  // Start, center
  ws("B",0);
  Serial.print("Center. Starting in 5 seconds");
  for(int i=0; i<5; i++){
    delay(1000);
    Serial.print(".");
  }
  Serial.println();

  // Fast sweep
  Serial.println("Fast sweep.");
  ws_sweep("B", 0, 1, 500);
  ws_sweep("B", 1, -1, 1000);
  ws_sweep("B", -1, 0, 500);
  delay(1000);

  // Sweep BL motors
  float str = 0.3;
  float lowStr = 0.1;
  Serial.println("Sweep BL motors forward.");
  wm_sweep("A", lowStr, str, 2000);
  wm_sweep("A", str, lowStr, 2000);
  wm("A", 0);
  delay(500);
  Serial.println("Sweep BL motors backward.");
  wm_sweep("A", -lowStr, -str, 2000);
  wm_sweep("A", -str, -lowStr, 2000);
  wm("A", 0);
  delay(1000);
}

void test_fancy() {
  // Start, center
  ws("B",0);
  Serial.print("Center. Starting in 5 seconds");
  for(int i=0; i<5; i++){
    delay(1000);
    Serial.print(".");
  }
  Serial.println();

  // Slow sweep
  Serial.println("Slow sweep.");
  ws_sweep("B", 0, 1, 2000);
  ws_sweep("B", 1, -1, 4000);
  ws_sweep("B", -1, 0, 2000);
  delay(1000);

  // Medium sweep
  Serial.println("Medium sweep.");
  ws_sweep("B", 0, 1, 1000);
  ws_sweep("B", 1, -1, 2000);
  ws_sweep("B", -1, 0, 1000);
  delay(1000);

  // Fast sweep
  Serial.println("Fast sweep.");
  ws_sweep("B", 0, 1, 500);
  ws_sweep("B", 1, -1, 1000);
  ws_sweep("B", -1, 0, 500);
  delay(1000);

  // Fast sweep individual
  Serial.println("Fast sweep left.");
  ws_sweep("L", 0, 1, 500);
  ws_sweep("L", 1, -1, 1000);
  ws_sweep("L", -1, 0, 500);
  Serial.println("Fast sweep right.");
  ws_sweep("R", 0, 1, 500);
  ws_sweep("R", 1, -1, 1000);
  ws_sweep("R", -1, 0, 500);
  delay(1000);

  // Pulse BL motors
  float str = 0.3;
  Serial.println("Pulse BL motors forward.");
  wm("L", str);
  delay(1000);
  wm("L", 0);
  wm("C", str);
  delay(1000);
  wm("C", 0);
  wm("R", str);
  delay(1000);
  wm("R", 0);
  delay(1000);

  // Sweep BL motors
  float lowStr = 0.1;
  Serial.println("Sweep BL motors forward.");
  wm_sweep("A", lowStr, str, 2000);
  wm_sweep("A", str, lowStr, 2000);
  wm("A", 0);
  delay(500);
  Serial.println("Sweep BL motors backward.");
  wm_sweep("A", -lowStr, -str, 2000);
  wm_sweep("A", -str, -lowStr, 2000);
  wm("A", 0);
  delay(1000);

  // Combo alternate sweep
  Serial.println("Combo alternate sweep.");
  ws("L", 1);
  ws("R", -1);
  delay(500);
  for(int i=0; i<1000; i++){
    ws("L",  1-i/1000.0);
    ws("R", -1+i/1000.0);
    wm("L", i/1000.0 * (str-lowStr) + lowStr);
    wm("R", i/1000.0 * (str-lowStr) + lowStr);
    delay(1);
  }
  for(int i=0; i<1000; i++){
    ws("L", -i/1000.0);
    ws("R",  i/1000.0);
    wm("L",  i/1000.0 * (lowStr-str) + str);
    wm("R",  i/1000.0 * (lowStr-str) + str);
    delay(1);
  }
  wm("A", 0);
  delay(500);
  ws("B", 0);
  delay(1000);
}