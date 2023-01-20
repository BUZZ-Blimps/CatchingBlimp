#include "MotorControl.h"

MotorControl thrust(2,5,60,1000,2000);

void setup() {
  // put your setup code here, to run once:
  delay(10000);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (millis() < 12000) {
    thrust.motorCom(1000);
  } else {
    thrust.motorCom(0);
  }
}
