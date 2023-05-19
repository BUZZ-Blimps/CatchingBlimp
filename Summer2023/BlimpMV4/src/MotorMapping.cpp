#include "MotorMapping.h"

MotorMapping::MotorMapping(int LSPin, int RSPin, int LMPin, int RMPin, double newdeadband, double newturnOnCom, double newminCom, double newmaxCom, double servoFilter) {
    //set servo pins
    this->LServo.attach(LSPin);
    this->RServo.attach(RSPin);
    this->LMotor.attach(LMPin);
    this->RMotor.attach(RMPin);

    //set initial values
    this->LServo.write(135);
    this->RServo.write(45);
    this->LMotor.write(1500);
    this->RMotor.write(1500);

    this->servoLFilter.setAlpha(servoFilter);
    this->servoRFilter.setAlpha(servoFilter);

    this->deadband = newdeadband;
    this->turnOnCom = newturnOnCom;
    this->minCom = newminCom;
    this->maxCom = newmaxCom;
}

void MotorMapping::update(double pitch, double forward, double up, double yaw) {
  //yaw positive right, negative left for positive yaw
  //calcs are in motor command domain that is shifted by -1500 so that zero throttle is the origin
  //yaw, up, and forward are bounded by -500 to 500;


  //bound combined motor commands
  if (up > 500) {
    up = 500;
  } else if (up < -500) {
    up = -500;
  }
  
  if (yaw > 500) {
    yaw = 500;
  } else if (yaw < -500) {
    yaw = -500;
  }

  if (forward > 500) {
    forward = 500;
  } else if (forward < -500) {
    forward = -500;
  }

  //maintain yaw after sum
  if (forward + yaw > 500) {
    forward = - yaw+500;
  }

  if (forward - yaw > 500) {
    forward = yaw+500;
  }

  if (forward - yaw < -500) {
    forward = yaw - 500;
  }

  if (forward + yaw < -500) {
    forward = -yaw - 500;
  }
  
  double brx = forward + yaw;
  double brz = up;

  double blx = forward - yaw;
  double blz = up;

  /*
  Serial.print(brx);
  Serial.print(" :brx \t");
  Serial.print(blx);
  Serial.print(" :blx\t");

  Serial.print(brz);
  Serial.print(" :brz \t");
  Serial.print(blz);
  Serial.println(" :blz\n");
  
  Serial.print(forward);
  Serial.print(" :f\t");
  Serial.print(yaw);
  Serial.print(" :y\t");
  Serial.print(up);
  Serial.println(" :u \n");
  */

  
  //right motor calculation
  double thetaR = atan2(brz, brx);

  //left motor calculation
  double thetaL = atan2(blz, blx);

  //mag calculation
  double magR = sqrt(pow(brx,2)+pow(brz,2))*sqrt(2.0)/2.0;
  double magL = sqrt(pow(blx,2)+pow(blz,2))*sqrt(2.0)/2.0;

  //quadrant validation
  double sR = sin(thetaR)*magR;
  double cR = cos(thetaR)*magR;

  double sL = sin(thetaL)*magL;
  double cL = cos(thetaL)*magL;

  /*
  Serial.println("quadrant validation");
  Serial.print(brx*sqrt(2.0)/2.0);
  Serial.print("\t, -> brx = ");
  Serial.println(cR);

  Serial.print(brz*sqrt(2.0)/2.0);
  Serial.print("\t, -> brz = ");
  Serial.println(sR);

  Serial.print(blx*sqrt(2.0)/2.0);
  Serial.print("\t, -> blx = ");
  Serial.println(cL);

  Serial.print(blz*sqrt(2.0)/2.0);
  Serial.print("\t, -> blz = ");
  Serial.println(sL);

  Serial.println();
  */
  //conversion to degrees for servo
  thetaR = thetaR*180/3.1415 - pitch;
  thetaL = thetaL*180/3.1415 - pitch;
  /*
  Serial.println("Coordinate Orientation");
  Serial.print("ThetaR: ");
  Serial.print(thetaR);
  Serial.print("\tThetaL: ");
  Serial.println(thetaL);
  Serial.println();
  */
  //switch to servo angles
  thetaR = 45 + thetaR;
  thetaL = 135 - thetaL;

  //make sure the angles are within sero motion limits

  if (thetaR > 180) {
    thetaR -= 180;
    magR = -magR;
  }

  if (thetaR < 0) {
    thetaR += 180;
    magR = -magR;
  }

  if (thetaL > 180) {
    thetaL -= 180;
    magL = -magL;
  }

  if (thetaL < 0) {
    thetaL += 180;
    magL = -magL;
  }

  
  //comute correct motor direction
  /*
  Serial.println("Servo Orientation");
  Serial.print("ThetaR: ");
  Serial.print(thetaR);
  Serial.print("\tThetaL: ");
  Serial.println(thetaL);
  Serial.println();
  Serial.print("MagR: ");
  Serial.print(magR);
  Serial.print("\tMagL: ");
  Serial.print(magL);
  Serial.println("\n");
  */
  //shift motor mag to pulse width in microseconds
  magR = magR * 2;
  magL = magL * 2;

    // char msg[128];
    // snprintf(msg, 128, "magL=%0.2f, magR=%0.2f",magL, magR);
    // Serial.println(msg);

  RServo.write(servoRFilter.filter(thetaR));
  LServo.write(servoLFilter.filter(thetaL));

  delay(50);
  
  RMotor.write(this->motorCom(magR));
  LMotor.write(this->motorCom(magL));
}

void MotorMapping::writeLServo(double angle) {
  this->LServo.write(angle);
}

void MotorMapping::writeRServo(double angle) {
  this->RServo.write(angle);
}

double MotorMapping::motorCom(double command) {
    //input from -1000, to 1000 is expected from controllers
    double adjustedCom = 1500;
    
    if (abs(command) <= deadband/2) {
        adjustedCom = 1500;
    } else if (command > deadband/2) {
        double xo1 = deadband/2;
        double yo1 = turnOnCom+1500;
        double m1 = (maxCom-yo1)/(1000-xo1);
        adjustedCom = m1*command-m1*xo1+yo1;
    } else if (command < deadband/2) {
        double xo2 = -deadband/2;
        double yo2 = -turnOnCom+1500;
        double m2 = (yo2-minCom)/(xo2-(-1000));
        adjustedCom = m2*command-m2*xo2+yo2;
    } else {
        //should never happend, but write 1500 anyway for safety
        adjustedCom = 1500;
    }
    
    return adjustedCom;
}
