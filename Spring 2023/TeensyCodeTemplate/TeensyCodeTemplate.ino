//import libraries

//set up enums for state machine
/*
enum Mode {
manual,
auto,
lost,
}

enum AutoStates {
searching,
approach,
...
}
 */

//set up global varibles (state machine states)

//define motors and servos object

//define PID controllers

//define Filters

void setup() {
  //Set up Serial ports

  //Begin communication with pi or any other wifi module
  
}

void loop() {
  if (/*regulate time interval that this occurs*/) {
    //get updated sensor data
    //imu, ultrasonics, camera, etc...
    //Note: not all sensors update at the same frequency,
    //outer loop mush update faster than the fastes sensor,
    //all sensor update loops should be inividually regulated
    //in a synchronized manner for accurate control
    
    //update grabber state
  
    //perform decisions with state machine
  
    //init zero control commands
    double yaw = 0;
    double z = 0;
    double x = 0;
    /*
    //undo block comment for switch statement
    switch (state variable) {
      case state1:
      case state2:
      ...
      default:
      //default is called if state is invalid
      //(no defined parameters for the state)
    }
    */
  
    //update actuators using motor object
    //(call some method or a motor object here)
  }
}
