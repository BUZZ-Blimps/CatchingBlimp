#define HWSERIAL Serial4
//https://www.pjrc.com/teensy/td_uart.html
//TX pin 17
//RX pin 16
#include "Optical_Flow.h"

Optical_Flow Flow;

unsigned long now = 0;
unsigned long flowTime = 0;
unsigned long printTime = 0;
int16_t x_motion = 0; 
int16_t y_motion = 0;
int surface_quality = 0;

void setup() {
   // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }
  HWSERIAL.begin(19200);
  while (!HWSERIAL) {
    delay(100);
  }
  flowTime = micros();
}

void loop() {
  Flow.read_buffer();

  now = micros();
  if (now - flowTime >= 100000) {
    Flow.update_flow();
    flowTime = now;
    x_motion = Flow.x_motion;
    y_motion = Flow.y_motion;
    surface_quality = Flow.surface_quality;
    /*
    Serial.print(x_motion*1000);
    Serial.print(",");
    Serial.println(y_motion*1000);
    //Serial.print(",");
    //Serial.println(surface_quality);  
    */
  }
  //if (now - printTime >= ){
    
  //}
}
