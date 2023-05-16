/*
  Optical_Flow.cpp - Library that will output the flow rate and surface quality content
*/
#define HWSERIAL Serial3
//https://www.pjrc.com/teensy/td_uart.html
//TX pin 14
//RX pin 15
#include "Arduino.h"
#include "Optical_Flow.h"
#define FOOTER 0xAA
#define HEADER 0xFE
#include <iterator>

Optical_Flow::Optical_Flow() {
  HWSERIAL.begin(19200);
  while (!HWSERIAL) {
    delay(100);
  }
}

void Optical_Flow::read_buffer() {
  while (HWSERIAL.available()) {
    uint8_t c = HWSERIAL.read();
    //Serial.println(c);
    buffer.push_back(c);
  }
}

void Optical_Flow::update_flow(float roll_rate, float pitch_rate, float Z_distance_m) {  
  bool foundPacket = false;
  if (!buffer.empty()) {
    std::vector<uint8_t>::iterator it = buffer.end();
    while (it != buffer.begin()) {
      if (*it == FOOTER) {
        //Serial.println("found footer");
        int distToHead = std::distance(buffer.begin(), it);
        //Serial.println(distToHead);
        if (distToHead >= 8) {
          std::vector<uint8_t>::iterator headerCheck = it - 8;
          //          while(headerCheck != it) {
          //Serial.println(*headerCheck, HEX);
          //            headerCheck++;
          //          }
          //          Serial.println(*it, HEX);
          //          foundPacket = true;
          if (*(it - 8) == HEADER) {
            //Serial.println(*(it-8));
            //Serial.println("Found header");

            if (*(headerCheck + 1) == 4) {
              //Serial.println("Found Packet");
              foundPacket = true;

              //Serial.print(*(headerCheck),HEX);
              //Serial.print(",");
              //Serial.print(*(headerCheck+1));
              //Serial.print(",");
              //Serial.print(*(headerCheck+2));
              //Serial.print(",");
              //Serial.print(*(headerCheck+3));
              //Serial.print(",");
              //Serial.print(*(headerCheck+4));
              //Serial.print(",");
              //Serial.print(*(headerCheck+5));
              //Serial.print(",");
              //Serial.print(*(headerCheck+6));
              //Serial.print(",");
              //Serial.println(*(headerCheck+7));
              //Serial.print(",");
              //Serial.println(*(headerCheck+8),HEX);

              uint8_t x_motion_HB = *(headerCheck + 2);
              uint8_t x_motion_LB = *(headerCheck + 3);
              uint8_t y_motion_HB = *(headerCheck + 4);
              uint8_t y_motion_LB = *(headerCheck + 5);
              uint8_t checksum = *(headerCheck + 6);
              surface_quality = *(headerCheck + 7);

              x_motion = (((uint16_t)x_motion_LB << 8) | x_motion_HB);
              y_motion = (((uint16_t)y_motion_LB << 8) | y_motion_HB);

              
              //Serial.print(x_motion);
              //Serial.print(",");
              //Serial.println(y_motion);
              //Serial.print(",");
              //Serial.println(surface_quality);

              
              //Optical Flow Sensor Specs
              float FOV = 42;
              float sensor_res = 1225;
              float deg_rad = (FOV/2)*0.0174533;

              //First Scalar
              float scalar_alt = .005;

              //Altitude Compenstation
              float x_motion_scaled = (((x_motion * Z_distance_m) / (sensor_res * scalar_alt)) * 2 * tan(deg_rad));
              float y_motion_scaled = ((y_motion * Z_distance_m) / (sensor_res * scalar_alt)) * 2 * tan(deg_rad);
              //Serial.print(x_motion_scaled);
              //Serial.print(",");
              //Serial.println(y_motion_scaled);

              //Serial.print(">X motion scaled:");
              //Serial.println(x_motion_scaled);

              //Roll and Pitch Compensation
              //Would subtract this value from the scaled motion to cancel out motion by the roll and pitch

              //Second Scalar
              float scalar_comp = .00035;
              
              float x_motion_change = (pitch_rate*sensor_res*scalar_comp)/FOV;
              float y_motion_change = (roll_rate*sensor_res*scalar_comp)/FOV;
              //Serial.print(x_motion_change);
              //Serial.print(",");
              //Serial.println(y_motion_change);

              //Serial.print(">change:");
              //Serial.println(x_motion_change);


              //Final adjustment
              x_motion_comp = x_motion_scaled-x_motion_change;
              y_motion_comp = y_motion_scaled-y_motion_change;
              //Serial.println(x_motion_comp);
              //Serial.print(",");
              //Serial.println(y_motion_comp);
              
              break;
            
            }
          }
        }
      }
      it--;
    }
    if (foundPacket || buffer.size() >= 100) {
      buffer.clear();
    }
  }

}
