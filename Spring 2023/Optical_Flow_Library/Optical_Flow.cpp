/*
 Optical_Flow.cpp - Library that will output the flow rate and surface quality content 
*/
#define HWSERIAL Serial4
#include "Arduino.h"
#include "Optical_Flow.h"
#define FOOTER 0xAA
#define HEADER 0xFE
#include <iterator>

Optical_Flow::Optical_Flow(){
}

void Optical_Flow::read_buffer(){
  while(HWSERIAL.available()) {
    uint8_t c = HWSERIAL.read();
    buffer.push_back(c);
  }
}

void Optical_Flow::update_flow(){
  bool foundPacket = false;
  if(!buffer.empty()){
    std::vector<uint8_t>::iterator it = buffer.end();
    while(it!=buffer.begin()){
      if(*it==FOOTER){
        //Serial.println("found footer");
        int distToHead = std::distance(buffer.begin(),it);
        //Serial.println(distToHead);
        if (distToHead >= 8){
          std::vector<uint8_t>::iterator headerCheck = it-8;
//          while(headerCheck != it) {
//            Serial.println(*headerCheck, HEX);
//            headerCheck++;
//          }
//          Serial.println(*it, HEX);
//          foundPacket = true;
          if (*(it-8) == HEADER){
            //Serial.println("Found header");
            
            if(*(headerCheck+1)==4){
              //Serial.println("Found Packet");
              foundPacket = true;
              uint8_t x_motion_HB = *(headerCheck+2);
              uint8_t x_motion_LB = *(headerCheck+3);
              uint8_t y_motion_HB = *(headerCheck+4);
              uint8_t y_motion_LB = *(headerCheck+5);
              uint8_t checksum = *(headerCheck+6);
              Serial.print(x_motion_HB);
              Serial.print("\t");
              Serial.print(x_motion_LB);
              Serial.print("\t");
              surface_quality = *(headerCheck+7);
              x_motion = (((uint16_t)x_motion_HB << 8) | x_motion_LB);
              y_motion = (((uint16_t)y_motion_HB << 8) | y_motion_LB);
              Serial.print(x_motion);
              Serial.print("\t");
              Serial.print(y_motion);
              Serial.print("\t");
              Serial.println(surface_quality);
              break;
            }
          }
        }
      }
      it--;
    }
    if(foundPacket || buffer.size() >= 100){
      buffer.clear();
    }
  }

}
