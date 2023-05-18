/*
 Optical_Flow.h - Library that will output the flow rate and surface quality content 
*/
#pragma once

#define HWSERIAL Serial3
#include "Arduino.h"
#include <vector>

class Optical_Flow
{
  public:
    Optical_Flow();
    int16_t x_motion;
    int16_t y_motion;
    float x_motion_comp;
    float y_motion_comp;
    int surface_quality;
    void update_flow(float roll_rate, float pitch_rate, float Z_distance_m);
    void read_buffer();
    

  private:
    int8_t idx;
    std::vector<uint8_t> buffer;
  
};
