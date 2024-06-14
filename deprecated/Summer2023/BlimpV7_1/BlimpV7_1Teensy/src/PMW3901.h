//Libraries
#include "Arduino.h"
#include <stdint.h>
#include <vector>

#pragma once

class PMW3901
{
private:
    uint8_t _cs;

    uint8_t registerRead(uint8_t reg);
    void registerWrite(uint8_t reg, uint8_t value);
    void initRegisters(void);

public:
    PMW3901(uint8_t cspin);
    bool begin(void);

    void readMotionCount(int16_t *deltaX, int16_t *deltaY, float Z_distance_m, float pitch_rate, float roll_rate);
    float x_motion_comp;
    float y_motion_comp;
};
