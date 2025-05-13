#ifndef VL6180_IR_H
#define VL6180_IR_H

#include <Arduino.h>

void TOF_init();
float TOF_getDistance(uint8_t sensor_id);  // returns mm, or negative error code

#endif
