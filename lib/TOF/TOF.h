#ifndef TOF_H
#define TOF_H

#include <Arduino.h>

void TOF_init();
float TOF_getDistance(uint8_t sensor_id);  // returns mm, or negative error code

#endif