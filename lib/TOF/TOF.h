#ifndef TOF_H
#define TOF_H

#include <Arduino.h>

#define FRONT_LEFT 2
#define FRONT_RIGHT 1
#define RIGHT 0
#define LEFT 3

void TOF_init();
float TOF_getDistance(uint8_t sensor_id);  // returns mm, or negative error code

#endif