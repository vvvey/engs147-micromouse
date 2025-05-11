#ifndef IR_H
#define IR_H

#include <Arduino.h>
#include <math.h> 

#define LEFT_IR A8
#define FRONT_IR A9
#define RIGHT_IR A10

void IR_init(void);
float IR_getDistance(int id);

#endif

