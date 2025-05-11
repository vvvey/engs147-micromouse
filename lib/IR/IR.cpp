#include "IR.h"

void IR_init(void) {
    analogReadResolution(12);
}

static float v2d_4_30(float v) { // Bigger IR sensor
    // Fitted Curve: y = 51.4366 * exp(-3.4633 * x) + 18.7568 * exp(-0.4963 * x)
    // 2^12 bits of resolution
    float voltage = v * (3.3 / 4095.0);
    float distance = 51.4366 * exp(-3.4633 * voltage) + 18.7568 * exp(-0.4963 * voltage);
    return distance;
}

static float v2d_2_15(float v) { // Different voltage-->distance conversion for smaller IR sensor
    // Fitted Curve: y = 37.9240 * exp(-4.7013 * x) + 12.8494 * exp(-0.8068 * x)
    float voltage = v * (3.3 / 4095.0);
    float distance = 37.924 * exp(-4.7013 * voltage) + 12.8494 * exp(-0.8068 * voltage);
    return distance;
}

float IR_getDistance(int id) { 
    int ir_analog_value = analogRead(id);
    float ir_distance;
    if (id == FRONT_IR){
        ir_distance = v2d_4_30((float)ir_analog_value);
    }
    else {
        ir_distance = v2d_2_15((float)ir_analog_value);
    }    
    return ir_distance;
}