#include "Motors.h"
#include "IR.h"

#define TS 10
unsigned long prev_time_ms = 0;
unsigned long start_time_ms = 0;
Motors motors;

void setup() {
    motors.begin();
    Serial.begin(115200);
    motors.forward_straight_begin(30.0);
    IR_init();
}



void loop() {
    unsigned long curr_time_ms = millis();
    long start_time_ms = millis();

    while (curr_time_ms - start_time_ms < 10000 && IR_getDistance(A9) > 12) {
        // Wait for 10 seconds
        curr_time_ms = millis();
        if (curr_time_ms - prev_time_ms >= TS) {
            motors.forward_straight_controller();
            prev_time_ms = curr_time_ms;
        }
    }

    motors.stop();
    exit(0);
    
}

