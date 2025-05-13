#include "Motors.h"
#include "IR.h"
#include "ArduinoMotorShieldR3.h"
#include "ForwardControl.h"

#define TS 10
unsigned long prev_time_ms = 0;
unsigned long start_time_ms = 0;
ForwardControl forward;

void setup() {
    Serial.begin(115200);
    forward.init(30.0); // 30 rad/s
    IR_init();
}



void loop() {
    unsigned long curr_time_ms = millis();
    long start_time_ms = millis();

    while (curr_time_ms - start_time_ms < 10000 && IR_getDistance(A9) > 12 && forward.isFinished() == false) {
        // Wait for 10 seconds
        curr_time_ms = millis();
        if (curr_time_ms - prev_time_ms >= TS) {
            forward.update();
            prev_time_ms = curr_time_ms;
        }
    }

    motor_driver.setSpeeds(0.0,0.0);
    exit(0);
    
}

