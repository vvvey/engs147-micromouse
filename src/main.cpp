#include "Motors.h"
#include "IR.h"
#include "IMU.h"
#include "Encoder.h"
#include "ArduinoMotorShieldR3.h"
#include "ForwardControl.h"

#define TS 10
unsigned long prev_time_milli = 0;
unsigned long start_time_milli = 0;
unsigned long curr_time_milli = 0;

ForwardControl forward;

void setup() {
    Serial.begin(115200);
    IMU_init();
    IR_init();
    rightEnc.begin();
    leftEnc.begin();
    motor_driver.init();

    Serial.println("Hello World");

    forward.init(30.0); // 30 rad/s
    delay(100);
}



void loop() {
    unsigned long curr_time_milli = millis();
    long start_time_milli = millis();

    while (curr_time_milli - start_time_milli < 10000) {
        // Wait for 10 seconds
        curr_time_milli = millis();
        if (curr_time_milli - prev_time_milli >= TS) {
            forward.update();
            prev_time_milli = curr_time_milli;
        }
    }

    motor_driver.setSpeeds(0.0,0.0);
    exit(0);
    
}

