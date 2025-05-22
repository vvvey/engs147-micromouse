#include "Motors.h"
#include "IR.h"
#include "IMU.h"
#include "Encoder.h"
#include "ArduinoMotorShieldR3.h"
#include "MotionController.h"

#define RIGHT_IR A10
#define LEFT_IR A8
#define FRONT_IR A9

MotionController motion;

void setup() {
    Serial.begin(115200);
    IMU_init();
    IR_init();
    rightEnc.begin();
    leftEnc.begin();
    motor_driver.init();

    Serial.println("Hello World");

    delay(100);
}


int state = 0;

void loop() {
    motion.update();

    if (!motion.isBusy()) {
        if (state == 0) {
            motion.fwd_2_dis(9.0, 50.0);
            // motion.rotate(90); // turn right
            state++;
        } else if (state == 1) {
            motion.rotate(-90); // turn left
            state++;
        } else if (state == 2) {
            motion.fwd_2_dis(8.0, 30.0);
            state++;
        } else if (state == 3) {
            motion.rotate(90); // turn right
            state++;
        } else if (state == 4) {
            motion.fwd_2_dis(8.0, 40.0);
            state++;
        } else {
            stop_motors();
            exit(0);
        }
    }
}