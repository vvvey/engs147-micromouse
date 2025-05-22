#include "Motors.h"
#include "IR.h"
#include "IMU.h"
#include "Encoder.h"
#include "ArduinoMotorShieldR3.h"
#include "MotionController.h"

#define RIGHT_IR A10
#define LEFT_IR A8
#define FRONT_IR A9

#define NORTH 0
#define EAST 90
#define SOUTH 180
#define WEST 270

MotionController motion;

void setup() {
    Serial.begin(115200);
    IMU_init();
    IR_init();
    rightEnc.begin();
    leftEnc.begin();
    motor_driver.init();

    Serial.println("Hello World");

    delay(1000);
}


int state = 0;

void loop() {

    motion.update();

    if (!motion.isBusy()) {
        if (state == 0) {
            motion.fwd_to_wall(NORTH, 5.0, 30.0);
            state++;
            delay(1000);
        } else if (state == 1) {
            motion.rotate(WEST); // turn left
            state++;
            delay(1000);
        } else if (state == 2) {
            motion.fwd_to_wall(WEST, 5.0, 30.0);
            state++;
            delay(1000);
        } else if (state == 3) {
            motion.rotate(NORTH); // turn right
            state++;
            delay(1000);
        } else if (state == 4) {
            motion.fwd_to_wall(NORTH, 5.0, 30.0);
            state++;
            delay(1000);

        } else if (state == 5) {
            motion.rotate(WEST);
            state++;
            delay(1000);
        } else if (state == 6){
            motion.fwd_to_wall(WEST, 5.0, 30.0);
            state++;
            delay(1000);
        } else if (state == 7){
            motion.rotate(SOUTH);
            state++;
            delay(1000);
        } else if (state == 8) {
            motion.fwd_to_wall(SOUTH, 5.0, 30.0);
            state++;
            delay(1000);
        }
        else {
            stop_motors();
            exit(0);
        }
    }
}