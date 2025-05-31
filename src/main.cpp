#include "Motors.h"
#include "IMU.h"
#include "TOF.h"
#include "Encoder.h"
#include "ArduinoMotorShieldR3.h"
#include "MotionController.h"

#define NORTH 0
#define EAST 90
#define SOUTH 180
#define WEST 270

#define START_BTN 28
#define LOG_BTN 30
#define STOP_BTN 32

MotionController motion;

void setup() {
    Serial.begin(115200);
    IMU_init();
    TOF_init();
    rightEnc.begin();
    leftEnc.begin();
    motor_driver.init();
    IMU_init();

    Serial.println("Hello World");

    pinMode(START_BTN, INPUT_PULLUP);
    pinMode(LOG_BTN, INPUT_PULLUP);
    pinMode(STOP_BTN, INPUT_PULLUP);

    delay(1000);
}


int state = 0;
bool run = false;
float curr_heading_main = IMU_readZ();

void loop() {
    if (digitalRead(START_BTN) == LOW) { 
        Serial.println("Start Button Pressed");
        run = true;
        delay(500);    
    }

    if (digitalRead(STOP_BTN) == LOW) {
        stop_motors();
        exit(0);
    }

    if (run) {
        motion.update();

        if (!motion.isBusy()) { 
            if (state == 0) {
                motion.fwd_to_wall(NORTH, 40, 450.0, 0.0); // Move forward to wall
                state++;
                delay(500);
            } else if (state == 1) {
                motion.rotate(WEST);
                state++;
                delay(500);
            } else if (state == 2) {
                motion.fwd_to_wall(WEST, 40, 450.0, 0.0); // Move forward to wall
                state++;
                delay(500);
            } else if (state == 3) {
                motion.rotate(SOUTH);
                state++;
                delay(500);
            } else if (state == 4) {
                motion.fwd_to_wall(SOUTH, 40, 450.0, 0.0); // Move forward to wall
                state++;
                delay(500);
            } else if (state == 5) {
                motion.rotate(WEST);
                state++;
                delay(500);
            } else if (state == 6) {
                motion.fwd_to_wall(WEST, 40, 450.0, 0.0); // Move forward to wall
                state++;
                delay(500);
            } else if (state == 7) {
                motion.rotate(EAST);
                state++; 
                delay(500);
            } else if (state == 8) {
                motion.fwd_to_wall(EAST, 40, 450.0, 0.0); // Move forward to wall
                state++;
                delay(500);
            } else if (state == 9) {
                motion.rotate(NORTH);
                state++;
                delay(500);
            } else if (state == 10) {
                motion.fwd_to_wall(NORTH, 40, 450.0, 0.0); // Move forward to a distance of 100mm
                state++;
                delay(500);
            } else if (state == 11) {
                motion.rotate(EAST);
                state++;
                delay(500);
            }
            else if (state == 12) {
                motion.fwd_to_wall(EAST, 40, 450.0, 0.0); 
                state++;
                delay(500);
            } else if (state == 13) {
                motion.rotate(SOUTH);
                state++;
                delay(500);
            } else if (state == 14) {
                motion.fwd_to_wall(SOUTH, 40, 450.0, 0.0); 
                state++;
                delay(500);
            } else if (state == 15) {
                motion.rotate(NORTH);
                state++;
                delay(500);
            }
            else {
                stop_motors();
                // exit(0);
            }
        }
    }

    
}