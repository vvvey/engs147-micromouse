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
 
    if (run) {
        motion.update();

        if (!motion.isBusy()) { 
            if (state == 0) {
                motion.fwd_to_dis(1500, round(curr_heading_main)); 
                motion.logData();
                state++;
                delay(500);
            }
            else {
                stop_motors();
                exit(0);
            }
        }
    }

    
}