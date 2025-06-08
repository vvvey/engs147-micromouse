#include "Motors.h"
#include "IMU.h"
#include "TOF.h"
#include "Encoder.h"
#include "MotionController.h"
#include "WallLogic.h"
#include "floodfill.h"
// #include "Maze.h"
// 
#define NORTH 0
#define EAST 90
#define SOUTH 180
#define WEST 270

#define START_BTN 28
#define LOG_BTN 30
#define STOP_BTN 32

MotionController motion;
// Maze maze;

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


    // maze.initialize(0, 0, 8, 8); // Initialize maze with start and target positions
    delay(1000);

}
int state = 0;
bool run = true;
bool checked = false;
// float curr_heading_main = IMU_readZ();

void loop() {
    // if (digitalRead(START_BTN) == LOW) { 
    //     Serial.println("Start Button Pressed");
    //     run = true;
    //     delay(500);    
    // }

    // if (digitalRead(STOP_BTN) == LOW) {
    //     stop_motors();
    //     exit(0);
    // }

    if (run) {
        motion.update();

        if (!motion.isBusy()) { 
            if (state == 0) {
                motion.fwd(NORTH, 300); 
                state++;
                delay(500);
            } 
            
            else {
                stop_motors();
                // exit(0);
            }
        }
        
        
        WallReading_t wall_status = motion.getWallStatus();
        // Serial.print("Left TOF: ");
        // Serial.print(wall_status.left_tof);
        // Serial.print(" | Front TOF: ");
        // Serial.print(wall_status.front_tof);
        // Serial.print(" | Right TOF: ");
        // Serial.print(wall_status.right_tof);
        // Serial.print(" | Distance Traveled: ");
        // Serial.println(wall_status.dis_traveled_mm);
        if (!checked && wall_status.right_tof > 100 && wall_status.dis_traveled_mm > 90) {
            motion.stop_next_block();
            checked = true;
        }

    }

}