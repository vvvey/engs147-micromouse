#include "Motors.h"
#include "IMU.h"
#include "TOF.h"
#include "Encoder.h"
#include "MotionController.h"
#include "WallLogic.h"
#include "floodfill.h"


#define NORTH 0
#define EAST 90
#define SOUTH 180
#define WEST 270

#define START_BTN 28
#define LOG_BTN 30
#define STOP_BTN 32

MotionController motion;

// Maze position & direction tracking
int curRow = 0, curCol = 0;
int lastRow = 0, lastCol = 0;
int direction = NORTH;

bool run = false;

void setup() {
    Serial.begin(115200);
    IMU_init();
    TOF_init();
    rightEnc.begin();
    leftEnc.begin();
    motor_driver.init();

    pinMode(START_BTN, INPUT_PULLUP);
    pinMode(LOG_BTN, INPUT_PULLUP);
    pinMode(STOP_BTN, INPUT_PULLUP);

    initializeMaze();
    initializeFloodfill();

    delay(1000);
    Serial.println("Ready");
}

void loop() {
    if (digitalRead(START_BTN) == LOW) { 
        run = true;
        Serial.println("Start Pressed");
        delay(500);
    }

    if (digitalRead(STOP_BTN) == LOW) {
        stop_motors();
        exit(0);
    }

    if (!run) return;

    motion.update();

    if (!motion.isBusy()) {
        // Step 1: Sense and update wall map
        WallReading walls = readWalls(curRow, curCol, direction);
        updateWallMap(curRow, curCol, walls, direction);

        // Step 2: Floodfill map
        floodfill();

        // Step 3: Decide where to go
        int nextRow, nextCol, nextDir;
        getNextMove(curRow, curCol, direction, &nextRow, &nextCol, &nextDir);

        // Step 4: Rotate and move
        if (nextDir != direction) {
            motion.rotate(nextDir);
            direction = nextDir;  // Only update heading
        } else {
            motion.fwd_to_wall(nextDir, 35, 450.0, 0.0);
            curRow = nextRow;
            curCol = nextCol;
        }

        // Step 5: Update state
        lastRow = curRow;
        lastCol = curCol;

        // Step 6: Check center condition
        if (inCenter(curRow, curCol)) {
            stop_motors();
            Serial.println("Reached Center!");
            run = false;
        }
    }
}
