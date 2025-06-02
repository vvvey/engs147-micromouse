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

#define LED_LEFT   29   
#define LED_FRONT  31    
#define LED_RIGHT  33
#define CONTINUE_BTN 25 

#define SEARCH 0
#define HOME 1
#define RACE 2


void MoveProcess(int goalType);
void showWallsAndWait(WallReading w, int row, int col, int direction, int nextRow, int nextCol, int nextDir);
void printMazeDebugLoop();

MotionController motion;

// Maze position & direction tracking
int curRow = 0, curCol = 0;
int lastRow = 0, lastCol = 0;
int direction = NORTH;
bool run = false;
int current_state = 0;

void setup() {
    digitalWrite(LED_LEFT, HIGH);
    digitalWrite(LED_FRONT, HIGH);
    digitalWrite(LED_RIGHT, HIGH);

    Serial.begin(115200);
    IMU_init();
    TOF_init();
    rightEnc.begin();
    leftEnc.begin();
    motor_driver.init();

    pinMode(START_BTN, INPUT_PULLUP);
    pinMode(LOG_BTN, INPUT_PULLUP);
    pinMode(STOP_BTN, INPUT_PULLUP);

    pinMode(LED_FRONT, OUTPUT);
    pinMode(LED_LEFT, OUTPUT);
    pinMode(LED_RIGHT, OUTPUT);
    pinMode(CONTINUE_BTN, INPUT_PULLUP);

    initializeMaze();
    initializeFloodfill();

    delay(1000);
    Serial.println("Ready");
    digitalWrite(LED_LEFT, LOW);
    digitalWrite(LED_FRONT, LOW);
    digitalWrite(LED_RIGHT, LOW);
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

        switch (current_state) {
        case SEARCH:
            MoveProcess(GOAL_CENTER);

            if (inCenter(curRow, curCol)) {
                while (motion.isBusy()){
                    motion.update();
                }
                /*motion.rotate(180);
                while (motion.isBusy());
                motion.rotate(180);
                while (motion.isBusy());*/
                stop_motors();
                Serial.println("Reached center. Entering debug mode.");
                printMazeDebugLoop();
                Serial.println("Continuing to HOME...");
                current_state = HOME;
            }
            break;
        case HOME:
            MoveProcess(GOAL_HOME);
            if (inHome(curRow, curCol)) {
                motion.rotate(NORTH);
                stop_motors();
                current_state = RACE;
            }
            break;
        case RACE:
            /*MoveProcess(GOAL_CENTER);

            if (inHome(curRow, curCol)) {
                motion.rotate(direction+180);
                stop_motors();
                current_state = RACE;
            }*/
            break;
        default:
            printf("Error, default case.");
            stop_motors();
            break;
        }
    }
}


void MoveProcess(int goalType) {
    // Step 1: Sense and update wall map
    WallReading walls = readWalls(curRow, curCol, direction);
    updateWallMap(curRow, curCol, walls, direction);         

    // Step 2: Floodfill map
    floodfill(goalType);

    // Step 3: Decide next move
    int nextRow, nextCol, nextDir;
    getNextMove(curRow, curCol, direction, &nextRow, &nextCol, &nextDir);

    // Step 3.5: Debug
    showWallsAndWait(walls, curRow, curCol, direction, nextRow, nextCol, nextDir);

    // Step 4: Decide whether to rotate or move (don't update cell if rotating)
    if (nextDir != direction) {
        motion.rotate(nextDir);
        direction = nextDir;
    } else {
        motion.fwd_to_dis(direction, 180, 450.0);
        if (direction == NORTH) curRow++;
        else if (direction == EAST)  curCol++;
        else if (direction == SOUTH) curRow--;
        else if (direction == WEST)  curCol--;
    }

    // Step 5: Update state
    lastRow = curRow;
    lastCol = curCol;
}

void showWallsAndWait(WallReading w, int row, int col, int direction, int nextRow, int nextCol, int nextDir) {
    digitalWrite(LED_FRONT, w.front ? HIGH : LOW);
    digitalWrite(LED_LEFT,  w.left  ? HIGH : LOW);
    digitalWrite(LED_RIGHT, w.right ? HIGH : LOW);

    //while (digitalRead(CONTINUE_BTN) == HIGH) {
        // Serial.println("=== Wall & Navigation Debug ===");
        // Serial.print("TOF Front Left: ");  Serial.println(TOF_getDistance(FRONT_LEFT));
        // Serial.print("TOF Front Right: "); Serial.println(TOF_getDistance(FRONT_RIGHT));
        // Serial.print("TOF Left: ");        Serial.println(TOF_getDistance(LEFT));
        // Serial.print("TOF Right: ");       Serial.println(TOF_getDistance(RIGHT));

        // Serial.print("Current Position: (");
        // Serial.print(row); Serial.print(", "); Serial.print(col); Serial.println(")");

        // Serial.print("Facing Direction: "); Serial.println(direction);

        // Serial.print("→ Next Move: Go to (");
        // Serial.print(nextRow); Serial.print(", ");
        // Serial.print(nextCol); Serial.print("), Dir: ");
        // Serial.println(nextDir);

        // Serial.println("Waiting for continue button...\n");

        //delay(10);
    //}
    delay(1000);

    digitalWrite(LED_LEFT, LOW);
    digitalWrite(LED_FRONT, LOW);
    digitalWrite(LED_RIGHT, LOW);
    delay(500);
}

void printMazeDebugLoop() {
    while (digitalRead(CONTINUE_BTN) == HIGH) {
        Serial.println("===== Maze View =====");
        for (int row = LENGTH - 1; row >= 0; row--) {
            for (int col = 0; col < LENGTH; col++) {
                Serial.print(existWall(row, col, NORTH, DEBUG_TRUE) ? "↑" : " ");
                Serial.print(existWall(row, col, EAST, DEBUG_TRUE)  ? "→" : " ");
                Serial.print(existWall(row, col, SOUTH, DEBUG_TRUE) ? "↓" : " ");
                Serial.print(existWall(row, col, WEST, DEBUG_TRUE)  ? "←" : " ");
                Serial.print(" | ");
            }
            Serial.println();
        }
        Serial.println("=====================");
        delay(1000); 
    }

    
    delay(300); 
}
