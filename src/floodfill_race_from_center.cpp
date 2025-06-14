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
#define STOP 3

#define CELL_LENGTH 185.0 // Distance to stop in fwd_dis
#define DISTANCE_SPEED 600.0
#define DISTANCE_SPEED_RACE 450.0
#define CELL_LENGTH_RACE 180
#define FWD_WALL_DISTANCE 20

bool visited[LENGTH * LENGTH] = { false };

void MoveProcess(int goalType);
void showWallsAndPrintWait(WallReading w, int row, int col, int direction, int nextRow, int nextCol, int nextDir);
void showWallsAndWait(WallReading w, int row, int col, int direction, int nextRow, int nextCol, int nextDir);
void printMazeDebugLoop();
void reversePath(int* original, int* reversed, int& length);
void buildBestPathFromFloodfillDebug(int startRow, int startCol, int* path, int* pathLen);
void blinkLEDS();
void printMaze();
void clearUnvisitedCells();

// const int LENGTH = 16;
const int HEIGHT = 16;

extern bool horizontalWalls[HEIGHT + 1][LENGTH]; // Between rows, so +1
extern bool verticalWalls[HEIGHT][LENGTH + 1];   // Between cols, so +1
extern int floodfillArr[LENGTH * HEIGHT];


MotionController motion;

// Maze position & direction tracking
int curRow = 0, curCol = 0;
int lastRow = 0, lastCol = 0;
int direction = NORTH;
bool run = false;
int current_state = 0;

int best_path[256] = {0};
int best_path_index = 0;
int reversed_path[256];
int move_cnt = 0;

int stuff_path[256];
int pathLen;

// float total_imu_drift = 0.0;
// float total_imu_drift_count = 0.0;
int current_imu_drift = 0;

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
    // if (digitalRead(START_BTN) == LOW) { 
    //     run = true;
    //     Serial.println("Start Pressed");
    //     delay(500);
    // }

    // if (digitalRead(STOP_BTN) == LOW) {
    //     stop_motors();
    //     exit(0);
    // }

    // if (!run) return;

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
                while (motion.isBusy());*/ // for fun
                stop_motors();
                blinkLEDS();
                // Serial.println("Reached center. Entering debug mode.");
                // printMazeDebugLoop();
                // Serial.println("Continuing to HOME...");
                current_state = RACE;
                motion.rotate(NORTH);
                while (motion.isBusy()){
                    motion.update();
                } 
                direction = NORTH;
                
                clearUnvisitedCells();
                floodfill(GOAL_CENTER);
                buildBestPathFromFloodfillDebug(0, 0, stuff_path, &pathLen);
                reversePath(stuff_path, reversed_path, pathLen);
            }
            break;
        case RACE:
            // while(1){
                
            //     printMaze();
            //     buildBestPathFromFloodfillDebug(0, 0, stuff_path, &pathLen);
            //     for (int i = 0; i < pathLen; i++) {
            //         Serial.print(stuff_path[i]);
            //         Serial.print(" ");
            //     }
            //     Serial.println("\n===============================");
            //     // buildBestPathFromFloodfillDebug(curRow,curCol, stuff_path, &pathLen);
            //     delay(1000);
            //     if (digitalRead(LOG_BTN) == LOW) {break;}
            // }
            // while(1){
            //     Serial.println("=== Path to Center (Encoded) ===");
            //     for (int i = 0; i < pathLen; i++) {
            //         Serial.print(stuff_path[i]);
            //         Serial.print(" ");
            //     }
            //     Serial.println("\n===============================");
            //     delay(1000);
            //     if (digitalRead(LOG_BTN) == LOW) {break;}
            // }
            if ((reversed_path[move_cnt] == 0) || (reversed_path[move_cnt] == 1)){
                float front_left_dist  = TOF_getDistance(FRONT_LEFT); 
                float front_right_dist = TOF_getDistance(FRONT_RIGHT);
                if ((front_left_dist >= 0 && front_right_dist >= 0) && (front_left_dist < front_threshold) && (front_right_dist < front_threshold)){
                    motion.fwd_to_wall(direction, FWD_WALL_DISTANCE, DISTANCE_SPEED_RACE, 0.0); // Recover error by bringing robot closer to wall
                    while (motion.isBusy()){
                        motion.update();
                    }

                    int heading = IMU_readZ();
                    int imu_err = direction - heading;
                    if (imu_err < -180) imu_err += 360; // Normalize to [-180, 180]
                    else if (imu_err > 180) imu_err -= 360;
                    current_imu_drift = constrain(imu_err, -5, 5); // Constrain drift to [-5, 5]
                    // delay(100);
                }

                if (reversed_path[move_cnt] == 0) {
                    int dir = (direction + 270 - current_imu_drift) % 360;
                    if (dir < 0) dir += 360; // Normalize to [0, 360)
                    else if (dir >= 360) dir -= 360;
                    motion.rotate(dir);  // Left turn
                    direction = (direction + 270) % 360;
                    move_cnt++;
                    // delay(200); // Small delay to allow rotation to complete
                }
                else {
                    int dir = (direction + 90 - current_imu_drift) % 360;
                    if (dir < 0) dir += 360; // Normalize to [0, 360)
                    else if (dir >= 360) dir -= 360;
                    motion.rotate(dir);   // Right turn
                    direction = (direction + 90) % 360;
                    move_cnt++;
                    // delay(200); // Small delay to allow rotation to complete
                }
            }
            else if (reversed_path[move_cnt] == 3) {
                // Count how many consecutive forwards
                int forward_steps = 0;
                while (move_cnt + forward_steps < pathLen && reversed_path[move_cnt + forward_steps] == 3) {
                    forward_steps++;
                }
                int dir = direction - current_imu_drift;
                if (dir < 0) dir += 360; // Normalize to [0, 360)
                else if (dir >= 360) dir -= 360;
                motion.fwd_to_dis(dir, CELL_LENGTH_RACE*forward_steps, DISTANCE_SPEED_RACE);
                move_cnt += forward_steps;

                // Update position just in case we want to do anything with robot in future
                if (direction == NORTH) curRow += forward_steps;
                else if (direction == SOUTH) curRow -= forward_steps;
                else if (direction == EAST)  curCol += forward_steps;
                else if (direction == WEST)  curCol -= forward_steps;

                // delay(200); // Small delay to allow movement to complete
            }

            if (inHome(curRow, curCol)) {
                //motion.rotate(direction+180);
                while (motion.isBusy()){
                    motion.update();
                }
                stop_motors();
                blinkLEDS();
                current_state = STOP;
            }     
            /*Serial.println("=== Best Path (Return Home) ===");
            for (int i = 0; i < best_path_index; i++) {
                Serial.print(best_path[i]);
                Serial.print(" ");
            }
            Serial.println("\n===============================");
            
            Serial.println("=== Best Reverse Path (Race Center) ===");
            for (int i = 0; i < best_path_index; i++) {
                Serial.print(reversed_path[i]);
                Serial.print(" ");
            }
            Serial.println("\n===============================");*/

            break;
        case STOP:
            Serial.println("Done.");
            stop_motors();
            break;
        default:
            Serial.println("Error, default case.");
            stop_motors();
            break;
        }
    }
}


void MoveProcess(int goalType) {
    // Step 1: Sense and update wall map
    WallReading walls = readWalls(curRow, curCol, direction);
    updateWallMap(curRow, curCol, walls, direction);       
    setWall(0,0, EAST, true); // Debugging: Set wall at (0,0) to EAST

    // Step 2: Floodfill map
    floodfill(goalType);

    visited[curRow * LENGTH + curCol] = true;
    // Step 3: Decide next move
    int nextRow, nextCol, nextDir;
    getNextMove(curRow, curCol, direction, &nextRow, &nextCol, &nextDir);

    // Step 3.5: Debug
    //showWallsAndPrintWait(walls, curRow, curCol, direction, nextRow, nextCol, nextDir); // Takes time, removing to see results

    // Step 4: Decide whether to rotate or move (don't update cell if rotating)
   if (nextDir != direction) {
    if (walls.front){
        motion.fwd_to_wall(direction, FWD_WALL_DISTANCE, DISTANCE_SPEED, 0.0); // Recover error by bringing robot closer to wall
        while (motion.isBusy()){
            motion.update();
        }
        int imu_heading = IMU_readZ();
        int imu_err = direction - imu_heading;
        
        if (imu_err < -180) imu_err += 360; // Normalize to [-180, 180]
        else if (imu_err > 180) imu_err -= 360;
        current_imu_drift = constrain(imu_err, -3, 3); // Constrain drift to [-5, 5]
        // delay(100);
    }
     float dir = nextDir - current_imu_drift;
        if (dir < 0) dir += 360; // Normalize to [0, 360)
        else if (dir >= 360) dir -= 360;
    motion.rotate(dir);

    if (goalType == GOAL_HOME) {
        int delta = (nextDir - direction + 360) % 360;
        int moveCode = 0;
        if (delta == 90) moveCode = 1;        // Right
            else if (delta == 270) moveCode = 0;  // Left
            else if (delta == 180) moveCode = 2;  // 180 Turn

            best_path[best_path_index++] = moveCode;
        }

        direction = nextDir;
    } else {
        int dir = direction - current_imu_drift;
        if (dir < 0) dir += 360; // Normalize to [0, 360)
        else if (dir >= 360) dir -= 360;
        motion.fwd_to_dis(dir, CELL_LENGTH, DISTANCE_SPEED);

        if (direction == NORTH) curRow++;
        else if (direction == EAST)  curCol++;
        else if (direction == SOUTH) curRow--;
        else if (direction == WEST)  curCol--;

        if (goalType == GOAL_HOME) {
            best_path[best_path_index++] = 3;  // Straight
        }
    }


    // Step 5: Update state
    lastRow = curRow;
    lastCol = curCol;
}

void clearUnvisitedCells() {
    for (int i = 0; i < HEIGHT * LENGTH; i++) {
        int row = i / LENGTH;
        int col = i % LENGTH;

        if (!visited[i] && !inCenter(row, col)) {
            // Set walls in all four directions
            setWall(row, col, 0, true);
            setWall(row, col, 1,  true);
            setWall(row, col, 2, true);
            setWall(row, col, 3,  true);
        }
    }
}

void showWallsAndPrintWait(WallReading w, int row, int col, int direction, int nextRow, int nextCol, int nextDir) {
    digitalWrite(LED_FRONT, w.front ? HIGH : LOW);
    digitalWrite(LED_LEFT,  w.left  ? HIGH : LOW);
    digitalWrite(LED_RIGHT, w.right ? HIGH : LOW);

    // Print once
    Serial.println("=== Wall Debug Info ===");
    Serial.print("Position: (");
    Serial.print(row); Serial.print(", "); Serial.print(col); Serial.println(")");

    Serial.print("Direction: "); Serial.println(direction);

    Serial.print("Walls → Front: "); Serial.print(w.front);
    Serial.print(", Left: "); Serial.print(w.left);
    Serial.print(", Right: "); Serial.println(w.right);
    Serial.println("Press CONTINUE button to proceed...\n");

    // Wait for button
    //while (digitalRead(CONTINUE_BTN) == HIGH) {
      //  delay(10);
    //}
    delay(500);

    digitalWrite(LED_LEFT, LOW);
    digitalWrite(LED_FRONT, LOW);
    digitalWrite(LED_RIGHT, LOW);
    delay(300);
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

void reversePath(int* original, int* reversed, int& length) {
    int newIndex = 0;

    for (int i = length - 1; i >= 0; i--) {
        int t = original[i];

        // Skip 180 degree turns
        if (t == 2) {
            continue;
        }

        // Only need to change left and right turn I believe
        switch (t) {
            case 0: reversed[newIndex++] = 1; break; // Left --> Right
            case 1: reversed[newIndex++] = 0; break; // Right --> Left
            case 3: reversed[newIndex++] = 3; break; 
        }
    }

    length = newIndex; // Update original index to reflect new length
}

void buildBestPathFromFloodfillDebug(int startRow, int startCol, int* path, int* pathLen) {
    int tempDir = 0;
    int row = startRow;
    int col = startCol;
    int index = 0;

    // Serial.println("=== Debug: Building Best Path from Floodfill ===");

    while (!inCenter(row, col) && index < 256) {
        // Serial.print("At cell: (");
        // Serial.print(row);
        // Serial.print(", ");
        // Serial.print(col);
        // Serial.println(")");

        int minVal = 255;
        int bestDir = -1;
        int nextRow = row;
        int nextCol = col;

        for (int d = 0; d < 4; d++) {
            int r = row + (d == 0) - (d == 2);
            int c = col + (d == 1)  - (d == 3);

            if (!isValid(r, c)) continue;
            
            if (existWall(row, col, d, DEBUG_TRUE)) {
                // Serial.print("  Wall exists at dir ");
                // Serial.println(d);
                continue;
            }

            int val = getFloodfillValue(r, c);
            if (val == UNDEFINED) {
                // Serial.print("  Skipping undefined floodfill at dir ");
                // Serial.println(d);
                continue;
            }

            // Serial.print("  Direction ");
            // Serial.print(d);
            // Serial.print(": floodfill val = ");
            // Serial.println(val);

            if (val < minVal) {
                minVal = val;
                bestDir = d;
                nextRow = r;
                nextCol = c;
            }
        }

        if (bestDir == -1) {
            // Serial.println("  Path dead-end: floodfill may be invalid.");
            break;
        }

        int delta = (bestDir - tempDir + 4) % 4;
        int move = -1;

        if (delta == 0) {
            move = 3; // Forward
            row = nextRow;
            col = nextCol;
        } else if (delta == 1) {
            move = 1; // Right
        } else if (delta == 2) {
            move = 2; // 180
        } else if (delta == 3) {
            move = 0; // Left
        }

        // Serial.print("  Chosen direction: ");
        // Serial.print(bestDir);
        // Serial.print(", move encoded as: ");
        // Serial.println(move);

        path[index++] = move;

        if (delta != 0) {
            tempDir = bestDir;
        }

        if (inCenter(row, col)) {
            // Serial.print("Reached center at: (");
            // Serial.print(row);
            // Serial.print(", ");
            // Serial.print(col);
            // Serial.println(")");
            break;
        }
        // delay(100);
    }

    *pathLen = index;

    // Serial.print("Total path length: ");
    // Serial.println(*pathLen);
    // Serial.println("===============================================");
}

void blinkLEDS(){
    digitalWrite(LED_LEFT, HIGH);
    digitalWrite(LED_FRONT, HIGH);
    digitalWrite(LED_RIGHT, HIGH);
    delay(1000);
    digitalWrite(LED_LEFT, LOW);
    digitalWrite(LED_FRONT, LOW);
    digitalWrite(LED_RIGHT, LOW);
}
void printMaze() {
    Serial.println("===== Maze View =====");

    for (int row = HEIGHT - 1; row >= 0; row--) {
        // Print top horizontal walls of the current row
        for (int col = 0; col < LENGTH; col++) {
            Serial.print("+");
            if (existWall(row, col, 0, DEBUG_TRUE)) {
                Serial.print("---");
            } else {
                Serial.print("   ");
            }
        }
        Serial.println("+");

        // Print vertical walls and floodfill values
        for (int col = 0; col < LENGTH; col++) {
            // West wall
            if (existWall(row, col, 3, DEBUG_TRUE)) {
                Serial.print("|");
            } else {
                Serial.print(" ");
            }

            // Floodfill value, padded to 3 characters
            int val = floodfillArr[row * LENGTH + col];
            if (val < 10) Serial.print("  ");
            else if (val < 100) Serial.print(" ");
            Serial.print(val);
        }

        // Rightmost east wall at the end of the row
        if (true) {
            Serial.println("|");
        } else {
            Serial.println(" ");
        }
    }

    // Print bottom horizontal walls of row 0
    for (int col = 0; col < LENGTH; col++) {
        Serial.print("+");
        if (existWall(0, col, 2, DEBUG_TRUE)) {
            Serial.print("---");
        } else {
            Serial.print("   ");
        }
    }
    Serial.println("+");

    Serial.println("=====================");
}
