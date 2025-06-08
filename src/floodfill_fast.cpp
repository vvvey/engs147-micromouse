#include "Motors.h"
#include "IMU.h"
#include "TOF.h"
#include "Encoder.h"
#include "MotionController.h"
#include "WallLogic.h"
#include "floodfill.h"
// #include "<Arduino.h>"


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

// void MoveProcess(int goalType);
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


     
    floodfill(GOAL_CENTER);
}
int num_block_traveled = 0;

void loop() {
    motion.update();

    switch (current_state) {
        case SEARCH:
            if (!motion.isBusy()) {

                if (inCenter(curRow, curCol)) {
                    while (motion.isBusy()){
                        motion.update();
                    } 

                    motion.rotate(180);
                    while (motion.isBusy()){
                        motion.update();
                    }
                    motion.rotate(0);
                    while (motion.isBusy()){
                        motion.update();
                    }

                    stop_motors();
                    direction = NORTH;
                
                    clearUnvisitedCells();
                    floodfill(GOAL_CENTER);
                    buildBestPathFromFloodfillDebug(0, 0, stuff_path, &pathLen);
                    reversePath(stuff_path, reversed_path, pathLen);
                    // while(1) {
                    //     printMaze();

                    //     for (int i = 0; i < pathLen; i++) {
                    //         Serial.print(reversed_path[i]);
                    //         Serial.print(" ");
                    //     }
                    // }
                    current_state = RACE;
                    move_cnt = 0;
                    return;
                }

                WallReading_t wall_status = motion.getWallStatus();
                WallReading wall_reading = WallReading();
                int dis_traveled = wall_status.dis_traveled_mm;
                wall_reading.front = (wall_status.front_left_tof < front_threshold &&  wall_status.front_right_tof < front_threshold);
                wall_reading.left  = wall_status.left_tof  < side_threshold;
                wall_reading.right = wall_status.right_tof < side_threshold;
                
                visited[curRow * LENGTH + curCol] = true; // Mark current cell as visited
                updateWallMap(curRow, curCol, wall_reading, direction);    
                setWall(0,0, EAST, true); 
                floodfill(GOAL_CENTER);
                int nextRow, nextCol, nextDir;
                getNextMove(curRow, curCol, direction, &nextRow, &nextCol, &nextDir);
                
                if (direction == nextDir) {
                    int dir = direction - current_imu_drift;
                    if (dir < 0) dir += 360; // Normalize to [0, 360)
                    else if (dir >= 360) dir -= 360;
                    motion.fwd(dir, 355); 
                    delay(100);

                    if (direction == NORTH) curRow++;
                    else if (direction == EAST)  curCol++;
                    else if (direction == SOUTH) curRow--;
                    else if (direction == WEST)  curCol--;    
                    
                    break;
                } 
            }

            if (motion.controlType() == 1) { // Forward Search Mode
                WallReading_t wall_status = motion.getWallStatus();

                static bool prev_left = true;
                static bool prev_right = true;

                WallReading wall_reading = WallReading();
                wall_reading.front = (wall_status.front_left_tof < front_threshold &&  wall_status.front_right_tof < front_threshold);
                wall_reading.left  = wall_status.left_tof  < side_threshold;
                wall_reading.right = wall_status.right_tof < side_threshold;

                int dis_traveled = wall_status.dis_traveled_mm;
                if (dis_traveled % 180 > 125 && (dis_traveled + 90) / 180 > num_block_traveled ) { // check grid only when left and right tof are valid
                    // MoveProcess(GOAL_CENTER, wall_status);
                    

                    prev_left = wall_reading.left;
                    prev_right = wall_reading.right;

                    visited[curRow * LENGTH + curCol] = true; // Mark current cell as visited
                    updateWallMap(curRow, curCol, wall_reading, direction);
                    setWall(0,0, EAST, true);
                    floodfill(GOAL_CENTER);
                    
                    if (wall_reading.front) {
                        motion.stop_next_block(); // Stop the current block
                        while (motion.isBusy()) {
                            motion.update();
                        }
                        motion.fwd_to_wall(direction, FWD_WALL_DISTANCE, 450, 0.0); // Recover error by bringing robot closer to wall
                        delay(100);
                        while (motion.isBusy()) {
                            motion.update();
                        }

                        int imu_heading = IMU_readZ();
                        int imu_err = imu_heading - direction;
                        if (imu_err < -180) imu_err += 360; // Normalize to [-180, 180]
                        else if (imu_err > 180) imu_err -= 360;
                        // current_imu_drift = constrain(imu_err, -3, 3); // Constrain drift to [-3, 3]
                    }

                    int nextRow, nextCol, nextDir;
                    getNextMove(curRow, curCol, direction, &nextRow, &nextCol, &nextDir);

                    if (direction != nextDir) {
                        motion.stop_next_block(); // Stop the current block
                        while (motion.isBusy()) {
                            motion.update();
                        }

                    
                        int dir = nextDir - current_imu_drift;
                        if (dir < 0) dir += 360; // Normalize to [0, 360)
                        else if (dir >= 360) dir -= 360;
                        
                        motion.rotate(dir);
                        delay(100);
                        while (motion.isBusy()) {
                            motion.update();
                        }
                        direction = nextDir;
                        num_block_traveled = 0;
                    } 

                    if (motion.isBusy()) {
                        curRow = nextRow;
                        curCol = nextCol;
                        visited[curRow * LENGTH + curCol] = true;
                        num_block_traveled = (dis_traveled + 90) / 180; // Update number of blocks traveled
                    } else {
                        num_block_traveled = 0;
                    }

                    
                    
                }
            }

            break;
        case RACE:
            if (motion.isBusy()) {break;} 
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
                    current_imu_drift = constrain(imu_err, -3, 3); // Constrain drift to [-3, 3]
                    delay(100);
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
    }
}




// void MoveProcess(int goalType, WallReading_t walls) {
//     // Step 1: Sense and update wall map
//     WallReading wall_reading = WallReading();
//     wall_reading.front = walls.front_tof < front_threshold;
//     wall_reading.left  = walls.left_tof  < side_threshold;
//     wall_reading.right = walls.right_tof < side_threshold;

//     updateWallMap(curRow, curCol, wall_reading, direction);       
//     // setWall(0,0, EAST, true); // Debugging: Set wall at (0,0) to EAST

//     // Step 2: Floodfill map
//     floodfill(goalType);

//     visited[curRow * LENGTH + curCol] = true;
//     // Step 3: Decide next move
//     int nextRow, nextCol, nextDir;
//     getNextMove(curRow, curCol, direction, &nextRow, &nextCol, &nextDir);

//     // Step 4: Decide whether to rotate or move (don't update cell if rotating)
//    if (nextDir != direction) {
//         if (wall_reading.front){
//             motion.fwd_to_wall(direction, FWD_WALL_DISTANCE, DISTANCE_SPEED, 0.0); // Recover error by bringing robot closer to wall
//             while (motion.isBusy()){
//                 motion.update();
//             }
//             int imu_heading = IMU_readZ();
//             int imu_err = direction - imu_heading;
            
//             if (imu_err < -180) imu_err += 360; // Normalize to [-180, 180]
//             else if (imu_err > 180) imu_err -= 360;
//             current_imu_drift = constrain(imu_err, -3, 3); // Constrain drift to [-5, 5]
//             // delay(100);
//         }
//             float dir = nextDir;
//             if (dir < 0) dir += 360; // Normalize to [0, 360)
//             else if (dir >= 360) dir -= 360;
//             motion.rotate(dir);

//         if (goalType == GOAL_HOME) {
//             int delta = (nextDir - direction + 360) % 360;
//             int moveCode = 0;
//             if (delta == 90) moveCode = 1;        // Right
//                 else if (delta == 270) moveCode = 0;  // Left
//                 else if (delta == 180) moveCode = 2;  // 180 Turn

//                 best_path[best_path_index++] = moveCode;
//             }

//             direction = nextDir;
//     } else {
//         int dir = direction;
//         if (dir < 0) dir += 360; // Normalize to [0, 360)
//         else if (dir >= 360) dir -= 360;
//         motion.fwd_to_dis(dir, CELL_LENGTH, DISTANCE_SPEED);

//         if (direction == NORTH) curRow++;
//         else if (direction == EAST)  curCol++;
//         else if (direction == SOUTH) curRow--;
//         else if (direction == WEST)  curCol--;

//         if (goalType == GOAL_HOME) {
//             best_path[best_path_index++] = 3;  // Straight
//         }
//     }


//     // Step 5: Update state
//     lastRow = curRow;
//     lastCol = curCol;
// }

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
