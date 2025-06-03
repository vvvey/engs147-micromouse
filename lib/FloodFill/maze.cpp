#include "maze.h"


bool horizontalWalls[WALL_ARRAY_SIZE] = { false }; // For NORTH/SOUTH
bool verticalWalls[WALL_ARRAY_SIZE] = { false }; // For EAST/WEST


void initializeMaze() {
    for (int i = 0; i < WALL_ARRAY_SIZE; i++) {
        verticalWalls[i] = false;
        horizontalWalls[i] = false;
    }
}

void setWall(int row, int col, int dir, bool state) {
    int index;

    switch (dir) {
        case NORTH:
            if (row < LENGTH - 1) {
                index = row * LENGTH + col;
                horizontalWalls[index] = state;
                int neighborRow = row + 1;
                if (neighborRow < LENGTH) {
                    int neighborIndex = neighborRow * LENGTH + col; // same index
                    horizontalWalls[neighborIndex] = state; // reflects SOUTH of neighbor
                }
            }
            break;

        case SOUTH:
            if (row > 0) {
                index = (row - 1) * LENGTH + col;
                horizontalWalls[index] = state;
                int neighborRow = row - 1;
                if (neighborRow >= 0) {
                    int neighborIndex = neighborRow * LENGTH + col;
                    horizontalWalls[neighborIndex] = state;
                }
            }
            break;

        case EAST:
            if (col < LENGTH - 1) {
                index = row * LENGTH + col;
                verticalWalls[index] = state;
                int neighborCol = col + 1;
                if (neighborCol < LENGTH) {
                    int neighborIndex = row * LENGTH + neighborCol;
                    verticalWalls[neighborIndex] = state;
                }
            }
            break;

        case WEST:
            if (col > 0) {
                index = row * LENGTH + (col - 1);
                verticalWalls[index] = state;
                int neighborCol = col - 1;
                if (neighborCol >= 0) {
                    int neighborIndex = row * LENGTH + (neighborCol);
                    verticalWalls[neighborIndex] = state;
                }
            }
            break;
    }
}


bool existWall(int row, int col, int dir, bool safeMode) {
    int index;

    switch (dir) {
        case NORTH:
            if (row < LENGTH - 1) {
                index = row * LENGTH + col;
                return horizontalWalls[index];
            }
            break;

        case SOUTH:
            if (row > 0) {
                index = (row - 1) * LENGTH + col;
                return horizontalWalls[index];
            }
            break;

        case EAST:
            if (col < LENGTH - 1) {
                index = row * LENGTH + col;  //
                return verticalWalls[index];
            }
            break;

        case WEST:
            if (col > 0) {
                index = row * LENGTH + (col - 1);  // 
                return verticalWalls[index];
            }
            break;
    }

    // Default/fallback logic
    if (safeMode == DEBUG_TRUE) return false;
    else return true;
}


bool* getHorizontalWalls() {
    return horizontalWalls;
}

bool* getVerticalWalls() {
    return verticalWalls;
}

bool inCenter(int row, int col) {
    return (row == 7 || row == 8) && (col == 7 || col == 8);
}

bool inHome(int row, int col) {
    return (row == 0) && (col == 0);
}