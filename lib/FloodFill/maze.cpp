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
            }
            break;

        case SOUTH:
            if (row > 0) {
                index = (row - 1) * LENGTH + col;
                horizontalWalls[index] = state;
            }
            break;

        case EAST:
            if (col < LENGTH - 1) {
                index = col * LENGTH + row;
                verticalWalls[index] = state;
            }
            break;

        case WEST:
            if (col > 0) {
                index = (col - 1) * LENGTH + row;
                verticalWalls[index] = state;
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
                index = col * LENGTH + row;
                return verticalWalls[index];
            }
            break;

        case WEST:
            if (col > 0) {
                index = (col - 1) * LENGTH + row;
                return verticalWalls[index];
            }
            break;
    }

    if (safeMode == DEBUG_TRUE) return false; // Assume it doesn't for debugging purposes
    else return true; // Assume wall exists if index is invalid
}

bool inCenter(int row, int col) {
    return (row == 7 || row == 8) && (col == 7 || col == 8);
}

bool inHome(int row, int col) {
    return (row == 0) && (col == 0);
}