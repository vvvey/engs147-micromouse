#include "floodfill.h"
#include "maze.h"
#include <cppQueue.h>

int floodfillArr[AREA];

void initializeFloodfill() {
    for (int i = 0; i < AREA; i++) {
        floodfillArr[i] = UNDEFINED;
    }
}

void floodfill() {
    initializeFloodfill();

    cppQueue queue(sizeof(int), AREA, FIFO, false);

    // Start from maze center (e.g. (7,7) or (8,8) for 16×16)
    int centerRow = LENGTH / 2;
    int centerCol = LENGTH / 2;

    int startZ = rowColtoZ(centerRow, centerCol);
    int startVal = 0;
    int packed = startZ | (startVal << 8);
    queue.push(&packed); 

    while (!queue.isEmpty()) {
        int encoded;
        queue.pop(&encoded);

        int z = encoded & 0xFF;
        int val = (encoded >> 8) & 0xFF;

        if (z < 0 || z >= AREA) continue;
        if (floodfillArr[z] != UNDEFINED && floodfillArr[z] <= val) continue;

        floodfillArr[z] = val;

        int row, col;
        zToRowCol(z, row, col);

        for (int dir = 0; dir < 4; dir++) {
            if (!existWall(row, col, dir)) {
                int nextRow = row + (dir == NORTH) - (dir == SOUTH);
                int nextCol = col + (dir == EAST) - (dir == WEST);
                int nextZ = rowColtoZ(nextRow, nextCol);
                int nextEncoded = nextZ | ((val + 1) << 8);
                queue.push(&nextEncoded);
            }
        }
    }
}

int getFloodfillValue(int row, int col) {
    return floodfillArr[rowColtoZ(row, col)];
}

void getNextMove(int row, int col, int dir, int* nextRow, int* nextCol, int* nextDir) {
    int bestVal = 255;
    int chosenDir = dir;

    for (int d = 0; d < 4; d++) {
        if (!existWall(row, col, d)) {
            int r = row + (d == NORTH) - (d == SOUTH);
            int c = col + (d == EAST)  - (d == WEST);

            if (!isValid(r, c)) continue;

            int val = getFloodfillValue(r, c);
            if (val < bestVal || (val == bestVal && d == dir)) {
                bestVal = val;
                *nextRow = r;
                *nextCol = c;
                chosenDir = d;
            }
        }
    }

    *nextDir = chosenDir * 90;  // Convert to degrees (0, 90, 180, 270)
}