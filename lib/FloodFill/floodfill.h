#ifndef FLOODFILL_H
#define FLOODFILL_H

#define AREA (LENGTH * LENGTH)
#define UNDEFINED 255
#define GOAL_CENTER 0
#define GOAL_HOME 1

void initializeFloodfill();
void floodfill(int goalType);
int getFloodfillValue(int row, int col);
void getNextMove(int row, int col, int dir, int* nextRow, int* nextCol, int* nextDir);

#endif
