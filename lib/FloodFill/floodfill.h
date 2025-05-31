#ifndef FLOODFILL_H
#define FLOODFILL_H

#define AREA (LENGTH * LENGTH)
#define UNDEFINED 255

void initializeFloodfill();
void floodfill();
int getFloodfillValue(int row, int col);
void getNextMove(int row, int col, int dir, int* nextRow, int* nextCol, int* nextDir);

#endif
