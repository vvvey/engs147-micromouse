#ifndef WALLLOGIC_H
#define WALLLOGIC_H

#include "maze.h"   // for LENGTH, setWall
#include "TOF.h"    // for TOF_getDistance()

#define side_threshold 100.0
#define front_threshold 150.0
#define wall_stop 50.0

struct WallReading {
    bool front;
    bool left;
    bool right;
};

// Declaration of functions
WallReading readWalls(int row, int col, int direction);
void updateWallMap(int row, int col, WallReading w, int direction);

#endif
