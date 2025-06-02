#include "WallLogic.h"
#include "TOF.h"
#include "maze.h"

WallReading readWalls(int row, int col, int dir) {
    WallReading result;

    float front_left_dist  = TOF_getDistance(FRONT_LEFT); 
    float front_right_dist = TOF_getDistance(FRONT_RIGHT);
    float left_dist        = TOF_getDistance(LEFT); 
    float right_dist       = TOF_getDistance(RIGHT);

    bool front_wall = false;
    bool left_wall  = false;
    bool right_wall = false;

    // Only consider valid readings (>= 0)
    if (front_left_dist >= 0 && front_right_dist >= 0)
        front_wall = (front_left_dist < wall_threshold) && (front_right_dist < wall_threshold);

    if (left_dist >= 0)
        left_wall = left_dist < wall_threshold;

    if (right_dist >= 0)
        right_wall = right_dist < wall_threshold;

    result.front = front_wall;
    result.left  = left_wall;
    result.right = right_wall;

    return result;
}


void updateWallMap(int row, int col, WallReading w, int direction) {
    // Assumes you have setWall(row, col, dir, value)
    // dir: 0 = NORTH, 1 = EAST, 2 = SOUTH, 3 = WEST

    // Map relative sensor directions to absolute directions
    int dir_front = direction;
    int dir_left  = (direction + 270) % 360;  // -90 degrees
    int dir_right = (direction + 90)  % 360;

    int abs_front = dir_front / 90;
    int abs_left  = dir_left  / 90;
    int abs_right = dir_right / 90;

    if (w.front) setWall(row, col, abs_front, true);
    if (w.left)  setWall(row, col, abs_left,  true);
    if (w.right) setWall(row, col, abs_right, true);
}
