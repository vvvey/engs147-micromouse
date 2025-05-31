#include "WallLogic.h"
#include "TOF.h"
#include "maze.h"

WallReading readWalls(int row, int col, int dir) {
    WallReading result;

    // Read raw TOF distances
    int front_left_dist = TOF_getDistance(FRONT_LEFT); 
    int front_right_dist = TOF_getDistance(FRONT_RIGHT);
    int left_dist = TOF_getDistance(LEFT); 
    int right_dist = TOF_getDistance(RIGHT); 

    // Assume a wall is present if distance < threshold

    bool front_wall = (front_left_dist < wall_threshold) && (front_right_dist < wall_threshold);
    bool left_wall  = left_dist < wall_threshold;
    bool right_wall = right_dist < wall_threshold;

    // Fill struct with logical wall directions based on orientation
    result.front = front_wall;
    result.left = left_wall;
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
