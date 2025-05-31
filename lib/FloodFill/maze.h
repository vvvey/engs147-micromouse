#ifndef MAZE_H
#define MAZE_H

#define LENGTH 16
#define WALL_ARRAY_SIZE (LENGTH * (LENGTH - 1))

enum Direction {
    NORTH = 0,
    EAST  = 1,
    SOUTH = 2,
    WEST  = 3
};

void initializeMaze();
void setWall(int row, int col, int dir, bool state);
bool existWall(int row, int col, int dir);
bool inCenter(int row, int col);

// Convert (row, col) to flat index
inline int rowColtoZ(int row, int col) {
    return row * LENGTH + col;
}

// Convert flat index to (row, col)
inline void zToRowCol(int z, int &row, int &col) {
    row = z / LENGTH;
    col = z % LENGTH;
}

// Optional: bounds check helper
inline bool isValid(int row, int col) {
    return row >= 0 && row < LENGTH && col >= 0 && col < LENGTH;
}

#endif
