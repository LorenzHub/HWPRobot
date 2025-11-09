#include "labyrinth.h"
#include <communication/communication.h>

#include <string.h>


// determine byte number of wall with index idx
#define BYTE(idx) (idx >> 3)

// determine bit number of byte BYTE(idx) of wall with index idx
#define BIT(idx) (idx & 0x07)

// get bit of wall with index idx
#define GET_WALL(idx, array) ((array[BYTE(idx)] & (1 << BIT(idx))) != 0)

// set bit of wall with index idx
#define SET_WALL(idx, array) (array[BYTE(idx)] |= (1 << BIT(idx)))

// clear bit of wall with index idx
#define CLEAR_WALL(idx, array) (array[BYTE(idx)] &= ~(1 << BIT(idx)))


// wall storage
static LabyrinthWalls_t labyrinthWalls;


void labyrinth_init(void) {
    // initialize wall storage, all walls are not present
    labyrinthWalls.rows = LABYRINTH_ROWS;
    labyrinthWalls.cols = LABYRINTH_COLS;
    memset(&labyrinthWalls.walls, 0, sizeof(labyrinthWalls.walls));
}


uint16_t labyrinth_getCellIndex(const uint8_t row, const uint8_t col) {
    return (uint16_t)row * (uint16_t)LABYRINTH_COLS + (uint16_t)col;
}


uint16_t labyrinth_getWallIndex(const uint8_t row, const uint8_t col, const Direction_t dir) {

    if (row >= LABYRINTH_ROWS || col >= LABYRINTH_COLS) {
        return 0xFFFF;
    }

    uint16_t wallIdx = 0;

    uint8_t _row = row;
    uint8_t _col = col;
    Direction_t _dir = dir;
    if (_dir == DIRECTION_EAST) {
        _dir = DIRECTION_WEST;
        _col++;
    } else if (_dir == DIRECTION_SOUTH) {
        _dir = DIRECTION_NORTH;
        _row++;
    }

    if (_dir == DIRECTION_WEST) {
        wallIdx = (uint16_t)_row * ((uint16_t)LABYRINTH_COLS + 1)
                + (uint16_t)_col; // number of the vertical wall
    } else if (_dir == DIRECTION_NORTH) {
        wallIdx = (uint16_t)_col * ((uint16_t)LABYRINTH_ROWS + 1)
                + (uint16_t)_row // number of the horizontal wall
                + ((uint16_t)LABYRINTH_ROWS * ((uint16_t)LABYRINTH_COLS + 1)); // number of vertical walls in total
    }

    return wallIdx;
}


Walls_t labyrinth_getWalls(const uint8_t row, const uint8_t col) {
    Walls_t walls;

    uint16_t i = labyrinth_getWallIndex(row, col, DIRECTION_WEST);
    if (i == 0xFFFF) {
        walls.walls = 0;
        return walls;
    }

    walls.wall.west = GET_WALL(i, labyrinthWalls.walls) ? WALLSTATE_SET : WALLSTATE_CLEARED;
    walls.wall.east = GET_WALL((i+1), labyrinthWalls.walls) ? WALLSTATE_SET : WALLSTATE_CLEARED;

    i = labyrinth_getWallIndex(row, col, DIRECTION_NORTH);
    walls.wall.north = GET_WALL(i, labyrinthWalls.walls) ? WALLSTATE_SET : WALLSTATE_CLEARED;
    walls.wall.south = GET_WALL((i+1), labyrinthWalls.walls) ? WALLSTATE_SET : WALLSTATE_CLEARED;

    return walls;
}


void labyrinth_setWalls(const uint8_t row, const uint8_t col, const Walls_t walls) {
    uint16_t i = labyrinth_getWallIndex(row, col, DIRECTION_WEST);
    if (i == 0xFFFF)
        return;

    if (walls.wall.west == WALLSTATE_CLEARED)
        CLEAR_WALL(i, labyrinthWalls.walls);
    else
        SET_WALL(i, labyrinthWalls.walls);

    if (walls.wall.east == WALLSTATE_CLEARED)
        CLEAR_WALL((i+1), labyrinthWalls.walls);
    else
        SET_WALL((i+1), labyrinthWalls.walls);

    i = labyrinth_getWallIndex(row, col, DIRECTION_NORTH);
    if (walls.wall.north == WALLSTATE_CLEARED)
        CLEAR_WALL(i, labyrinthWalls.walls);
    else
        SET_WALL(i, labyrinthWalls.walls);

    if (walls.wall.south == WALLSTATE_CLEARED)
        CLEAR_WALL((i+1), labyrinthWalls.walls);
    else
        SET_WALL((i+1), labyrinthWalls.walls);
}


void labyrinth_clearAllWalls(void) {
    memset(&labyrinthWalls.walls, 0, sizeof(labyrinthWalls.walls));
}


const LabyrinthWalls_t* labyrinth_getAllWalls(void) {
    return &labyrinthWalls;
}
