#ifndef LABYRINTH_H
#define LABYRINTH_H

#include "position.h"
#include <communication/packetTypes.h>

typedef struct{
    uint8_t  x;
    uint8_t y;
    uint8_t cardinalDirection; //1=NORTH,2=EAST,3=SOUTH,4=WEST
}LabyrinthPose_t;

typedef struct {
    int visited;      // 0, 1, 2 for Algorithm of Trémaux
    bool north, south, east, west;  // isWall?
} Cell;

void exploreMaze();

void setLabyrinthPose(Pose_t pose);

#endif // LABYRINTH_H