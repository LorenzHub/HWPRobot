#ifndef LABYRINTH_H
#define LABYRINTH_H

#include "position.h"
#include <communication/packetTypes.h>
#include <stdio.h>
#include <stdbool.h>
#include "communication/packetTypes.h"

typedef struct{
    int16_t  x;
    int16_t y;
    Direction_t cardinalDirection; //0=NORTH,1=EAST,2=SOUTH,3=WEST
}LabyrinthPose_t;

typedef struct {
    bool north, south, east, west;  // isWall?
    uint8_t dirNorth, dirSouth, dirEast, dirWest; //0,1,2 sign for Algorithm of Tremaux
} Cell;

void exploreMaze();

void setLabyrinthPose(Pose_t pose);

bool hasEscaped();

void checkWalls(uint8_t* availableDirections);

bool isPlace();

Direction_t choosePlaceDirection();

Direction_t chooseWayDirection();

uint8_t oppositDirection(uint8_t dir);

Direction_t getCardinalDirectionfromLookingDirection(Direction_t dirLooking);

void setNoWall(Direction_t cardinalDirection);

bool hasWall(Direction_t cardinalDirection);

Direction_t leastVisitedDirection();

void DriveDirection(Direction_t nextDirection);

void resetMaze();
#endif // LABYRINTH_H