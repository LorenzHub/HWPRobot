#include "labyrinth.h"
#include <math.h>
#include <communication/communication.h>
#include <inttypes.h>
#include "io/adc/adc.h"
#include <time.h>
#include "statemachine.h"
#include "position.h"
#include "tools/labyrinth/labyrinth.h"

//initialize fromDirection with opposite of startCardinalDirection(to ensure that turn around in chooseWayDirection works)
//also in resetMaze()
//and also nextDirection
//and in labyrintPose.cardinalDirection
//has to start {3,3,0} because of updatePose


// Forward declaration of helper function
static uint8_t* dirCountPtr(Cell *c, uint8_t dir);

/*Maze is in 0...6*/
LabyrinthPose_t labyrinthPose = {3,3,0}; 
Cell maze[7][7]; 
static uint8_t fromDirection = 2; //0=NORTH,1=EAST,2=SOUTH,3=WEST,initial=oppositeDirection(startCardinalDirection)
static Direction_t nextDirection = DIRECTION_NORTH; //initialize nextDirection with startCardinalDirection!

void exploreMaze() {
    
    //Initialization
    static uint8_t initialized = 0;
    if(!initialized){
        srand(time(NULL));  // intialize random generator
        resetMaze();
        statemachine_setTargetDistance(243); //Cell size 253.3mm with wall
        statemachine_setTargetPWM(5500);
        initialized=1;
    }

    if(isPlace()){
        choosePlaceDirection();
    }

    else{
        chooseWayDirection();
    }

    static uint8_t init = 0;
    //Increment fromDirection counter
    if(init!=0)*dirCountPtr(&maze[labyrinthPose.x][labyrinthPose.y], fromDirection) += 1;
    init=1;
    
    //increment chosen direction counter
    *dirCountPtr(&maze[labyrinthPose.x][labyrinthPose.y], (uint8_t)nextDirection) += 1; 

    fromDirection = oppositDirection((uint8_t) nextDirection);

    DriveDirection(nextDirection);

    if(hasEscaped())setState(IDLE);
}

void resetMaze(){
    for(uint8_t i=0;i<7;i++){
        for(uint8_t j=0;j<7;j++){
            maze[i][j].north=true; maze[i][j].south=true; maze[i][j].east=true; maze[i][j].west=true;
            maze[i][j].dirNorth=0; maze[i][j].dirSouth=0; maze[i][j].dirEast=0; maze[i][j].dirWest=0;
        }
    }
    labyrinthPose.x = 3;
    labyrinthPose.y = 3;
    labyrinthPose.cardinalDirection = DIRECTION_NORTH;
    nextDirection = DIRECTION_NORTH;
    fromDirection = 2; //initialize fromDirection with opposite of startCardinalDirection(to ensure that turn around in chooseWayDirection works)
    Pose_t initialPose = {0.0f,0.0f, M_PI_2};
    position_setPose(&initialPose);
    labyrinth_clearAllWalls();
}

bool hasEscaped(){
    return (labyrinthPose.x > 7 || labyrinthPose.x < 0  || labyrinthPose.y > 7|| labyrinthPose.y < 0 );
}

void checkWalls(uint8_t* availableDirections) {
    Walls_t walls = labyrinth_getWalls(labyrinthPose.x, labyrinthPose.y); //(for communication with HWPCS)
    *availableDirections = 0;
    if(ADC_getFilteredValue(2) < 200) { //front
        *availableDirections += 1;    
        walls.walls &= ~(1 << getCardinalDirectionfromLookingDirection(DIRECTION_NORTH)); //set wall to no wall(for communication with HWPCS); standard is wall present
        setNoWall(getCardinalDirectionfromLookingDirection(DIRECTION_NORTH));
    }
    if(ADC_getFilteredValue(0) < 200) { //right front
        *availableDirections += 1;
        walls.walls &= ~(1 << getCardinalDirectionfromLookingDirection(DIRECTION_EAST)); //set wall to no wall(for communication with HWPCS); standard is wall present
        setNoWall(getCardinalDirectionfromLookingDirection(DIRECTION_EAST));
    }
    if(ADC_getFilteredValue(3) < 200) { //left front
        *availableDirections += 1;
        walls.walls &= ~(1 << getCardinalDirectionfromLookingDirection(DIRECTION_WEST)); //set wall to no wall(for communication with HWPCS); standard is wall present
        setNoWall(getCardinalDirectionfromLookingDirection(DIRECTION_WEST));
    }
    labyrinth_setWalls(labyrinthPose.x, labyrinthPose.y, walls);
    communication_log(LEVEL_INFO, "Walls checked: %" PRIu16 " directions available (IR: front=%u, right=%u, left=%u)", 
                     *availableDirections, ADC_getFilteredValue(2), ADC_getFilteredValue(0), ADC_getFilteredValue(3));
}

bool isPlace(){
    uint8_t availableDirectionsCounter;
    checkWalls(&availableDirectionsCounter);
    return (availableDirectionsCounter >= 2);
}

Direction_t choosePlaceDirection(){

    //Place is unknown    
    if (maze[labyrinthPose.x][labyrinthPose.y].dirNorth == 0 && maze[labyrinthPose.x][labyrinthPose.y].dirSouth == 0 &&
        maze[labyrinthPose.x][labyrinthPose.y].dirEast == 0 && maze[labyrinthPose.x][labyrinthPose.y].dirWest == 0){
        //choose random direction, but not backwards
        do{
            nextDirection = (Direction_t)(rand() % 4);
        }while(oppositDirection(fromDirection) == (uint8_t)nextDirection || hasWall(nextDirection));
        communication_log(LEVEL_INFO, "Place (unknown): at (%" PRIu16 ",%" PRIu16 ") choose random dir %d", 
                         labyrinthPose.x, labyrinthPose.y, nextDirection);
        return nextDirection;
    }
    
    //Place is known but fromDir is unknown
    else if (*dirCountPtr(&maze[labyrinthPose.x][labyrinthPose.y], fromDirection) == 0){
        //turn around
        *dirCountPtr(&maze[labyrinthPose.x][labyrinthPose.y], ((fromDirection + 2) % 4)) += 1; //increment chosen direction counter
        communication_log(LEVEL_INFO, "Place (known): at (%" PRIu16 ",%" PRIu16 ") turn around (dir %d)", 
                         labyrinthPose.x, labyrinthPose.y, (fromDirection + 2) % 4);
        return nextDirection = fromDirection;
    }

    else {
        //choose least visited direction, but not backwards
        nextDirection = leastVisitedDirection();
        communication_log(LEVEL_INFO, "Place (known): at (%" PRIu16 ",%" PRIu16 ") choose least visited dir %d", 
                         labyrinthPose.x, labyrinthPose.y, nextDirection);
        return nextDirection;
    }
}

Direction_t chooseWayDirection(){

    if(hasWall(getCardinalDirectionfromLookingDirection(DIRECTION_NORTH)) && hasWall(getCardinalDirectionfromLookingDirection(DIRECTION_EAST)) 
    && hasWall(getCardinalDirectionfromLookingDirection(DIRECTION_WEST))){ //Dead End
        //turn around
        communication_log(LEVEL_INFO, "Dead end at (%" PRIu16 ",%" PRIu16 ") - turning around", 
                         labyrinthPose.x, labyrinthPose.y);
        nextDirection   = (Direction_t) fromDirection;                 
        return nextDirection; 
    }
    else{
        nextDirection = leastVisitedDirection();
        communication_log(LEVEL_INFO, "Way: at (%" PRIu16 ",%" PRIu16 ") choose dir %d", 
                         labyrinthPose.x, labyrinthPose.y, nextDirection);                     
        return nextDirection;
    }
}

uint8_t oppositDirection(uint8_t dir){
    return (dir + 2) % 4;
}

static uint8_t* dirCountPtr(Cell *c, uint8_t dir) {
    if (c == NULL) return NULL;
    switch (dir) {
        case 0: return &c->dirNorth; /* NORTH */
        case 1: return &c->dirEast;  /* EAST  */
        case 2: return &c->dirSouth; /* SOUTH */
        case 3: return &c->dirWest;  /* WEST  */
        default: return NULL;        /* should not happen */
    }
}

Direction_t getCardinalDirectionfromLookingDirection(Direction_t dirLooking) {
    Direction_t dirCardinal;
    switch (labyrinthPose.cardinalDirection) {
        case DIRECTION_NORTH:
            dirCardinal = dirLooking;
            break;
        case DIRECTION_EAST:
            dirCardinal = (Direction_t)((dirLooking + 1) % 4);
            break;
        case DIRECTION_SOUTH:
            dirCardinal = (Direction_t)((dirLooking + 2) % 4);
            break;
        case DIRECTION_WEST:
            dirCardinal = (Direction_t)((dirLooking + 3) % 4);
            break;
        default:
            dirCardinal = dirLooking; // should not happen
            break;
    }
    return dirCardinal;
}

void setNoWall(Direction_t cardinalDirection){
    switch(cardinalDirection){
        case DIRECTION_NORTH:
            maze[labyrinthPose.x][labyrinthPose.y].north = false;
            break;
        case DIRECTION_EAST:
            maze[labyrinthPose.x][labyrinthPose.y].east = false;
            break;
        case DIRECTION_SOUTH:
            maze[labyrinthPose.x][labyrinthPose.y].south = false;
            break;
        case DIRECTION_WEST:
            maze[labyrinthPose.x][labyrinthPose.y].west = false;
            break;
        default:
            break;
    }
}

bool hasWall(Direction_t cardinalDirection){
    switch(cardinalDirection){
        case DIRECTION_NORTH:
            return maze[labyrinthPose.x][labyrinthPose.y].north;
        case DIRECTION_EAST:
            return maze[labyrinthPose.x][labyrinthPose.y].east;
        case DIRECTION_SOUTH:
            return maze[labyrinthPose.x][labyrinthPose.y].south;
        case DIRECTION_WEST:
            return maze[labyrinthPose.x][labyrinthPose.y].west;
        default:
            return true;
    }
}

/* Returns the least visited direction, which is not fromDirection */
Direction_t leastVisitedDirection(){
        uint8_t min = 3; //max 
        if(fromDirection != (uint8_t)DIRECTION_NORTH && !hasWall(DIRECTION_NORTH)){
            nextDirection = DIRECTION_NORTH;
            min = maze[labyrinthPose.x][labyrinthPose.y].dirNorth;
        }
        if(maze[labyrinthPose.x][labyrinthPose.y].dirEast <= min && //smaller equal min because of !=fromDir
            fromDirection !=(uint8_t) DIRECTION_EAST && !hasWall(DIRECTION_EAST)){
            nextDirection = DIRECTION_EAST;
            min = maze[labyrinthPose.x][labyrinthPose.y].dirEast;
        }
        if(maze[labyrinthPose.x][labyrinthPose.y].dirSouth <= min &&
                fromDirection != (uint8_t)DIRECTION_SOUTH && !hasWall(DIRECTION_SOUTH)){
            nextDirection = DIRECTION_SOUTH;
            min = maze[labyrinthPose.x][labyrinthPose.y].dirSouth;
        }
        if(maze[labyrinthPose.x][labyrinthPose.y].dirWest <= min &&
            fromDirection != (uint8_t)DIRECTION_WEST && !hasWall(DIRECTION_WEST)){
            nextDirection = DIRECTION_WEST;
        }
        return nextDirection;
}

void DriveDirection(Direction_t nextDirection){
    int8_t diff = (int8_t)nextDirection - (int8_t)labyrinthPose.cardinalDirection;
    diff = (int8_t)((diff + 4) % 4);   // diff in 0..3
    if (diff == 3) diff = -1;          // 3 means -1 (left)
    // diff == 0,1,2,-1
    
    const char* dirNames[] = {"NORTH", "EAST", "SOUTH", "WEST"};
    const char* moveNames[] = {"forward", "right", "u-turn", "left"};
    const char* move = (diff == -1) ? moveNames[3] : moveNames[diff];
    
    communication_log(LEVEL_INFO, "Drive %s (dir %s, current %s)", move, dirNames[nextDirection], dirNames[labyrinthPose.cardinalDirection]);
    
    switch (diff) {
        case 0:
            // forwards
            setState(drive_Forward_distance_then_explore);
            break;
        case 1:
            // turn right 90°
            labyrinthPose.cardinalDirection = (Direction_t)((labyrinthPose.cardinalDirection + 1) % 4);
            statemachine_setTargetAngle(90); //positiv is right turn according to turn_On_Spot_degrees
            setState(turn_On_Spot_degrees_then_drive);
            break;
        case -1:
            // turn left 90°
            labyrinthPose.cardinalDirection = (Direction_t)((labyrinthPose.cardinalDirection + 3) % 4);
            statemachine_setTargetAngle(-90);
            setState(turn_On_Spot_degrees_then_drive);
            break;
        case 2:
            // U‑Turn 180°
            labyrinthPose.cardinalDirection = (Direction_t)((labyrinthPose.cardinalDirection + 2) % 4);
            statemachine_setTargetAngle(180);
            setState(turn_On_Spot_degrees_then_drive);
            break;
    }
}

// Manuelle Positionsaktualisierung nach Vorwärtsbewegung (eine Zelle)
void updateLabyrinthPosition(void) {
    switch (labyrinthPose.cardinalDirection) {
        case DIRECTION_NORTH:
            labyrinthPose.y++;
            break;
        case DIRECTION_EAST:
            labyrinthPose.x++;
            break;
        case DIRECTION_SOUTH:
            labyrinthPose.y--;
            break;
        case DIRECTION_WEST:
            labyrinthPose.x--;
            break;
    }
    
    static uint8_t lastX = 255, lastY = 255;
    if (labyrinthPose.x != lastX || labyrinthPose.y != lastY) {
        const char* dirNames[] = {"NORTH", "EAST", "SOUTH", "WEST"};
        communication_log(LEVEL_INFO, "Position updated: cell (%" PRId16 ",%" PRId16 "), facing %s", 
                         labyrinthPose.x, labyrinthPose.y, dirNames[labyrinthPose.cardinalDirection]);
        lastX = labyrinthPose.x;
        lastY = labyrinthPose.y;
    }
}

// Alte Funktion - nur noch für Debug/Logging, nicht für Navigation
void setLabyrinthPose(Pose_t pose) {
    // Nur für Debug-Logging, nicht für Navigation verwendet
    // Konvertiere Float-Werte zu Integer für Log-Ausgabe (AVR unterstützt kein Float-Format)
    int16_t x_mm = (int16_t)(pose.x);
    int16_t y_mm = (int16_t)(pose.y);
    int16_t theta_mrad = (int16_t)(pose.theta * 1000.0f);  // Theta in Milliradiant
    int16_t theta_deg = (int16_t)(pose.theta * 180.0f / M_PI);  // Theta in Grad
    communication_log(LEVEL_INFO, "Odometrie (nur Debug): x=%" PRId16 "mm y=%" PRId16 "mm theta=%" PRId16 "° (%" PRId16 "mrad)", 
                     x_mm, y_mm, theta_deg, theta_mrad);
}