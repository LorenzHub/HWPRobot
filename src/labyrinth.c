#include "labyrinth.h"
#include <math.h>
#include <communication/communication.h>
#include <inttypes.h>
#include <stdbool.h>

/*exploreMaze <-> Statemachine*/

LabyrinthPose_t labyrinthPose = {4,4,1};

Cell maze[7][7]; 

// Hilfsfunktion: Prüfe ob eine Position innerhalb des Labyrinths liegt
static bool isValidPosition(uint8_t x, uint8_t y) {
    return (x < 7) && (y < 7);
}

// Hilfsfunktion: Gebe Richtung als String aus
static const char* directionToString(uint8_t dir) {
    switch(dir) {
        case DIRECTION_NORTH: return "NORTH";
        case DIRECTION_EAST: return "EAST";
        case DIRECTION_SOUTH: return "SOUTH";
        case DIRECTION_WEST: return "WEST";
        default: return "UNKNOWN";
    }
}

// Hilfsfunktion: Gebe nächste Zelle in einer Richtung zurück
static bool getNextCell(uint8_t x, uint8_t y, uint8_t direction, uint8_t* nextX, uint8_t* nextY) {
    switch(direction) {
        case DIRECTION_NORTH:
            if (y > 0) { *nextX = x; *nextY = y - 1; return true; }
            break;
        case DIRECTION_SOUTH:
            if (y < 6) { *nextX = x; *nextY = y + 1; return true; }
            break;
        case DIRECTION_EAST:
            if (x < 6) { *nextX = x + 1; *nextY = y; return true; }
            break;
        case DIRECTION_WEST:
            if (x > 0) { *nextX = x - 1; *nextY = y; return true; }
            break;
    }
    return false;
}

// Hilfsfunktion: Gebe direkt entgegengesetzte Richtung zurück
static uint8_t oppositeDirection(uint8_t direction) {
    switch(direction) {
        case DIRECTION_NORTH: return DIRECTION_SOUTH;
        case DIRECTION_SOUTH: return DIRECTION_NORTH;
        case DIRECTION_EAST: return DIRECTION_WEST;
        case DIRECTION_WEST: return DIRECTION_EAST;
        default: return direction;
    }
}

// Hilfsfunktion: Prüfe ob Wand in einer Richtung existiert
static bool hasWall(uint8_t x, uint8_t y, uint8_t direction) {
    if (!isValidPosition(x, y)) return true;  // Außerhalb = Wand
    
    Cell* cell = &maze[y][x];
    switch(direction) {
        case DIRECTION_NORTH: return cell->north;
        case DIRECTION_SOUTH: return cell->south;
        case DIRECTION_EAST: return cell->east;
        case DIRECTION_WEST: return cell->west;
        default: return true;
    }
}

// Hilfsfunktion: Wähle nächste Richtung nach Trémaux-Algorithmus
static uint8_t chooseDirecion(uint8_t x, uint8_t y, uint8_t currentDir, uint8_t fromDir) {
    const uint8_t directions[4] = {DIRECTION_NORTH, DIRECTION_EAST, DIRECTION_SOUTH, DIRECTION_WEST};
    
    // Priorität 1: Nicht besuchte Wege
    for (int i = 0; i < 4; i++) {
        uint8_t dir = directions[i];
        if (dir == fromDir) continue;  // Nicht zurück gehen
        
        if (!hasWall(x, y, dir)) {
            uint8_t nextX, nextY;
            if (getNextCell(x, y, dir, &nextX, &nextY)) {
                if (maze[nextY][nextX].visited == 0) {
                    return dir;  // Nicht besuchter Weg
                }
            }
        }
    }
    
    // Priorität 2: Wege mit nur einer Markierung (visited == 1)
    for (int i = 0; i < 4; i++) {
        uint8_t dir = directions[i];
        if (dir == fromDir) continue;
        
        if (!hasWall(x, y, dir)) {
            uint8_t nextX, nextY;
            if (getNextCell(x, y, dir, &nextX, &nextY)) {
                if (maze[nextY][nextX].visited == 1) {
                    return dir;  // Weg mit einer Markierung
                }
            }
        }
    }
    
    // Priorität 3: Zurück gehen (wenn nötig)
    if (fromDir != 0) {
        return fromDir;
    }
    
    // Fallback: Beliebige offene Richtung
    for (int i = 0; i < 4; i++) {
        uint8_t dir = directions[i];
        if (!hasWall(x, y, dir)) {
            return dir;
        }
    }
    
    return currentDir;  // Gleiche Richtung halten wenn nichts verfügbar
}

void exploreMaze(void) {
    static uint8_t initialized = 0;
    static uint8_t x = 4, y = 4;  // Start-Position
    static uint8_t previousX = 4, previousY = 4;
    static uint8_t previousDir = 0;
    
    if (!initialized) {
        // Initialisiere Labyrinth
        for (uint8_t i = 0; i < 7; i++) {
            for (uint8_t j = 0; j < 7; j++) {
                maze[i][j].visited = 0;
                maze[i][j].north = false;
                maze[i][j].south = false;
                maze[i][j].east = false;
                maze[i][j].west = false;
            }
        }
        
        // Starte bei Position (4, 4) mit Markierung 1
        maze[4][4].visited = 1;
        x = 4;
        y = 4;
        previousX = 4;
        previousY = 4;
        previousDir = 0;
        initialized = 1;
        
        communication_log(LEVEL_INFO, "Labyrinth-Erkundung gestartet bei Position (%" PRIu8 ", %" PRIu8 ")", x, y);
    }
    
    // Wende Trémaux-Algorithmus an
    uint8_t currentDir = labyrinthPose.cardinalDirection;
    uint8_t fromDir = oppositeDirection(previousDir);  // Richtung aus der wir kamen
    
    // Wähle nächste Richtung nach Trémaux-Regeln
    uint8_t nextDir = chooseDirecion(x, y, currentDir, fromDir);
    
    // Prüfe ob Wand in gewählter Richtung
    if (!hasWall(x, y, nextDir)) {
        uint8_t nextX, nextY;
        if (getNextCell(x, y, nextDir, &nextX, &nextY) && isValidPosition(nextX, nextY)) {
            // Bewegung möglich
            previousX = x;
            previousY = y;
            x = nextX;
            y = nextY;
            previousDir = nextDir;
            
            // Inkrementiere Besuchs-Zähler
            if (maze[y][x].visited == 0) {
                maze[y][x].visited = 1;  // Erste Markierung
                communication_log(LEVEL_INFO, "Neue Zelle gefunden: (%" PRIu8 ", %" PRIu8 ")", x, y);
            } else if (maze[y][x].visited == 1) {
                maze[y][x].visited = 2;  // Zweite Markierung
                communication_log(LEVEL_INFO, "Zelle erneut besucht (2x): (%" PRIu8 ", %" PRIu8 ")", x, y);
            }
            
            communication_log(LEVEL_FINE, "Bewegung: (%" PRIu8 ", %" PRIu8 ") -> (%" PRIu8 ", %" PRIu8 ") [%s], Besuch=%d", 
                             previousX, previousY, x, y, directionToString(nextDir), maze[y][x].visited);
        }
    } else {
        communication_log(LEVEL_WARNING, "Wand in Richtung %s - Alternative suchen", directionToString(nextDir));
    }
}

void setLabyrinthPose(Pose_t pose) {
    labyrinthPose.x = (uint8_t)(pose.x / 256.9f)+4.0f; //Cell size 256.9mm with wall
    labyrinthPose.y = (uint8_t)(pose.y / 256.9f)+4.0f;

    float t = pose.theta + M_PI_4; //range
	t = fmodf(t, 2.0f * M_PI);
	if (t < 0.0f)
		t += 2.0f * M_PI;

	int idx = (int) floorf(t / (M_PI_2));
	const Direction_t map[4] = { DIRECTION_EAST,DIRECTION_NORTH, DIRECTION_WEST, DIRECTION_SOUTH };
	labyrinthPose.cardinalDirection = map[idx & 0x3];
}