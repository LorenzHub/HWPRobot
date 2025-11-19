#include "exploration.h"
#include <tools/labyrinth/labyrinth.h>  // Labyrinth library for wall storage
#include "ir_sensors.h"
#include "statemachine.h" // For controlling robot movement
#include "communication/communication.h"
#include <stddef.h>
#include <avr/pgmspace.h>

// Grid dimensions
#define ROWS LABYRINTH_ROWS
#define COLS LABYRINTH_COLS

// Data structures
static uint8_t visit_count[ROWS][COLS];
static uint8_t cur_row = 0;
static uint8_t cur_col = 0;
static Direction_t cur_dir = DIRECTION_NORTH;

static ExplorationState_t expl_state = EXPLORE_IDLE;

// Helper to get cell in front based on current pos and dir
// Returns true if valid, false if out of bounds
static bool get_front_neighbor(uint8_t r, uint8_t c, Direction_t d, uint8_t* nr, uint8_t* nc) {
    int16_t tr = r;
    int16_t tc = c;
    
    switch (d) {
        case DIRECTION_NORTH: tr++; break;
        case DIRECTION_EAST:  tc++; break;
        case DIRECTION_SOUTH: tr--; break;
        case DIRECTION_WEST:  tc--; break;
    }

    if (tr >= 0 && tr < ROWS && tc >= 0 && tc < COLS) {
        *nr = (uint8_t)tr;
        *nc = (uint8_t)tc;
        return true;
    }
    return false;
}

void exploration_init(void) {
    // Reset visit counts
    for (uint8_t r = 0; r < ROWS; r++) {
        for (uint8_t c = 0; c < COLS; c++) {
            visit_count[r][c] = 0;
        }
    }
    
    // Start at 0,0 facing North
    cur_row = 0;
    cur_col = 0;
    cur_dir = DIRECTION_NORTH;
    
    // Mark start as visited
    visit_count[0][0] = 1;
    
    expl_state = EXPLORE_IDLE;
}

ExplorationState_t exploration_get_state(void) {
    return expl_state;
}

// Static variables for movement state
static int16_t target_angle = 0;
static int16_t target_ticks = 0;

void exploration_step(void) {
    switch (expl_state) {
        case EXPLORE_IDLE:
            // Transition to start
            expl_state = EXPLORE_CHECK_WALLS;
            break;

        case EXPLORE_CHECK_WALLS: {
            // Read IR sensors and update walls for current cell
            Walls_t walls = labyrinth_getWalls(cur_row, cur_col);
            
            // Front Sensor (2)
            if (ir_sensor_detects_wall(2)) {
                switch (cur_dir) {
                    case DIRECTION_NORTH: walls.wall.north = WALLSTATE_SET; break;
                    case DIRECTION_EAST:  walls.wall.east = WALLSTATE_SET; break;
                    case DIRECTION_SOUTH: walls.wall.south = WALLSTATE_SET; break;
                    case DIRECTION_WEST:  walls.wall.west = WALLSTATE_SET; break;
                }
            }
            
            // Right Sensor (0) - Assuming Right Front
            if (ir_sensor_detects_wall(0)) {
                Direction_t right_dir = (cur_dir + 1) % 4;
                 switch (right_dir) {
                    case DIRECTION_NORTH: walls.wall.north = WALLSTATE_SET; break;
                    case DIRECTION_EAST:  walls.wall.east = WALLSTATE_SET; break;
                    case DIRECTION_SOUTH: walls.wall.south = WALLSTATE_SET; break;
                    case DIRECTION_WEST:  walls.wall.west = WALLSTATE_SET; break;
                }
            }
            
            // Left Sensor (3) - Assuming Left Front
            if (ir_sensor_detects_wall(3)) {
                 Direction_t left_dir = (cur_dir + 3) % 4; 
                 switch (left_dir) {
                    case DIRECTION_NORTH: walls.wall.north = WALLSTATE_SET; break;
                    case DIRECTION_EAST:  walls.wall.east = WALLSTATE_SET; break;
                    case DIRECTION_SOUTH: walls.wall.south = WALLSTATE_SET; break;
                    case DIRECTION_WEST:  walls.wall.west = WALLSTATE_SET; break;
                }
            }
            
            labyrinth_setWalls(cur_row, cur_col, walls);
            expl_state = EXPLORE_DECIDE;
            break;
        }

        case EXPLORE_DECIDE: {
            // Least Visited Neighbor Strategy
            uint8_t min_visits = 255;
            Direction_t best_dir = cur_dir;
            bool found_move = false;
            
            // Priority: Front, Right, Left, Back
            Direction_t check_order[4] = {
                cur_dir,
                (Direction_t)((cur_dir + 1) % 4),
                (Direction_t)((cur_dir + 3) % 4),
                (Direction_t)((cur_dir + 2) % 4)
            };
            
            Walls_t walls = labyrinth_getWalls(cur_row, cur_col);
            
            for (int i = 0; i < 4; i++) {
                Direction_t d = check_order[i];
                
                // Check Wall
                bool blocked = false;
                switch (d) {
                    case DIRECTION_NORTH: blocked = (walls.wall.north == WALLSTATE_SET); break;
                    case DIRECTION_EAST:  blocked = (walls.wall.east == WALLSTATE_SET); break;
                    case DIRECTION_SOUTH: blocked = (walls.wall.south == WALLSTATE_SET); break;
                    case DIRECTION_WEST:  blocked = (walls.wall.west == WALLSTATE_SET); break;
                }
                
                if (blocked) continue;
                
                // Check Neighbor
                uint8_t nr, nc;
                if (get_front_neighbor(cur_row, cur_col, d, &nr, &nc)) {
                    uint8_t v = visit_count[nr][nc];
                    if (v < min_visits) {
                        min_visits = v;
                        best_dir = d;
                        found_move = true;
                    }
                }
            }
            
            if (found_move) {
                // Calculate turn needed
                int diff = (int)best_dir - (int)cur_dir;
                if (diff == 3) diff = -1;
                if (diff == -3) diff = 1;
                if (diff == 2 || diff == -2) diff = 2; // 180 deg
                
                if (diff == 0) {
                    // Straight
                    expl_state = EXPLORE_MOVE;
                } else {
                    // Turn required
                    if (diff == 1) target_angle = -90; // Right turn is negative in statemachine?
                    // Wait, statemachine.c: turn_On_Spot_degrees:
                    // "Bestimme Drehrichtung: positiv = links drehen"
                    // So Left = +90, Right = -90. Correct.
                    
                    if (diff == 1) target_angle = -90; // Right
                    else if (diff == -1) target_angle = 90; // Left
                    else target_angle = 180; // Back
                    
                    expl_state = EXPLORE_TURN;
                }
                cur_dir = best_dir; // Update direction
            } else {
                expl_state = EXPLORE_FINISHED;
            }
            break;
        }

        case EXPLORE_TURN:
            // Execute Turn
            if (turn_On_Spot_degrees(target_angle, 4000)) {
                expl_state = EXPLORE_MOVE;
            }
            break;
        
        case EXPLORE_WAIT_TURN:
            // Unused if we call turn_On_Spot_degrees repeatedly
            break;

        case EXPLORE_MOVE:
            // Calculate Ticks for 253.3mm
            // 1 Tick = 0.0688 mm => 253.3 / 0.0688 = 3681 Ticks
            target_ticks = 3681;
            expl_state = EXPLORE_WAIT_MOVE;
            // Fallthrough to start moving immediately
            // Intentional fallthrough? No, break to be safe and start next cycle
            // Actually we can just call drive_Forward_ticks here too.
            // Let's switch state to WAIT_MOVE and call it there.
            break;

        case EXPLORE_WAIT_MOVE:
             if (drive_Forward_ticks(target_ticks, 4000)) {
                 expl_state = EXPLORE_UPDATE_CELL;
             }
             break;

        case EXPLORE_UPDATE_CELL:
            // Update position
            {
                uint8_t nr, nc;
                if (get_front_neighbor(cur_row, cur_col, cur_dir, &nr, &nc)) {
                    cur_row = nr;
                    cur_col = nc;
                    if (visit_count[cur_row][cur_col] < 255) {
                        visit_count[cur_row][cur_col]++;
                    }
                    
                    // Send Info
                    LabyrinthCellInfo_t cellInfo;
                    cellInfo.row = cur_row;
                    cellInfo.col = cur_col;
                    cellInfo.info = visit_count[cur_row][cur_col];
                    communication_writePacket(CH_OUT_LABY_CELL_INFO, (uint8_t*)&cellInfo, sizeof(cellInfo));
                }
            }
            expl_state = EXPLORE_CHECK_WALLS; // Loop
            break;
            
        case EXPLORE_FINISHED:
             Motor_stopAll();
             break;
    }
}
