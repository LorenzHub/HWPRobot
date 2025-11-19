#include "exploration.h"
#include <tools/labyrinth/labyrinth.h>  // Labyrinth library for wall storage
#include "ir_sensors.h"
#include "statemachine.h" // For controlling robot movement
#include "communication/communication.h"
#include "position.h"  // For camera pose access
#include <stddef.h>
#include <avr/pgmspace.h>
#include <math.h>
#include <inttypes.h>

// Grid dimensions
#define ROWS LABYRINTH_ROWS
#define COLS LABYRINTH_COLS

// Cell size in mm (including wall thickness)
#define CELL_SIZE_MM 253.3f

// Origin is at center of middle cell (row 3, col 3)
#define ORIGIN_ROW 3
#define ORIGIN_COL 3

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

// Static variables for position correction
static uint8_t correction_initialized = 0;
static uint8_t angle_correction_initialized = 0;
static uint8_t move_correction_initialized = 0;

// Helper function: Calculate cell center coordinates in continuous space
// row, col: 0-based cell indices
// center_x, center_y: Output parameters for cell center coordinates in mm
static void calculateCellCenter(uint8_t row, uint8_t col, float* center_x, float* center_y) {
    // Origin is at (3,3) = (0mm, 0mm)
    // x-axis: right (col increases -> x increases)
    // y-axis: up (row 0 is top, row increases -> y decreases)
    *center_x = ((float)col - (float)ORIGIN_COL) * CELL_SIZE_MM;
    *center_y = ((float)ORIGIN_ROW - (float)row) * CELL_SIZE_MM;  // y-axis inverted (row 0 = top)
}

// Helper function: Calculate position correction needed to reach cell center
// Returns true if correction is needed (distance > threshold)
// correction_x, correction_y: Output parameters for correction vector in mm
static bool calculatePositionCorrection(const Pose_t* currentPose, uint8_t row, uint8_t col, 
                                        float* correction_x, float* correction_y) {
    float center_x, center_y;
    calculateCellCenter(row, col, &center_x, &center_y);
    
    *correction_x = center_x - currentPose->x;
    *correction_y = center_y - currentPose->y;
    
    // Check if correction is needed (threshold: 10mm)
    float distance = sqrtf((*correction_x) * (*correction_x) + (*correction_y) * (*correction_y));
    return (distance > 10.0f);
}

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
            expl_state = EXPLORE_CORRECT_POSITION; // Correct position before checking walls
            break;
            
        case EXPLORE_CORRECT_POSITION: {
            // Camera-based position correction to center in cell
            if (!position_hasValidCameraPose()) {
                // No camera pose available - skip correction
                correction_initialized = 0;
                angle_correction_initialized = 0;
                move_correction_initialized = 0;
                expl_state = EXPLORE_CHECK_WALLS;
                break;
            }
            
            // Get current pose (already corrected by position_applyCorrection)
            const Pose_t* currentPose = position_getCurrentPose();
            
            // Calculate correction needed
            float correction_x, correction_y;
            bool needsCorrection = calculatePositionCorrection(currentPose, cur_row, cur_col, 
                                                               &correction_x, &correction_y);
            
            if (!needsCorrection) {
                // Already centered - continue
                correction_initialized = 0;
                angle_correction_initialized = 0;
                move_correction_initialized = 0;
                expl_state = EXPLORE_CHECK_WALLS;
                break;
            }
            
            if (!correction_initialized) {
                // Initialize correction
                // Calculate distance and angle to cell center
                float distance = sqrtf(correction_x * correction_x + correction_y * correction_y);
                float angle = atan2f(correction_y, correction_x);
                
                // Limit correction distance (max 50mm per step to avoid large jumps)
                if (distance > 50.0f) {
                    distance = 50.0f;
                }
                
                // Calculate angle difference
                float angleDiff = angle - currentPose->theta;
                // Normalize to [-π, π]
                while (angleDiff > M_PI) angleDiff -= 2.0f * M_PI;
                while (angleDiff < -M_PI) angleDiff += 2.0f * M_PI;
                
                correction_initialized = 1;
                angle_correction_initialized = 0;  // Reset for next correction step
                move_correction_initialized = 0;
                
                // Check if angle correction is needed first (> 5° = 0.087 rad)
                if ((angleDiff > 0.087f) || (angleDiff < -0.087f)) {
                    // Turn first
                    expl_state = EXPLORE_CORRECT_ANGLE;
                } else {
                    // Angle is OK - do position correction directly
                    expl_state = EXPLORE_CORRECT_MOVE;
                }
            }
            break;
        }
        
        case EXPLORE_CORRECT_ANGLE: {
            static int16_t target_angle_deg = 0;
            
            if (!angle_correction_initialized) {
                // Get target angle from current pose
                const Pose_t* currentPose = position_getCurrentPose();
                float correction_x, correction_y;
                calculatePositionCorrection(currentPose, cur_row, cur_col, &correction_x, &correction_y);
                float angle = atan2f(correction_y, correction_x);
                float angleDiff = angle - currentPose->theta;
                // Normalize to [-π, π]
                while (angleDiff > M_PI) angleDiff -= 2.0f * M_PI;
                while (angleDiff < -M_PI) angleDiff += 2.0f * M_PI;
                
                target_angle_deg = (int16_t)(angleDiff * 180.0f / M_PI);
                angle_correction_initialized = 1;
            }
            
            // Execute turn
            if (turn_On_Spot_degrees(target_angle_deg, 3000)) {
                // Turn completed
                angle_correction_initialized = 0;
                move_correction_initialized = 0;  // Reset for next step
                expl_state = EXPLORE_CORRECT_MOVE;
            }
            break;
        }
        
        case EXPLORE_CORRECT_MOVE: {
            static uint16_t target_dist = 0;
            
            if (!move_correction_initialized) {
                // Get target distance from current pose
                const Pose_t* currentPose = position_getCurrentPose();
                float correction_x, correction_y;
                calculatePositionCorrection(currentPose, cur_row, cur_col, &correction_x, &correction_y);
                float distance = sqrtf(correction_x * correction_x + correction_y * correction_y);
                
                // Limit correction distance (max 50mm per step)
                if (distance > 50.0f) {
                    distance = 50.0f;
                }
                
                target_dist = (uint16_t)distance;
                move_correction_initialized = 1;
            }
            
            // Execute position correction
            if (drive_Forward_distance_mm(target_dist, 3000)) {
                // Correction completed
                move_correction_initialized = 0;
                // Re-check if more correction is needed (with hysteresis: only if > 10mm)
                const Pose_t* currentPose = position_getCurrentPose();
                float correction_x, correction_y;
                bool stillNeedsCorrection = calculatePositionCorrection(currentPose, cur_row, cur_col, 
                                                                        &correction_x, &correction_y);
                if (stillNeedsCorrection) {
                    // More correction needed - go back to start (reset initialization)
                    correction_initialized = 0;  // Reset to re-initialize
                    expl_state = EXPLORE_CORRECT_POSITION;
                } else {
                    // Correction complete - reset all correction states
                    correction_initialized = 0;
                    angle_correction_initialized = 0;
                    move_correction_initialized = 0;
                    expl_state = EXPLORE_CHECK_WALLS;
                }
            }
            break;
        }
        
        case EXPLORE_FINISHED:
             Motor_stopAll();
             break;
    }
}

// Test function: Mark all cell centers in HWPCS
void exploration_markAllCellCenters(void) {
    communication_log(LEVEL_INFO, "Markiere alle Zell-Mitten...");
    
    for (uint8_t row = 0; row < ROWS; row++) {
        for (uint8_t col = 0; col < COLS; col++) {
            float center_x, center_y;
            calculateCellCenter(row, col, &center_x, &center_y);
            
            // Send cell info with marker value 100
            LabyrinthCellInfo_t cellInfo;
            cellInfo.row = row;
            cellInfo.col = col;
            cellInfo.info = 100;  // Marker for cell center
            
            communication_writePacket(CH_OUT_LABY_CELL_INFO, (uint8_t*)&cellInfo, sizeof(cellInfo));
            
            // Log coordinates for debugging
            int16_t x_mm = (int16_t)center_x;
            int16_t y_mm = (int16_t)center_y;
            communication_log(LEVEL_FINE, "Zelle (%" PRIu8 ",%" PRIu8 "): Mitte bei (%d, %d) mm", 
                             row, col, x_mm, y_mm);
        }
    }
    
    communication_log(LEVEL_INFO, "Alle 49 Zell-Mitten markiert (info=100)");
}

// Test function: Clear all cell markers in HWPCS
void exploration_clearAllCellMarkers(void) {
    communication_log(LEVEL_INFO, "Lösche alle Zell-Markierungen...");
    
    for (uint8_t row = 0; row < ROWS; row++) {
        for (uint8_t col = 0; col < COLS; col++) {
            LabyrinthCellInfo_t cellInfo;
            cellInfo.row = row;
            cellInfo.col = col;
            cellInfo.info = -128;  // Clear marker
            
            communication_writePacket(CH_OUT_LABY_CELL_INFO, (uint8_t*)&cellInfo, sizeof(cellInfo));
        }
    }
    
    communication_log(LEVEL_INFO, "Alle Zell-Markierungen gelöscht");
}
