#ifndef EXPLORATION_H_
#define EXPLORATION_H_

#include <stdint.h>
#include <stdbool.h>
#include <communication/packetTypes.h>

// Exploration Sub-States
typedef enum {
    EXPLORE_IDLE,
    EXPLORE_CHECK_WALLS,
    EXPLORE_DECIDE,
    EXPLORE_TURN,
    EXPLORE_WAIT_TURN,
    EXPLORE_MOVE,
    EXPLORE_WAIT_MOVE,
    EXPLORE_UPDATE_CELL,
    EXPLORE_FINISHED
} ExplorationState_t;

/**
 * Initialize the exploration module.
 * Resets visit counts and sets start position.
 */
void exploration_init(void);

/**
 * Execute one step of the exploration logic.
 * Should be called periodically when the main state machine is in ExploreMaze.
 */
void exploration_step(void);

/**
 * Get current exploration state (for debugging).
 */
ExplorationState_t exploration_get_state(void);

#endif /* EXPLORATION_H_ */

