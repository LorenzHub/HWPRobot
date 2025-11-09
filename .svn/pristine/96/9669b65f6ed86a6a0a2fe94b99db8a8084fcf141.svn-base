/**
 * @file labyrinth.h
 * @ingroup tools
 *
 * Storage and manipulation functions for labyrinth walls.
 */

#ifndef LABYRINTH_H_
#define LABYRINTH_H_

#include <communication/packetTypes.h>

#include <stdint.h>


/**
 * Side length of the interior of a labyrinth cell (excluding the thickness of one wall) in mm
 */
#define LABY_INNER_CELLSIZE (250.0f)


/**
 * Side length of half the interior of a labyrinth cell (excluding the thickness of one wall) in mm
 */
#define LABY_INNER_CELLSIZE_2 (LABY_INNER_CELLSIZE / 2.0f)


/**
 * Side length of a labyrinth cell (including the thickness of one wall) in mm
 */
#define LABY_CELLSIZE (LABY_INNER_CELLSIZE + 3.3f)


/**
 * Side length of half a labyrinth cell (including the thickness of one wall) in mm
 */
#define LABY_CELLSIZE_2 (LABY_CELLSIZE / 2.0f)


/**
 * Enumeration type of wall states
 */
typedef enum {
    WALLSTATE_CLEARED = 0, ///< Wall is not present
    WALLSTATE_SET = 1,  ///< Wall is present
} WallState_t;


/**
 * State of all walls surrounding a cell.
 * This is used by labyrinth_setWalls() and labyrinth_getWalls().
 */
typedef union {
    struct {
        WallState_t north : 1; ///< State of wall in Northern direction
        WallState_t east : 1; ///< State of wall in Eastern direction
        WallState_t south : 1; ///< State of wall in Southern direction
        WallState_t west : 1; ///< State of wall in Western direction
    } wall; ///< for accessing the state of each wall individually

    uint8_t walls : 4; ///< for accessing the state of all walls simultaneously
} Walls_t;


/**
 * Initialize the labyrinth library.
 * Must be invoked before any other library function.
 * This function also clears the wall states which are kept in an internal data structure
 * of type LabyrinthWalls_t which contains the states of all walls for all cells.
 */
void labyrinth_init(void);


/**
 * Compute an integer number as a bijective mapping: (row,col)->cell number.
 *
 * Cell numbering is assumed to proceed row-wise with increasing column index.
 * E.g. for a labyrinth with 7 rows and columns:
 * (0,0)->0, (0,1)->1, (0,4)->4, ... (0,6)->6,
 * (1,0)->7, (1,1)->8, (1,4)->11, ... (1,6)->13, ...
 *
 * This function can for example be used to index an appropriate data structure
 * for determination of visited cells.
 *
 * @param   row   the row number of the cell
 * @param   col   the column number of the cell
 * @return  integer number representing the index of the corresponding cell
 */
uint16_t labyrinth_getCellIndex(const uint8_t row, const uint8_t col);


/**
 * Compute an integer number as a surjective mapping: (row,col,dir)->wall number.
 * Walls are numbered in the following order:
 * - The first (#LABYRINTH_COLS + 1) * #LABYRINTH_ROWS walls represent Western and
 *   Eastern walls of the cells in row0, row1, ...
 * - The last (#LABYRINTH_ROWS + 1) * #LABYRINTH_COLS walls represent Northern and
 *   Southern walls of the cells in col0, col1, ...
 *
 * This function is used by labyrinth_getWalls() and labyrinth_setWalls()
 * for computing the wall number from which an index into the field walls of
 * LabyrinthWalls_t can be computed. See LabyrinthWalls_t for further details
 * on the storage of walls.
 *
 * @param   row   the row number of the cell
 * @param   col   the column number of the cell
 * @param   dir   the direction for identifying the wall of the specified cell
 * @return  integer number representing the index of the corresponding wall.
 *          If specified cell is outside of the labyrinth, 0xFFFF is returned.
 */
uint16_t labyrinth_getWallIndex(const uint8_t row, const uint8_t col, const Direction_t dir);


/**
 * Get states of all surrounding walls of the specified cell.
 * An internal data structure of type LabyrinthWalls_t is read by this function.
 * It contains the states of all walls which can be acquired by labyrinth_getAllWalls().
 *
 * @param   row   the row number of the cell
 * @param   col   the column number of the cell
 * @return  the states of all surrounding walls of the specified cell
 */
Walls_t labyrinth_getWalls(const uint8_t row, const uint8_t col);


/**
 * Set states of all surrounding walls of the specified cell.
 * An internal data structure of type LabyrinthWalls_t is modified by this function.
 * It contains the states of all walls which can be acquired by labyrinth_getAllWalls().
 *
 * @param   row   the row number of the cell
 * @param   col   the column number of the cell
 * @param   walls the new wall states for the specified cell
 */
void labyrinth_setWalls(const uint8_t row, const uint8_t col, const Walls_t walls);


/**
 * Clear the states of all walls for all cells.
 * An internal data structure of type LabyrinthWalls_t is modified by this function.
 * It contains the states of all walls which can be acquired by labyrinth_getAllWalls().
 */
void labyrinth_clearAllWalls(void);


/**
 * Get the stored walls of all cells as LabyrinthWalls_t.
 * The internally managed LabyrinthWalls_t can be updated by labyrinth_setWalls()
 * and read by labyrinth_getWalls() for a specific cell.
 *
 * Send the return value of this function on channel #CH_OUT_LABY_WALLS for visualization
 * of labyrinth walls in HWPCS.
 *
 * @return  a pointer to the internal structure of type LabyrinthWalls_t which contains all
 *          walls for all cells
 */
const LabyrinthWalls_t* labyrinth_getAllWalls(void);

#endif /* LABYRINTH_H_ */
