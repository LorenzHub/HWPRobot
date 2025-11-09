/**
 * @file packetTypes.h
 * @ingroup communication
 *
 * Packet types used for communication between robot and HWPCS
 */

#ifndef PACKETTYPES_H_
#define PACKETTYPES_H_

#include <tools/variablesAccess.h>

#include <stdint.h>
#include <stdbool.h>


/**
 * Number of rows in labyrinth
 */
#define LABYRINTH_ROWS 7


/**
 * Number of columns in labyrinth
 */
#define LABYRINTH_COLS 7


/**
 * Enumeration type of directions in labyrinth
 */
typedef enum {
    DIRECTION_NORTH = 0,  ///< Northern direction
    DIRECTION_EAST = 1,   ///< Eastern direction
    DIRECTION_SOUTH = 2,  ///< Southern direction
    DIRECTION_WEST = 3    ///< Western direction
} Direction_t;



/**
 * Enumeration type for communication channels.
 *
 * These values are intended to be used by communication_writePacket(), communication_setCallback() and communication_clearCallback().
 */
typedef enum {
    CH_IN_DEBUG = 0x00, ///< Debug channel, HWPCS->Robot, used by Debug View in HWPCS
    CH_OUT_DEBUG = 0x00, ///< Debug channel, Robot->HWPCS, used by communication_log()
    CH_IN_DRIVE = 0x01, ///< for receiving DriveCommand_t from Control View in HWPCS
    CH_OUT_TELEMETRY = 0x01, ///< for sending Telemetry_t to Telemetry View in HWPCS
    CH_IN_ROBOT_PARAMS = 0x02, ///< for receiving RobotParameters_t from Scene View in HWPCS
    CH_OUT_POSE = 0x02, ///< for sending Pose_t to Scene View in HWPCS
    CH_IN_PATH_FOLLOW_CTRL = 0x03, ///< for receiving PathFollowerControl_t from Scene View in HWPCS
    CH_OUT_PATH_FOLLOW_STATUS = 0x03, ///< for sending PathFollowerStatus_t to Scene View in HWPCS
    CH_IN_PATH_FOLLOW_PARAMS = 0x04, ///< for receiving PathFollowerParameters_t from Scene View in HWPCS
    CH_OUT_LABY_WALLS = 0x04, ///< for sending LabyrinthWalls_t to Scene View in HWPCS
    CH_IN_POSE = 0x05, ///< ///< for receiving Pose_t of main AprilTag from HWPCS
    CH_OUT_GET_POSE = 0x05, ///< for sending GetPose_t to HWPCS (request to get tracked pose of main or additional AprilTag)
    CH_IN_USER_COMMAND = 0x06, ///< for receiving UserCommand_t from User Command View in HWPCS
    CH_OUT_USER_DATA = 0x06, ///< for sending UserData_t to User Data View in HWPCS
    CH_OUT_RDP = 0x07, ///< for sending remote data processing command to RDP View in HWPCS
    CH_IN_ADDITIONAL_POSE = 0x08, ///< for receiving Pose_t of additional AprilTag from HWPCS
    CH_OUT_LABY_CELL_INFO = 0x08, ///< for sending LabyrinthCellInfo_t to be displayed in Scene View in HWPCS
    CH_OUT_LABY_WALL_INFO = 0x09 ///< for sending LabyrinthWallInfo_t to be displayed in Scene View in HWPCS
} Channel_t;


/**
 * Telemetry data of robot.
 *
 * - sent on channel #CH_OUT_TELEMETRY (0x01)
 * - size: 22 Bytes
 * - minimum transmission duration (incl. minimum overhead of 5 bytes): 0.54ms
 */
typedef struct __attribute__((__packed__)) {
    bitset8_t bumpers;   ///< bitset containing onset of 5 bumpers in the lowest 5 significant bits
    uint8_t contacts;    ///< number of registered bumper contacts
    int16_t encoder1;    ///< encoder 1 output measured in tics
    int16_t encoder2;    ///< encoder 2 output measured in tics
    uint16_t infrared1;  ///< distance of infrared sensor 1 measured in mm
    uint16_t infrared2;  ///< distance of infrared sensor 2 measured in mm
    uint16_t infrared3;  ///< distance of infrared sensor 3 measured in mm
    uint16_t infrared4;  ///< distance of infrared sensor 4 measured in mm
    uint16_t infrared5;  ///< distance of infrared sensor 5 measured in mm
    int16_t user1;       ///< user defined value 1 (int16_t)
    float user2;         ///< user defined value 2 (float)
} Telemetry_t;


/**
 * Drive command for robot.
 *
 * This packet is sent from HWPCS from the control input view.
 *
 * - received on channel #CH_IN_DRIVE (0x01)
 * - size: 4 Bytes
 * - minimum transmission duration (incl. minimum overhead of 5 bytes): 0.18ms
 */
typedef struct __attribute__((__packed__)) {
    int16_t speed;     ///< directional speed in [-8191;8191]
    int16_t steering;  ///< steering direction in [-8191;8191]
} DriveCommand_t;


/**
 * Pose of robot.
 *
 * - sent on channel #CH_OUT_POSE (0x02)
 * - received on channel #CH_IN_POSE (0x05), see GetPose_t
 * - size: 12 Bytes
 * - minimum transmission duration (incl. minimum overhead of 5 bytes): 0.34ms
 */
typedef struct __attribute__((__packed__)) {
    float x;      ///< x coordinate of robot in global frame measured in mm
    float y;      ///< y coordinate of robot in global frame measured in mm
    float theta;  ///< orientation angle of robot measured in radians,
                  ///< defined as the counter-clockwise angle between the global x-axis
                  ///< and robot's forward direction, i.e.
                  ///<   - theta = 0    => robot points in direction of global x-axis (east)
                  ///<   - theta = pi/2 => robot points in direction of global y-axis (north)
} Pose_t;


/**
 * Robot parameters.
 * Contains several robot parameters which can be easily changed from within HWPCS.
 * The information can be used for odometry calculation or user defined purposes.
 * This package is especially useful for tuning these parameters or changing some user
 * defined values without the need to frequently change hard-coded values and flash the microcontroller.
 *
 * - received on channel #CH_IN_ROBOT_PARAMS (0x02)
 * - size: 16 Bytes
 * - minimum transmission duration (incl. minimum overhead of 5 bytes): 0.42ms
 */
typedef struct __attribute__((__packed__)) {
    float axleWidth;   ///< distance between wheels on axle, unit as defined by the user
    float distPerTick; ///< factor for converting encoder ticks to traveled distance, unit as defined by the user
    float user1;       ///< user defined value 1
    float user2;       ///< user defined value 2
} RobotParameters_t;


/**
 * 2-D Point based on int16_t.
 * Defines a 2-dimensional point in the global frame measured in mm
 * represented by int16_t values
 */
typedef struct __attribute__((__packed__)) {
    int16_t x; ///< x coordinate in global frame measured in mm
    int16_t y; ///< y coordinate in global frame measured in mm
} Point_t;


/**
 * 2-D Point based on float.
 * Defines a 2-dimensional point in the global frame measured in mm
 * represented by float values
 */
typedef struct __attribute__((__packed__)) {
    float x; ///< x coordinate in global frame measured in mm
    float y; ///< y coordinate in global frame measured in mm
} FPoint_t;


/**
 * Status of the path follower.
 * All contained information is evaluated by HWPCS and displayed in the Scene View.
 *
 * - sent on channel #CH_OUT_PATH_FOLLOW_STATUS (0x03)
 * - size: 17 Bytes
 * - minimum transmission duration (incl. minimum overhead of 5 bytes): 0.44ms
 */
typedef struct __attribute__((__packed__)) {
    bool enabled;       ///< true if path following is enabled
    Point_t segStart;   ///< start point of current path segment
    Point_t segEnd;     ///< end point of current path segment
    FPoint_t lookahead; ///< current lookahead point where robot is heading to
} PathFollowerStatus_t;


/**
 * Control command enumeration for PathFollowerControl_t.
 *
 * Note: Make sure to compile with "-fshort-enums" in order to have
 * PathFollowerCommmand_t represented as 8-bit type!
 */
typedef enum {
    FOLLOWER_CMD_NEWPATH = 0, ///< New path is being uploaded
    FOLLOWER_CMD_START   = 1, ///< start path following
    FOLLOWER_CMD_PAUSE   = 2, ///< pause path following
    FOLLOWER_CMD_RESET   = 3  ///< reset path following (but keep current path)
} PathFollowerCommand_t;


/**
 * Control command for path follower.
 * This package is sent from HWPCS for controlling the path follower.
 * If cmd is equal to #FOLLOWER_CMD_NEWPATH then the field pathLength defines
 * the total number of Point_t elements in the array points[].
 * If cmd is different from #FOLLOWER_CMD_NEWPATH, the values of pathLength and
 * points are ignored.
 *
 * Note: Make sure to compile with "-fshort-enums" in order to have
 * #PathFollowerCommand_t represented as 8-bit type!
 *
 * - received on channel #CH_IN_PATH_FOLLOW_CTRL (0x03)
 * - size: 2 + pathLength*4 Bytes
 * - minimum transmission duration (incl. minimum overhead of 5 bytes):
 *   0.02ms * (5 + 2 + pathLength*4)
 */
typedef struct __attribute__((__packed__)) {
    PathFollowerCommand_t cmd; ///< command for the path follower.
    uint8_t pathLength;        ///< number of path points in array points[]
    Point_t points[];          ///< array of path points with elements of type Point_t
} PathFollowerControl_t;


/**
 * Parameters for the path follower.
 *
 * - received on channel #CH_IN_PATH_FOLLOW_PARAMS (0x04)
 * - size: 5 Bytes
 * - minimum transmission duration (incl. minimum overhead of 5 bytes): 0.2ms
 */
typedef struct __attribute__((__packed__)) {
    float lookaheadDistance; ///< lookahead distance, see pathFollower_setLookaheadDistance()
    uint8_t segmentLimit;    ///< segment limit, see pathFollower_setSegmentLimit()
} PathFollowerParameters_t;


/**
 * Representation of labyrinth walls.
 * Each bit in walls represents the state of a wall. The bit number (i.e. wall number)
 * can be determined with labyrinth_getWallIndex().
 *
 * The first (#LABYRINTH_COLS + 1) * #LABYRINTH_ROWS bits represent Western and
 * Eastern walls of the cells in row0, row1, ...
 *
 * The last (#LABYRINTH_ROWS + 1) * #LABYRINTH_COLS bits represent Northern and
 * Southern walls of the cells in col0, col1, ...
 *
 * Use this for communicating the wall state to HWPCS for visualization:
 * - sent on channel #CH_OUT_LABY_WALLS (0x04)
 * - size: 16 Bytes (for a 7x7 labyrinth)
 * - minimum transmission duration (incl. minimum overhead of 5 bytes): 0.42ms
 */
typedef struct __attribute__((__packed__)) {
    uint8_t cols; ///< number of columns in labyrinth
    uint8_t rows; ///< number of rows in labyrinth
    /**
     * bit representation of walls (14 Bytes for a 7x7 labyrinth).
     * Computation of the array size is done by a ceiling operation expressed by integers:
     *   - number of Eastern/Western walls is: (cols + 1) * rows
     *   - number of Northern/Southern walls is: (rows + 1) * cols
     *   - sum of both equals total number of walls, i.e. number of bits
     *   - array size is ceiling of number of bits divided by 8
     *   - ceiling of an integer division x/y is achieved by 1 + (x - 1) / y
     */
    uint8_t walls[1 + (((LABYRINTH_COLS+1)*LABYRINTH_ROWS + (LABYRINTH_ROWS+1)*LABYRINTH_COLS - 1) >> 3)];
} LabyrinthWalls_t;


/**
 * Enumeration used by GetPose_t to determine for which AprilTag the
 * pose is requested from HWPCS.
 *
 * Note: Make sure to compile with "-fshort-enums" in order to have
 * AprilTagType_t represented as 8-bit type!
 */
typedef enum {
    APRIL_TAG_MAIN = 0,      ///< get pose of main AprilTag
    APRIL_TAG_ADDITIONAL = 1 ///< get pose of additional AprilTag
} AprilTagType_t;


/**
 * Get pose from HWPCS.
 * Send request to get pose from HWPCS of tracked AprilTags (visual markers).
 * If #APRIL_TAG_MAIN is requested, HWPCS will respond with Pose_t on channel
 * #CH_IN_POSE (0x05).
 * If #APRIL_TAG_ADDITIONAL is requested, HWPCS will respond with Pose_t on channel
 * #CH_IN_ADDITIONAL_POSE (0x08).
 *
 * A pose request of #APRIL_TAG_MAIN is left unanswered by HWPCS if multiple request
 * arrive within the interval specified by "Minimum Pose Update Interval" (see
 * HWPCS -> Preferences -> Scene -> Tracking). Requests for #APRIL_TAG_ADDITIONAL are
 * unconstrained.
 *
 * Accurate information is provided only if the robot stands still and enough time
 * has passed for HWPCS to acquire the tracking information.
 * Timing depends on various factors including WLAN. Safe timeouts are
 * considered at about 500ms or larger.
 *
 * - sent on channel #CH_OUT_GET_POSE (0x05)
 * - size: 1 Byte
 * - minimum transmission duration (incl. minimum overhead of 5 bytes): 0.12ms
 */
typedef struct __attribute__((__packed__)) {
    AprilTagType_t aprilTagType; ///< type of AprilTag for which tracking information is requested
} GetPose_t;


/**
 * User defined command.
 * User defined command which is sent from HWPCS. Can be used to execute
 * frequently occurring predefined tasks on the robot.
 *
 * - received on channel #CH_IN_USER_COMMAND (0x06)
 * - size: 1 Byte
 * - minimum transmission duration (incl. minimum overhead of 5 bytes): 0.12ms
 */
typedef struct __attribute__((__packed__)) {
    uint8_t id; ///< ID of the user defined command
} UserCommand_t;


/**
 * User defined data.
 *
 * - sent on channel #CH_OUT_USER_DATA (0x06)
 * - size: 28 Bytes
 * - minimum transmission duration (incl. minimum overhead of 5 bytes): 0.66ms
 */
typedef struct __attribute__((__packed__)) {
    uint16_t uint16;  ///< user defined uint16_t value
    uint32_t uint32;  ///< user defined uint32_t value
    int16_t int16;    ///< user defined int16_t value
    int32_t int32;    ///< user defined int32_t value
    float float1;     ///< user defined float value 1
    float float2;     ///< user defined float value 2
    float float3;     ///< user defined float value 3
    float float4;     ///< user defined float value 4
} UserData_t;


/**
 * Cell information to be shown in HWPCS.
 * Send this packet on channel #CH_OUT_LABY_CELL_INFO (0x08) to display the
 * given information in the Scene view of HWPCS. This can be useful for
 * debugging purposes if a labyrinth exploration algorithm is developed
 * which works with states of cells.
 *
 * Displayed information values range from -127 to 127. If a value of -128 is
 * given, the cell information in HWPCS is cleared.
 *
 * If either col or row exceed the number of cells - 1, cell information of
 * all cells is cleared in HWPCS. In this case, the field info can be set to
 * any value.
 *
 * - sent on channel #CH_OUT_LABY_CELL_INFO (0x08)
 * - size: 3 Bytes
 * - minimum transmission duration (incl. minimum overhead of 5 bytes): 0.16ms
 */
typedef struct __attribute__((__packed__)) {
    uint8_t col; ///< zero-based column index of labyrinth cell
    uint8_t row; ///< zero-based row index of labyrinth cell
    int8_t info; ///< information associated with the cell (use -128 to clear cell info in HWPCS)
} LabyrinthCellInfo_t;


/**
 * Wall information to be shown in HWPCS.
 * Send this packet on channel #CH_OUT_LABY_WALL_INFO (0x09) to display the
 * given information in the Scene view of HWPCS. This can be useful for
 * debugging purposes if a labyrinth exploration algorithm is developed
 * which works with states of walls.
 *
 * Displayed information values range from -127 to 127. If a value of -128 is
 * given, the wall information in HWPCS is cleared.
 *
 * If either col or row exceed the number of cells - 1, wall information of
 * all cells is cleared in HWPCS. In this case, the field info can be set to
 * any value.
 *
 * - sent on channel #CH_OUT_LABY_WALL_INFO (0x09)
 * - size: 4 Bytes
 * - minimum transmission duration (incl. minimum overhead of 5 bytes): 0.18ms
 */
typedef struct __attribute__((__packed__)) {
    uint8_t col;     ///< zero-based column index of labyrinth cell
    uint8_t row;     ///< zero-based row index of labyrinth cell
    Direction_t dir; ///< direction identifying the wall of the cell
    int8_t info;     ///< information associated with the wall (use -128 to clear wall info in HWPCS)
} LabyrinthWallInfo_t;

#endif /* PACKETTYPES_H_ */
