/**
 * @file pathFollower.h
 * @ingroup pathFollower
 *
 * Implementation of a path follower which interfaces with HWPCS.
 * Additionally, the path follower can be controlled directly by functions
 * such as pathFollower_setNewPath() and pathFollower_command().
 *
 * The path follower internally manages two states of type PathFollowerStatus_t
 * and PathFollowerExtStatus_t. These states are updated by calls to
 * pathFollower_update(). The current state can be acquired with
 * pathFollower_getStatus() and pathFollower_getExtStatus().
 *
 * In order to have HWPCS reflect the recent state and update the Scene view accordingly,
 * the state returned by pathFollower_getStatus() must be sent regularly
 * to HWPCS.
 *
 * You are welcome to modify the path follower according to your needs.
 *
 * When compiling with the symbol PATHFOLLOWER_QUIET defined, no informational
 * messages are sent to HWPCS. This includes information when a path has been
 * set, following is started, paused or reset, path following parameters are
 * set, end of path is reached and a switch to another segment occurs. Only
 * error messages (such as when a path can not be set or when the path follower
 * cannot find the orthogonal projection or lookahead point) are transmitted to
 * HWPCS. PATHFOLLOWER_QUIET saves 428 bytes of flash memory and 32 bytes of
 * SRAM.
 *
 * The error messages can be suppressed when additionally compiling with the
 * symbol PATHFOLLOWER_REALLY_QUIET defined (only applicable if
 * PATHFOLLOWER_QUIET is defined as well).
 * PATHFOLLOWER_REALLY_QUIET additionally saves 278 bytes of flash memory.
 */

#ifndef PATHFOLLOWER_H_
#define PATHFOLLOWER_H_

#include <communication/packetTypes.h>

#include <stdint.h>
#include <stdbool.h>


/**
 * Definition of a path consisting of multiple points.
 * This data structure is used in conjunction with pathFollower_setNewPath() for
 * manually setting a new path for the path follower.
 */
typedef struct {
    uint8_t pathLength;     ///< number of path points in array points
    Point_t* points;        ///< array of path points with elements of type Point_t
} Path_t;


/**
 * Extended state of the path follower which is updated by pathFollower_update()
 * and which can be acquired by pathFollower_getExtStatus().
 *
 * As soon as the orthogonal projection of the robot's position is located on the
 * last path segment, the field onLastSegment becomes true and the field
 * distanceOPToEnd holds the distance in mm between the orthogonal projection
 * and the end of the last path segment.
 *
 * This information is helpful when implementing a speed controller to slow
 * down when the end of the path is approached.
 */
typedef struct {
    bool onLastSegment;    ///< true if the orthogonal projection is on the last path segment, otherwise false
    float distanceOPToEnd; ///< if onLastSegment is true, distanceOPToEnd holds the distance in mm between the orthogonal projection and the end of the path
} PathFollowerExtStatus_t;


/**
 * Type definition of a function pointer defining the callback function
 * which is executed when a path follower command is received on channel #CH_IN_PATH_FOLLOW_CTRL.
 *
 * This is useful if user defined actions are to be performed on reception
 * of a command for the path follower. For example, the motors can be stopped
 * when the path follower is paused.
 *
 * The callback function is invoked from the communication callback function which
 * is automatically registered for channel #CH_IN_PATH_FOLLOW_CTRL (in pathFollower_init()).
 *
 * Registering the callback function <code>cbf()</code> can be done by
 * {@link pathFollower_setCallback}(cbf).
 *
 * <b>Example:</b>
 * @code
 * void cbf(const PathFollowerCommand_t cmd, const uint8_t pathLen) {
 *    if (cmd == FOLLOWER_CMD_PAUSE) {
 *        Motor_stopAll();
 *    }
 * }
 * ...
 * pathFollower_setCallback(cbf);
 * @endcode
 *
 * The signature of the callback function is:
 * @param   cmd       the path follower command from #PathFollowerCommand_t
 * @param   pathLen   length of the path (non-zero only for #FOLLOWER_CMD_NEWPATH,
 *                    otherwise zero)
 */
typedef void (*PathFollowerCallback_t)(const PathFollowerCommand_t, const uint8_t);


/**
 * Initialize the path following library.
 *
 * Must be invoked before any other path follower library function, but
 * after initialization of the communication library (communication_init()).
 *
 * Registers a communication callback on channel #CH_IN_PATH_FOLLOW_PARAMS for
 * receiving path follower parameters (PathFollowerParameters_t) from HWPCS with
 * which the lookahead distance and the segment limit can be conveniently set
 * (see pathFollower_setLookaheadDistance() and pathFollower_setSegmentLimit()
 * for details).
 *
 * Another communication callback is registered on channel #CH_IN_PATH_FOLLOW_CTRL
 * for receiving path follower control commands (PathFollowerControl_t) from HWPCS.
 * This enables control of the path follower from HWPCS. If an additional callback
 * function for the path follower is registered with pathFollower_setCallback(),
 * that function gets called when a PathFollowerControl_t was received from HWPCS.
 */
void pathFollower_init(void);


/**
 * Calculate and determine all the fields in the current PathFollowerStatus_t
 * and PathFollowerExtStatus_t (which are internally managed and can be acquired
 * with pathFollower_getStatus() and pathFollower_getExtStatus())
 * from the current robot position and the path (either received by the
 * communication protocol or manually set by pathFollower_setNewPath()) in case
 * path following is enabled.
 *
 * This function should be called from <code>main()</code> context in regular intervals.
 *
 * The function operates in two steps:
 * 1. First, the location of the orthogonal projection of the current pose onto the path
 *    is determined. The function determines, starting from the current segment (in
 *    PathFollowerStatus_t.segStart and PathFollowerStatus_t.segEnd) along subsequent
 *    path segments, which orthogonal projection of the current position onto the
 *    segments has smallest distance to the current position. In order to keep
 *    execution time acceptable and also to avoid taking shortcuts (or skipping
 *    too many segments) in case of concave or crossing paths, the number of
 *    consecutive segments examined is limited by the value supplied to
 *    pathFollower_setSegmentLimit().
 * 2. Once the location of the orthogonal projection is found in step 1, the
 *    location of the lookahead point is determined by advancing the distance
 *    given by pathFollower_setLookaheadDistance() along the path, even over
 *    multiple segments if necessary.
 *
 * If the function returns true, a controller should generate control inputs for
 * approaching the lookahead point.
 * If the function returns false, the robot should stop and no further control
 * inputs should be applied.
 *
 * Use timeTask_getTimestamp() and timeTask_getDuration() timeTask_getDuration()
 * to measure the execution time of this function as it depends heavily on the
 * values given by pathFollower_setSegmentLimit() and
 * pathFollower_setLookaheadDistance(). Note that the first run of
 * pathFollower_update() after a new path has been set takes much longer than
 * following runs since several internal variables need to be initialized.
 *
 * @param   currentPose   current pose of the robot
 * @return  true if the internally managed state structures were updated and
 *               following is still enabled,
 *          false if following was disabled in case of reaching the path's end
 */
bool pathFollower_update(const Pose_t* currentPose);


/**
 * Manually set a new path for the path follower.
 * The path must consist of at least two points. Two consecutive points must
 * be different from each other, i.e. any segment length must be non-zero.
 *
 * This function can be used if the path is not handed over by HWPCS but set
 * directly from C code.
 *
 * <b>Example:</b>
 * @code
 * Point_t pathPoints[3] = { { 0, 400 }, { 400, 600 }, { 600, 0 } };
 * Path_t path = { .pathLength = 3, .points = &pathPoints[0] };
 * pathFollower_setNewPath(&path);
 * @endcode
 *
 * @param   new_path   a pointer to a Path_t structure containing the new path
 * @return  true if the path was successfully set, false if not enough memory is
 *          available for storing the path or the path was too short.
 */
bool pathFollower_setNewPath(const Path_t* new_path);


/**
 * Clear a previously set path (either through HWPCS or by means of pathFollower_setNewPath())
 * and reset the path follower in order to free unused memory.
 */
void pathFollower_clearPath(void);


/**
 * Manually issue a command for the path follower (start, pause, reset).
 *
 * This function can be used if the path follower is commanded directly and not
 * from within HWPCS.
 *
 * A callback function registered with pathFollower_setCallback() will not be executed.
 *
 * Note that the command ::FOLLOWER_CMD_NEWPATH is ignored since this is
 * covered by pathFollower_setNewPath().
 *
 * @param   cmd   the command for the path follower. Can be one of
 *                ::FOLLOWER_CMD_START, ::FOLLOWER_CMD_PAUSE or
 *                ::FOLLOWER_CMD_RESET
 */
void pathFollower_command(const PathFollowerCommand_t cmd);


/**
 * Retrieves the current PathFollowerStatus_t which is managed internally and updated
 * by pathFollower_update().
 *
 * Fields PathFollowerStatus_t.segStart and PathFollowerStatus_t.segEnd denote the
 * path segment on which the orthogonal projection of the robot's pose onto the path was
 * found.
 *
 * The field PathFollowerStatus_t.lookahead specifies the coordinates of the lookahead
 * point on the path which is kept ahead by a distance (given by
 * pathFollower_setLookaheadDistance()) after the orthogonal projection.
 * The lookahead point is to be treated as an intermediate goal point used for
 * generating control commands for the robot.
 *
 * Send the return value of this function on channel #CH_OUT_PATH_FOLLOW_STATUS each
 * time after pathFollower_update() completed in order to have HWPCS evaluate it such
 * that all information is displayed in the Scene View. This also allows HWPCS to detect
 * the status of the path follower, e.g. when the robot reached the end of the path.
 *
 * @return  a pointer to the structure of type PathFollowerStatus_t which is updated by
 *          pathFollower_update().
 */
const PathFollowerStatus_t* pathFollower_getStatus(void);

/**
 * Retrieves the current PathFollowerExtStatus_t which is managed internally and updated
 * by pathFollower_update().
 *
 * As soon as the orthogonal projection of the robot's position is located on the
 * last path segment, the field onLastSegment becomes true and the field
 * distanceOPToEnd holds the distance in mm between the orthogonal projection
 * and the end of the last path segment.
 *
 * This information is helpful when implementing a speed controller to slow
 * down when the end of the path is approached.
 *
 * @return  a pointer to the structure of type PathFollowerExtStatus_t which is updated by
 *          pathFollower_update().
 */
const PathFollowerExtStatus_t* pathFollower_getExtStatus(void);

/**
 * Retrieve the currently used lookahead distance.
 *
 * See pathFollower_setLookaheadDistance() for details.
 *
 * @return  lookahead distance in millimeters
 */
float pathFollower_getLookaheadDistance(void);


/**
 * Set the distance in mm between orthogonal projection of current position onto path
 * and lookahead point. The distance must be positive.
 * Using pathFollower_update(), the lookahead point (computed in an internally managed
 * structure of type PathFollowerStatus_t, which can be fetched with
 * pathFollower_getStatus()) is always kept on the path ahead of the orthogonal
 * projection by this distance.
 *
 * Note that when updating this value while the path follower is running, the new
 * lookahead distance must be less than ten times the old one, otherwise the
 * lookahead point cannot be located. This is because the extrapolation of an
 * additional point at the end of the path (at distance of ten times the
 * lookahead distance after the last point) is only done when a new path is set.
 * An updated value of the lookahead distance is only taken over by the
 * path follower when a switch of the orthogonal projection onto the path to
 * another segment occurs.
 *
 * Default is 1mm.
 *
 * @param   lookaheadDistance   the lookahead distance given in millimeters
 */
void pathFollower_setLookaheadDistance(const float lookaheadDistance);


/**
 * Retrieve the currently used segment limit.
 *
 * See pathFollower_setSegmentLimit() for details.
 *
 * @return  the lookahead distance in millimeters
 */
uint8_t pathFollower_getSegmentLimit(void);


/**
 * Set the number of segments (starting from the current one) to look ahead when
 * locating the orthogonal projection of the current position onto the path.
 * The minimum value is 1, indicating that only the current and next segment
 * are taken into account for locating the lookahead point. A value of 0 is
 * ignored.
 *
 * This value should not be set too large, as this dramatically increases the
 * execution time of pathFollower_update().
 * Use timeTask_getTimestamp() and timeTask_getDuration() to measure the execution
 * time in order to select a feasible value for the segment limit.
 *
 * Additionally, a large value can cause the path following algorithm to take
 * unwanted shortcuts in case of crossing or concave paths.
 *
 * It is however necessary, to deal with the problem of a concave path or a path
 * which crosses itself.
 *
 * Default is 5.
 *
 * @param   segmentLimit   the segment limit which is used by pathFollower_update()
 */
void pathFollower_setSegmentLimit(const uint8_t segmentLimit);


/**
 * Register a callback function with prototype as defined by ::PathFollowerCallback_t
 * which is executed when a path follower command is received on channel #CH_IN_PATH_FOLLOW_CTRL
 * from HWPCS.
 *
 * The callback function is invoked from the communication callback function which
 * is automatically registered for channel #CH_IN_PATH_FOLLOW_CTRL (in pathFollower_init()).
 *
 * @param   callback function pointer to the callback function
 */
void pathFollower_setCallback(const PathFollowerCallback_t callback);


/**
 * Unregister the callback function for path follower commands from HWPCS.
 */
void pathFollower_removeCallback(void);


#endif /* PATHFOLLOWER_H_ */
