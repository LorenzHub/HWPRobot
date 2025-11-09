#include "pathFollower.h"
#include <communication/communication.h>

#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>
#include <avr/pgmspace.h>

// Uncomment the following define or put it into CFLAGS if informational
// messages should not be sent to HWPCS
//#define PATHFOLLOWER_QUIET

// Uncomment the following define or put it into CFLAGS if error messages should
// not be sent to HWPCS
//#define PATHFOLLOWER_REALLY_QUIET


#ifdef PATHFOLLOWER_QUIET
    #define pathFollowerInfo(msg, ...)
    #define pathFollowerInfoP(msg, ...)
    #define pathFollowerInfoPNoArg(msg)
#else
    #define pathFollowerInfo(msg, ...) communication_log(LEVEL_INFO, msg, ##__VA_ARGS__)
    #define pathFollowerInfoP(msg, ...) communication_log_P(LEVEL_INFO, PSTR(msg), ##__VA_ARGS__)
    #define pathFollowerInfoPNoArg(msg) communication_log_P(LEVEL_INFO, PSTR(msg))
#endif

#if defined(PATHFOLLOWER_REALLY_QUIET) && defined(PATHFOLLOWER_QUIET)
    #define pathFollowerErrorP(msg)
#else
    #define pathFollowerErrorP(msg) communication_log_P(LEVEL_SEVERE, PSTR(msg))
#endif


// *************************** LOCAL VARIABLES *********************************

// The current PathFollowerStatus_t to be used to generate control commands for
// the motors each time after pathFollower_update() was called
static PathFollowerStatus_t pathFollower_status = { .enabled = false, .segStart = { 0, 0 }, .segEnd = { 0, 0 }, .lookahead = { 0.0f, 0.0f } };

// The current PathFollowerExtStatus_t which is updated by pathFollower_update()
static PathFollowerExtStatus_t pathFollower_extStatus = { .onLastSegment = false, .distanceOPToEnd = 0.0f };

// Distance in mm between orthogonal projection of current pose onto path and
// lookahead point. The lookahead point (in pathFollower_status) is always kept
// ahead of the orthogonal projection by this distance.
static float pathFollower_lookaheadDistance = 1.0f;

// Number of segments to look ahead (starting from the current segment) when
// locating the orthogonal projection of the current position onto the path.
static uint8_t pathFollower_segmentLimit = 5;

// Callback function which is executed when a path follower command is received
// see commPathFollower()
static PathFollowerCallback_t pathFollower_callback = 0;

// Array for storing path points
// A point is appended after the the last one which is extrapolated in the same
// direction as the last segment at a distance of ten times the lookahead distance
// after the last point. This eliminates the need for doing the extrapolation
// in pathFollower_update().
static Point_t* path = NULL;

// Length of the path given as number of points, equals size of array path - 1
// (because of extrapolated point which is appended to path[])
static uint8_t pathLength = 0;

// Index in array path containing the segment on which the last orthogonal
// projection was located.
// The index specifies the start point of a segment.
// -1 indicates that the first segment is being approached as the path follower
// assumes that initially the robot is not positioned on the first segment.
static int16_t curSeg = -1;

// indicates if variables declared below have been initialized by the first call
// to pathFollower_update. This variable is only set to true by pathFollower_update
// and reset to false by FOLLOWER_CMD_RESET or when setting a new path
// by setPath().
static bool initialized = false;

// Vector specifying the current segment as (end point - start point)
static FPoint_t curSegVec = { 0, 0 };

// squared length of curSegVec [mm^2]
static float curSegLenSq = 0.0f;

// length of curSegVec [mm]
static float curSegLen = 0.0f;

// Point at which the next segment ends
static Point_t nextSegEnd = { 0, 0 };

// Vector specifying the next segment as (end point - start point)
static FPoint_t nextSegVec = { 0, 0 };

// squared length of nextSegVec [mm^2]
static float nextSegLenSq = 0.0f;

// length of nextSegVec [mm]
static float nextSegLen = 0.0f;

// relative lookahead distance w.r.t. length of current segment
// equals: pathFollower_lookaheadDistance / curSegLen
static float relL = 0.0f;


// *************************** LOCAL FUNCTIONS *********************************

// compute squared length of a vector
static inline float sqLen(const FPoint_t vec) {
    return square(vec.x) + square(vec.y);
}


// compute the location of a point on a line in the relative distance l (w.r.t.
// length of vector vec) where the line is determined by its start point start
// and the direction vector vec.
static inline FPoint_t pointVec(FPoint_t vec, float l, Point_t start) {
    // compute FPoint_t p = { (vec.x * l) + start.x, (vec.y * l) + start.y };
    FPoint_t p = { fmaf(vec.x, l, start.x), fmaf(vec.y, l, start.y) };
    return p;
}


// similar to sqLen(pointVec(vec, l, start))
static inline float sqLenPointVecF(FPoint_t vec, float l, FPoint_t start) {
    return square(fmaf(vec.x, l, start.x)) + square(fmaf(vec.y, l, start.y));
}


// reset state of path follower
static void resetState(void) {
    curSeg = -1;
    initialized = false;
    pathFollower_status.enabled = false;
    pathFollower_extStatus.onLastSegment = false;
    pathFollower_extStatus.distanceOPToEnd = 0.0f;
}


// Set a path of length pathLen from array pathPoints.
// This function is used by commPathFollower() when command FOLLOWER_CMD_NEWPATH
// is received from HWPCS as well as by pathFollower_setNewPath().
// It checks if the path definition contains at least two points,
// resets all path follower local variables and copies the path to a newly
// allocated memory region.
// Furthermore, it extrapolates an additional point at the end of the path
// in direction of the last path segment. The length of the extrapolated segment
// is ten times the lookahead distance.
static bool setPath(const uint8_t pathLen, const Point_t* pathPoints) {
    if (pathLen < 2)
        return false; // path is too short

    // reset everything to initial condition
    pathLength = 0;
    resetState();

    // if a path was previously received, free its memory
    if (path != NULL)
        free(path);

    // allocate memory for the new path
    // include space for the extrapolated point at the end
    size_t s = sizeof(Point_t) * (pathLen + 1);
    path = (Point_t*)malloc(s);
    if (path == NULL)
        return false; // error during memory allocation

    // copy the path points to the allocated region
    memcpy(path, pathPoints, s);
    pathLength = pathLen;

    // extrapolate a new point in direction of the last segment with a distance
    // of ten times the lookahead distance from the last point.
    Point_t
        *pathPtr = &path[pathLength - 2],
        lastSegStart = *pathPtr, // equals path[pathLength - 2]
        lastSegEnd = *++pathPtr; // equals path[pathLength - 1], i.e. the last point

    // Compute segment vector as (end point - start point)
    FPoint_t
        lastSegVec = { lastSegEnd.x - lastSegStart.x, lastSegEnd.y - lastSegStart.y };
    float
        // length of last segment
        lastSegLen = hypotf(lastSegVec.x, lastSegVec.y),
        // relative distance of extrapolated point (w.r.t. to segment length)
        relDist = 10.0f * pathFollower_lookaheadDistance / lastSegLen;

    // extrapolate and append to array path (i.e. set path[pathLength])
    (++pathPtr)->x = (int16_t)(lastSegVec.x * relDist) + lastSegEnd.x;
    pathPtr->y = (int16_t)(lastSegVec.y * relDist) + lastSegEnd.y;

    return true;
}


// *************************** GLOBAL FUNCTIONS ********************************

// communication callback function for channel CH_IN_PATH_FOLLOW_CTRL in order to control the
// path follower from HWPCS
void commPathFollower(const uint8_t* packet, __attribute__((unused)) const uint16_t size) {
    PathFollowerControl_t* ctrl = (PathFollowerControl_t*)packet;

    if (ctrl->cmd == FOLLOWER_CMD_NEWPATH) { // if a new path was received from HWPCS

        if (! setPath(ctrl->pathLength, ctrl->points)) {
            pathFollowerErrorP("Follower: out of memory or path has too few points");
        } else {
            pathFollowerInfoP("Path of %" PRIu8 " points, 1.(%" PRIi16 ",%" PRIi16 ") 2.(%" PRIi16 ",%" PRIi16 ") ...", pathLength, path[0].x, path[0].y, path[1].x, path[1].y);
        }

    } else if (ctrl->cmd == FOLLOWER_CMD_START) {  // start command received
        if (pathLength == 0) {
            pathFollowerErrorP("No path available");
        } else {
            pathFollower_status.enabled = true;
            pathFollowerInfoPNoArg("Follower started");
        }
    } else if (ctrl->cmd == FOLLOWER_CMD_PAUSE) {  // pause command received
        pathFollower_status.enabled = false;
        pathFollowerInfoPNoArg("Follower paused");
    } else if (ctrl->cmd == FOLLOWER_CMD_RESET) {  // reset command received
        resetState();
        pathFollowerInfoPNoArg("Follower reset");
    }

    if (pathFollower_callback != 0)
        pathFollower_callback(ctrl->cmd, ctrl->pathLength);
}


// communication callback function for channel CH_IN_PATH_FOLLOW_PARAMS in order to receive
// path follower parameters from HWPCS
void commPathFollowerParameters(const uint8_t* packet, __attribute__((unused)) const uint16_t size) {
    PathFollowerParameters_t* params = (PathFollowerParameters_t*)packet;
    if (params->lookaheadDistance > 0.0f)
        pathFollower_lookaheadDistance = params->lookaheadDistance;
    pathFollower_segmentLimit = params->segmentLimit;
    pathFollowerInfoPNoArg("Follower parameters set");
}


// install communication callback functions for channels CH_IN_PATH_FOLLOW_CTRL and CH_IN_PATH_FOLLOW_PARAMS
void pathFollower_init(void) {
    communication_setCallback(CH_IN_PATH_FOLLOW_CTRL, commPathFollower);
    communication_setCallback(CH_IN_PATH_FOLLOW_PARAMS, commPathFollowerParameters);
}


// manually set a new path for the path follower
bool pathFollower_setNewPath(const Path_t* new_path) {
    return setPath(new_path->pathLength, new_path->points);
}


void pathFollower_clearPath(void) {
    if (path != NULL) {
        free(path);
        path = NULL;
    }

    pathLength = 0;
    resetState();
}


// manually start, pause or reset the path follower
void pathFollower_command(const PathFollowerCommand_t cmd) {
    switch (cmd) {
    case FOLLOWER_CMD_START:  // start command
        if (pathLength > 0)
            pathFollower_status.enabled = true;
        break;
    case FOLLOWER_CMD_PAUSE:  // pause command
        pathFollower_status.enabled = false;
        break;
    case FOLLOWER_CMD_RESET:  // reset command
        resetState();
        break;
    case FOLLOWER_CMD_NEWPATH:  // new path command is ignored
        break;
    }
}


// compute a new lookahead point, call this function periodically from main loop
bool pathFollower_update(const Pose_t* currentPose) {
    // exit if following is disabled
    if (! pathFollower_status.enabled)
        return false;

    // copy current position into local variable, allows for large code optimization
    FPoint_t currentPos = { currentPose->x, currentPose->y };

    // Step 0: Initialization

    // if this is the first time, pathFollower_update() was called after either
    // reset or a path has been set
    if (! initialized) {
        initialized = true;

        // initialize all local and global variables used by the path follower:
        // In most of the cases (when the distance between the current position and the first
        // path point path[0] is sufficiently large), the first (current) segment is defined
        // to go from the current position of the robot to the first path point.
        // The next segment connects point path[0] to point path[1].
        pathFollower_status.segEnd = path[0];
        curSegVec.x = pathFollower_status.segEnd.x - currentPos.x;
        curSegVec.y = pathFollower_status.segEnd.y - currentPos.y;
        curSegLenSq = sqLen(curSegVec);

        // if the distance between current position and first path point is very small
        if (curSegLenSq < 1.0e-2) {
            // then already switch to segment between path[0] and path[1]
            nextSegEnd = path[2];
            pathFollower_status.segEnd = path[1];
            pathFollower_status.segStart = path[0];
            curSegVec.x = pathFollower_status.segEnd.x - pathFollower_status.segStart.x;
            curSegVec.y = pathFollower_status.segEnd.y - pathFollower_status.segStart.y;

            curSegLenSq = sqLen(curSegVec);
            curSeg = 0;
        } else {
            // otherwise (most of the cases), we introduce a linear segment from the
            // current position to the first path point path[0], keeping curSeg = -1
            nextSegEnd = path[1];
            pathFollower_status.segStart.x = currentPos.x;
            pathFollower_status.segStart.y = currentPos.y;
        }

        nextSegVec.x = nextSegEnd.x - pathFollower_status.segEnd.x;
        nextSegVec.y = nextSegEnd.y - pathFollower_status.segEnd.y;
        nextSegLenSq = sqLen(nextSegVec);
        nextSegLen = sqrt(nextSegLenSq);

        curSegLen = sqrt(curSegLenSq);

        relL = pathFollower_lookaheadDistance / curSegLen;
    }

    // Step 1: Find the segment with smallest (squared) distance between current
    //         position and orthogonal projection

    // compute vector ECur from start of current segment to current position
    FPoint_t ECur = { currentPos.x - pathFollower_status.segStart.x, currentPos.y - pathFollower_status.segStart.y };

    float
        // compute dot product between vector ECur and vector P of current segment
        // equivalent to P_dot_ECur = curSegVec.x * ECur.x + curSegVec.y * ECur.y, but more accurate
        P_dot_ECur = fmaf(curSegVec.x, ECur.x, curSegVec.y * ECur.y),

        // compute relative distance lCur (w.r.t. to current segment length) of
        // the orthogonal projection of the current position onto the current segment
        lCur = P_dot_ECur / curSegLenSq,

        // compute squared distance between current position and orthogonal projection onto current segment
        // Note, that the following is short for:
        //   1. compute point OP of orthogonal projection: OP = pointVec(curSegVec, lCur, pathFollower_status.segStart)
        //   2. compute vector between OP and current position: vecOP_CP = OP - currentPos
        //   3. compute squared distance between OP and current position: sqLen(vecOP_CP)
        // and initialize minDist, holding the minimum squared distance between an
        // orthogonal projection onto a path segment
        minDist = sqLenPointVecF(curSegVec, -lCur, ECur);

    // compute vector ENext from start of next segment to current position
    // Note that the following is equal to:
    //   FPoint_t ENext = { currentPos.x - pathFollower_status.segEnd.x, currentPos.y - pathFollower_status.segEnd.y };
    // but produces smaller code
    FPoint_t ENext = { ECur.x - curSegVec.x, ECur.y - curSegVec.y };

    float
        // compute dot product between vector ENext and vector P of next segment
        // equivalent to P_dot_ENext = nextSegVec.x * ENext.x + nextSegVec.y * ENext.y, but more accurate
        P_dot_ENext = fmaf(nextSegVec.x, ENext.x, nextSegVec.y * ENext.y),

        // compute relative distance lNext (w.r.t. to next segment length) of
        // the orthogonal projection of the current position onto the next segment
        lNext = P_dot_ENext / nextSegLenSq;

    //communication_log(LEVEL_FINE, "%.3f %.3f", lCur, lNext);

    // initialize minSeg with the current segment index
    // minSeg holds the segment index with smallest squared distance minDist
    int16_t minSeg = curSeg;


    // if the orthogonal projection of the current position onto the next segment
    // is located on that segment
    if ((lNext >= 0.0f) && (lNext <= 1.0f)) {
        // compute squared distance between current position and orthogonal projection onto next segment
        float sqDistOPCPNext = sqLenPointVecF(nextSegVec, -lNext, ENext);

        // if that distance is smaller than the current minimum distance
        // or if orthogonal projection onto the current segment is already
        // past the current segment (in which case we definitively need to
        // switch at least to the next segment)
        if ((sqDistOPCPNext < minDist) || (lCur > 1.0f) || (minSeg == -1)) {
            // update minDist and minSeg
            minDist = sqDistOPCPNext;
            ++minSeg;
        }
    }

    // now iterate over the remaining segments up to pathFollower_segmentLimit
    // and try to find orthogonal projections onto these segments with even
    // smaller minimum distance to the current position.

    // segStart holds the start of the segment considered
    Point_t segStart = nextSegEnd;
    // seg is the segment index (start point of segment path[seg])
    int16_t seg = curSeg + 1;
    // pointer in array path for reading out the segment's end point
    Point_t *pathPtr = &path[seg + 1];

    // counter for comparison with pathFollower_segmentLimit
    // initialization with zero means that we covered already the check of the
    // segment defined by nextSegVec and nextSegEnd from before
    uint8_t i = 0;
    while ((++i < pathFollower_segmentLimit) && (++seg < pathLength)) {
        // get end of segment
        Point_t segEnd = *++pathPtr;

        // compute segment vector as (end point - start point)
        FPoint_t vec = { segEnd.x - segStart.x, segEnd.y - segStart.y };

        // compute vector E from start of segment to current position
        FPoint_t E = { currentPos.x - segStart.x, currentPos.y - segStart.y };
        float
            // compute dot product between vector E and vector P of segment
            // equivalent to P_dot_E = vec.x * E.x + vec.y * E.y, but more accurate
            P_dot_E = fmaf(vec.x, E.x, vec.y * E.y),
            // compute relative distance l (w.r.t. to segment length) of
            // the orthogonal projection of the current position onto the segment
            l = P_dot_E / sqLen(vec);

        // if the orthogonal projection of the current position onto the segment
        // is located on that segment
        if ((l >= 0.0f) && (l <= 1.0f)) {
            // compute squared distance between current position and orthogonal projection onto segment
            float sqDistOPCP = sqLenPointVecF(vec, -l, E);

            // if that distance is smaller than the current minimum distance
            if (sqDistOPCP < minDist) {
                // update minDist and minSeg
                minDist = sqDistOPCP;
                minSeg = seg;
            }

        }

        // segEnd becomes the start of the segment in the next iteration
        segStart = segEnd;
    }

    // if minSeg hit the maximum, output an error (note that this should not happen)
    if (minSeg == pathLength) {
        pathFollowerErrorP("Unable to locate segment for orthogonal projection");

    // if segment with smallest distance between orthogonal projection and current position
    // is after the current segment
    } else if (minSeg > curSeg) {
        // if the end of the path has been reached, deactivate follower and report success
        if (minSeg == pathLength - 1) {
            pathFollower_status.enabled = false;
            pathFollowerInfoPNoArg("End of path reached");
            return false;
        }

        pathFollowerInfo("Switching from segment %" PRIi16 " to %" PRIi16, curSeg, minSeg);

        // get a pointer of path[minSeg], i.e. start of segment.
        // This produces more effective code than just addressing the array multiple
        // times at consecutive locations.
        Point_t* pathPtr = &path[minSeg];

        // if a switch to the next segment is needed, some required variables
        // are stored already in local variables
        if (minSeg == curSeg + 1) {
            ++pathPtr; // advance by one, since access to path[minSeg + 2] is required after this block
            lCur = lNext;
            curSegLenSq = nextSegLenSq;
            curSegVec = nextSegVec;
            pathFollower_status.segStart = pathFollower_status.segEnd;
            pathFollower_status.segEnd = nextSegEnd;
            curSegLen = nextSegLen;
        // if a switch to past the next segment is needed, update the local variables directly
        } else {
            pathFollower_status.segStart = *(pathPtr++); // equals path[minSeg]
            pathFollower_status.segEnd = *pathPtr;       // equals path[minSeg + 1]
            curSegVec.x = pathFollower_status.segEnd.x - pathFollower_status.segStart.x;
            curSegVec.y = pathFollower_status.segEnd.y - pathFollower_status.segStart.y;
            curSegLenSq = sqLen(curSegVec);

            // recomputation of lCur is required, as it is needed for the next step
            FPoint_t ECur = { currentPos.x - pathFollower_status.segStart.x, currentPos.y - pathFollower_status.segStart.y };
            float
                // equivalent to P_dot_ECur = curSegVec.x * ECur.x + curSegVec.y * ECur.y, but more accurate
                P_dot_ECur = fmaf(curSegVec.x, ECur.x, curSegVec.y * ECur.y);
            lCur = P_dot_ECur / curSegLenSq;

            curSegLen = sqrt(curSegLenSq);
        }

        // update the remaining local variables
        relL = pathFollower_lookaheadDistance / curSegLen;

        nextSegEnd = *(++pathPtr); // equals path[minSeg + 2]
        nextSegVec.x = nextSegEnd.x - pathFollower_status.segEnd.x;
        nextSegVec.y = nextSegEnd.y - pathFollower_status.segEnd.y;
        nextSegLenSq = sqLen(nextSegVec);
        nextSegLen = sqrt(nextSegLenSq);

        curSeg = minSeg;
    }


    if (curSeg == pathLength - 2) {
        pathFollower_extStatus.onLastSegment = true;
        pathFollower_extStatus.distanceOPToEnd = (1.0f - lCur) * curSegLen;
    }

    // Step 2: Locate the lookahead point in distance pathFollower_lookaheadDistance
    //         after the orthogonal projection from step 1

    // safe-guard in case robot deviates from path too much such that a switch to another segment
    // does not occur. In this case, the orthogonal projection stops at the end of the current segment.
    // This prevents the lookahead point from being placed outside the path.
    if (lCur > 1.0f)
        lCur = 1.0f;

    // compute the relative distance of the lookahead point in terms of the current segment
    float relLA = relL + lCur;

    // if lookahead point is on the current segment
    if (relLA <= 1.0f) {

        // compute lookahead point on current segment
        pathFollower_status.lookahead = pointVec(curSegVec, relLA, pathFollower_status.segStart);

    // if lookahead point is after the current segment
    } else {

        // subtract the distance between orthogonal projection on current segment to current segment end
        // from pathFollower_lookaheadDistance, such that Lrem contains the remaining distance of the lookahead
        // point along the path after the current segment.
        // This is expressed shorter as:
        float Lrem = (relLA - 1.0f) * curSegLen;

        // if lookahead point is on the next segment
        if (Lrem <= nextSegLen) {
            // compute lookahead point on next segment
            pathFollower_status.lookahead = pointVec(nextSegVec, Lrem / nextSegLen, pathFollower_status.segEnd);

        // if lookahead point is after the next segment
        } else {
            // update Lrem by subtracting the length of the next segment
            Lrem -= nextSegLen;

            // search for segment where lookahead point is located

            // segStart holds the start of the segment considered
            Point_t segStart = nextSegEnd;
            // seg is the segment index (start point of segment path[seg])
            uint8_t seg = curSeg + 1;
            // pointer in array path for reading out the segment's end point
            Point_t *pathPtr = &path[seg + 1];

            while (++seg < pathLength) {
                // get end of segment
                Point_t segEnd = *++pathPtr;

                // compute segment vector as (end point - start point)
                FPoint_t vec = { segEnd.x - segStart.x, segEnd.y - segStart.y };
                // compute length of vec
                float vecLen = hypotf(vec.x, vec.y);

                // if lookahead point is on the segment
                if (vecLen >= Lrem) {
                    // compute lookahead point on that segment
                    pathFollower_status.lookahead = pointVec(vec, Lrem / vecLen, segStart);
                    break;
                } else
                    // update Lrem by subtracting the length of the segment
                    Lrem -= vecLen;

                // segEnd becomes the start of the segment in the next iteration
                segStart = segEnd;
            }

            // if last segment was reached, but still the lookahead point could not
            // be found, output an error message.
            // Note that this condition should not occur, except when enlarging
            // the lookahead distance by more than that distance during a following
            // approach
            if (seg == pathLength) {
                pathFollowerErrorP("Unable to locate segment for lookahead point");
            }
        }
    }

    // report that pathFollower_status was updated with a new lookahead point
    // and possibly also new segments start and end points
    // A controller should now use the lookahead point as an intermediate goal
    // position and generate appropriate control inputs to the robot
    return true;
}


const PathFollowerStatus_t* pathFollower_getStatus(void) {
    return &pathFollower_status;
}


const PathFollowerExtStatus_t* pathFollower_getExtStatus(void) {
    return &pathFollower_extStatus;
}


float pathFollower_getLookaheadDistance(void) {
    return pathFollower_lookaheadDistance;
}


void pathFollower_setLookaheadDistance(const float lookaheadDistance) {
    if (lookaheadDistance > 0.0f)
        pathFollower_lookaheadDistance = lookaheadDistance;
}


uint8_t pathFollower_getSegmentLimit(void) {
    return pathFollower_segmentLimit;
}


void pathFollower_setSegmentLimit(const uint8_t segmentLimit) {
    pathFollower_segmentLimit = segmentLimit;
}


void pathFollower_setCallback(const PathFollowerCallback_t callback) {
    pathFollower_callback = callback;
}


void pathFollower_removeCallback(void) {
    pathFollower_callback = 0;
}
