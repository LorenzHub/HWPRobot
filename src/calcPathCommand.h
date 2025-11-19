#ifndef CALCPATHCOMMAND_H_
#define CALCPATHCOMMAND_H_

#include <communication/packetTypes.h>

/**
 * Calculates motor commands based on current pose and lookahead point.
 * Implements a Bang-Bang controller with +/- 0.05 rad threshold.
 * 
 * @param currentPose The current robot pose
 * @param lookahead The target lookahead point
 */
void calculateDriveCommand(const Pose_t *currentPose, const FPoint_t* lookahead);

#endif /* CALCPATHCOMMAND_H_ */
