#ifndef CalcPathCommand_H
#define CalcPathCommand_H

#include <communication/packetTypes.h>

void calculateDriveCommand (Pose_t *currentPose, FPoint_t* lookahead);

void drive_Path_Command();

#endif /* CalcPathCommand_H */