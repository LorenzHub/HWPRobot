#include "calcPathCommand.h"
#include <math.h>
#include <motor/motor.h>
#include "communication/communication.h"
#include <inttypes.h>

//check axleWidth, kAngular, forwardVelocity and conversion to PWM

#define axleWidth 166.0f 

static float vLeft = 0.0f;
static float vRight = 0.0f;

void calculateDriveCommand (Pose_t *currentPose, FPoint_t* lookahead) {
float dx = lookahead->x - currentPose->x;
float dy = lookahead->y - currentPose->y;
float angleToLookahead = atan2f(dy, dx);
float angleDiff = angleToLookahead - currentPose->theta;

// Normalize angleDiff to the range [-pi, pi]
while (angleDiff > M_PI) angleDiff -= 2.0f * M_PI;
while (angleDiff < -M_PI) angleDiff += 2.0f * M_PI;

// angular velocity
float kAngular = 1.0f; 
float angularVelocity = kAngular * angleDiff;

// Constant forward velocity
float forwardVelocity = 85.0f; 

// If the angle difference is very small, go straight
if (fabsf(angleDiff) < 0.1f) {
    vLeft = forwardVelocity;
    vRight = forwardVelocity;
} else {
    vLeft = forwardVelocity - (angularVelocity * (axleWidth / 2.0f)); // vLeft and vRight in mm/s
    vRight = forwardVelocity + (angularVelocity * (axleWidth / 2.0f));
    communication_log(LEVEL_INFO, "angularVelocity: %.3f", angularVelocity);
}

drive_Path_Command();
}

void drive_Path_Command() {
// PWM = 55260.63*v + 938.3438  (v in m/s)
communication_log(LEVEL_INFO, "vLeft: %.2f mm/s  vRight: %.2f mm/s", vLeft, vRight);
Motor_setPWM_A((int16_t)(55260.63f * (vLeft * 0.001f) + 938.3438f)); // Motor A drives the left wheel
Motor_setPWM_B((int16_t)(55260.63f * (vRight * 0.001f) + 938.3438f));
}