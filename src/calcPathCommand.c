#include "calcPathCommand.h"
#include <math.h>
#include <motor/motor.h>
#include "src/statemachine.h"

//check axleWidth, kAngular, forwardVelocity and conversion to PWM

#define axleWidth 187.0f 

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

//angular velocity
float kAngular = 1.0f; 
float angularVelocity = kAngular * angleDiff;

// Constant forward velocity
float forwardVelocity = 50.0f; 

// Compute left and right wheel velocities
vLeft = forwardVelocity - (angularVelocity * axleWidth / 2.0f);
vRight = forwardVelocity + (angularVelocity * axleWidth / 2.0f);

setState(Drive_Path_Command);
}

void drive_Path_Command() {
int16_t expectedEncoderL = (int16_t)(vLeft / 0.0688f);
int16_t expectedEncoderR = (int16_t)(vRight / 0.0688f);
//TODO
}