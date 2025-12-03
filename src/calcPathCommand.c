#include "calcPathCommand.h"
#include <math.h>
#include <motor/motor.h>
#include "communication/communication.h"
#include <inttypes.h>
#include <stdlib.h>

//check axleWidth, kAngular, forwardVelocity and conversion to PWM
//implement poseUpdate via Apriltracking data

#define axleWidth 170.0f 

static float vLeft = 0.0f;
static float vRight = 0.0f;

/*Follow the carrot*/
void calculateDriveCommand (Pose_t *currentPose, FPoint_t* lookahead) {
communication_log(LEVEL_INFO, "calculateDriveCommand called!");
float dx = lookahead->x - currentPose->x;
float dy = lookahead->y - currentPose->y;
float angleToLookahead = atan2f(dy, dx);
float angleDiff = angleToLookahead - currentPose->theta;

// Normalize angleDiff to the range [-pi, pi]
while (angleDiff > M_PI) angleDiff -= 2.0f * M_PI;
while (angleDiff < -M_PI) angleDiff += 2.0f * M_PI;

// angular velocity
float kAngular = 1.5f; 
float angularVelocity = kAngular * angleDiff;

// Constant forward velocity
float forwardVelocity = 25.0f; 

vLeft = forwardVelocity - (angularVelocity * (axleWidth / 2.0f)); // vLeft and vRight in mm/s
vRight = forwardVelocity + (angularVelocity * (axleWidth / 2.0f));

// Use dtostrf() for float logging (avr-libc doesn't support %f in printf by default)
char angVelBuf[10];
dtostrf(angularVelocity, 6, 3, angVelBuf);
communication_log(LEVEL_INFO, "angularVelocity: %s", angVelBuf);

drive_Path_Command();
}

void drive_Path_Command() {
communication_log(LEVEL_INFO, "drive_Path_Command called!");
// PWM = 55260.63*v + 938.3438  (v in m/s)
char vLeftBuf[10], vRightBuf[10];
dtostrf(vLeft, 6, 2, vLeftBuf);
dtostrf(vRight, 6, 2, vRightBuf);
communication_log(LEVEL_INFO, "vLeft: %s mm/s  vRight: %s mm/s", vLeftBuf, vRightBuf);
Motor_setPWM_A((int16_t)(55260.63f * (vLeft * 0.001f) + 938.3438f)); // Motor A drives the left wheel
Motor_setPWM_B((int16_t)(55260.63f * (vRight * 0.001f) + 938.3438f));
}