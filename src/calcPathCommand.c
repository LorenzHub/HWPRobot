#include "calcPathCommand.h"
#include <math.h>
#include <motor/motor.h>
#include "communication/communication.h"
#include <inttypes.h>

// Thresholds and PWM values from specification
#define ANGLE_THRESHOLD 0.05f // radians
#define PWM_FAST 6000
#define PWM_SLOW 1000
#define PWM_STRAIGHT 3000

void calculateDriveCommand (const Pose_t *currentPose, const FPoint_t* lookahead) {
    float dx = lookahead->x - currentPose->x;
    float dy = lookahead->y - currentPose->y;
    float angleToLookahead = atan2f(dy, dx);
    float angleDiff = angleToLookahead - currentPose->theta;

    // Normalize angleDiff to the range [-pi, pi]
    while (angleDiff > M_PI) angleDiff -= 2.0f * M_PI;
    while (angleDiff < -M_PI) angleDiff += 2.0f * M_PI;

    // Bang-Bang Control according to specification
    if (angleDiff < -ANGLE_THRESHOLD) {
        // Target is to the right -> Turn Right
        // Left motor fast, Right motor slow
        Motor_setPWM(PWM_FAST, PWM_SLOW);
    } else if (angleDiff > ANGLE_THRESHOLD) {
        // Target is to the left -> Turn Left
        // Left motor slow, Right motor fast
        Motor_setPWM(PWM_SLOW, PWM_FAST);
    } else {
        // Go Straight
        Motor_setPWM(PWM_STRAIGHT, PWM_STRAIGHT);
    }
}
