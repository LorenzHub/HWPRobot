#ifndef STATEMACHINE_H_
#define STATEMACHINE_H_

#include <stdint.h>

void stateMachine();

typedef enum {
    IDLE,
    Turn_On_Spot,
    Drive_Forward,
    Drive_Forward_5sec,
    Calibrate_Distance,
    Drive_Forward_Distance,
    Drive_Forward_Ticks,
    Turn_On_Spot_Degrees,
    ExploreMaze,
    drive_Forward_distance_then_explore,
    Turn_On_Spot_degrees_then_explore,
    turn_On_Spot_degrees_then_drive,
    FollowThePath,
    CorrectRotationMovement,
    CorrectRotationMovement_Wait,
    Drive_Forward_Ticks_With_Wall_Following
} state;

/* currentState is defined in statemachine.c to avoid multiple definitions
 * when this header is included by multiple translation units. */
extern state currentState;

void setState(state newState);
void mazeExplore(void);

void drive_Forward_5sec(void);
void drive_Forward_distance_mm(uint16_t distance_mm, int16_t pwmRight);
void statemachine_setTargetDistance(uint16_t distance_mm);
void drive_Forward_ticks(int32_t targetTicks, int16_t pwmRight);
void statemachine_setTargetTicks(int16_t targetTicks);
void statemachine_setTargetPWM(int16_t pwm);
void turn_On_Spot_degrees(int16_t angle_degrees, int16_t pwm);
void statemachine_setTargetAngle(int16_t angle_degrees);
void drive_Forward_distance_mm_then_explore(uint16_t distance_mm, int16_t pwmRight);
void turn_degrees_then_drive(int16_t angle_degrees, int16_t pwm);
void correctRotationMovement(void);
void drive_Forward_ticks_with_wall_following(int32_t targetTicks, int16_t pwmRight);
void statemachine_setWallDetectionThreshold(int16_t threshold_mm);
void statemachine_setWallCorrectionThreshold(int16_t threshold_mm);
void statemachine_setWallDistanceGain(int16_t gain);
void statemachine_setWallParallelGain(int16_t gain);
void statemachine_setCenterTargetDistance(int16_t distance_mm);

// Wandkorrektur Ein/Aus
void statemachine_setWallCorrectionEnabled(uint8_t enabled);
uint8_t statemachine_isWallCorrectionEnabled(void);
#endif /* STATEMACHINE_H_ */