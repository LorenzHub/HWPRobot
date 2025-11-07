#ifndef STATEMACHINE_H_
#define STATEMACHINE_H_

void stateMachine();

typedef enum {
    IDLE,
    Turn_On_Spot,
    Drive_Forward,
    Drive_Forward_5sec
} state;

/* currentState is defined in statemachine.c to avoid multiple definitions
 * when this header is included by multiple translation units. */
extern state currentState;

void setState(state newState);

void drive_Forward_5sec();

#endif /* STATEMACHINE_H_ */