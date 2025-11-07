#include "statemachine.h"
#include <motor/motor.h>
#include <tools/timeTask/timeTask.h>



/* Define the current state here (single-definition) */
state currentState = IDLE;

static timeTask_time_t start; 

void stateMachine() {
    switch(currentState){
        case IDLE:
            Motor_stopAll();
            break;
        case Turn_On_Spot:
            /* rotate in-place */
            Motor_setPWM(3000, -3000);
            break;
        case Drive_Forward:
            //do something
            break;
        case Drive_Forward_5sec:
            drive_Forward_5sec();
            break;
    }
}

void drive_Forward_5sec() {
    static uint8_t initialized = 0;
    timeTask_time_t now;
    
    if (!initialized) {
        timeTask_getTimestamp(&start);
        Motor_setPWM(3000, 3000);
        initialized = 1;
    }
    
    timeTask_getTimestamp(&now);
    if (timeTask_getDuration(&start, &now) > 5000000UL) {  // 5 seconds in microseconds
        setState(IDLE);
        initialized = 0;
    }
}

void setState(state newState) {
    currentState = newState;
}
