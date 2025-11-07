#include <avr/io.h>
#include "encoder.h"


void encoder_init(){
    PORTD = (1<<PD0) | (1<<PD1); // Set PD0 and PD1 as inputs for encoder1 with pull-up resistors
    PORTB = (1<<PB7)|(1<<PB6); // Set PB7 and PB6 as inputs for encoder2 with pull-up resistors
}


