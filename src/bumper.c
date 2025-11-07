#include "bumper.h"
#include <avr/io.h>

void bumper_init() {
    PORTE = (1<<PE7) | (1<<PE6); // Set PE7 and PE6 as inputs for bumpers with pull-up resistors
}

//TODO: implement ISR

