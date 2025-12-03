#include "bumper.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <tools/variablesAccess.h>
#include <stdint.h>

static volatile uint8_t contacts=0;
static volatile bitset8_t bumpers;

void bumper_init() {
    DDRD &= ~((1 << PD0) | (1 << PD1));     //set PD0 and PD1 as inputs
    PORTD |= (1 << PD0) | (1 << PD1);       //enable pull-up resistors on PD0 and PD1
    EIMSK |= (1 << INT0) | (1 << INT1);     //enable external interrupts INT0 and INT1
    EICRA |= (1 << ISC01) | (1 << ISC11);   //trigger INT0 and INT1 on falling edge
}

ISR(INT0_vect) {
    if(!bumper_getBumpers().bitset.bit1 == 1) //if one frontal collision detected, do not count contact twice
    contacts++;
}

ISR(INT1_vect) {
    contacts++;
}

uint8_t bumper_getContacts() {
    return contacts;
}

bitset8_t bumper_getBumpers() {
    bumpers.value = PIND & ((1 << PD0) | (1 << PD1));
    return bumpers;
}



