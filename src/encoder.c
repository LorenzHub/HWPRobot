#include <avr/io.h>
#include "encoder.h"


void encoder_init(){
    PORTD = (1<<PD0) | (1<<PD1); // 
    PORTB = (1<<PB7)|(1<<PB6); //

    DDRD = ~((1<<DDD0) | (1<<DDD1));               // PD0 & PD1 as Inputs
    EIMSK |= (1<<INT0) | (1<<INT1);                // Enable INT0 & INT1
    EICRA |= (1<<ISC00) | (1<<ISC10);              // Trigger INT0 & INT1 on any logical change

    DDRB = ~((1<<DDB7) | (1<<DDB6));           // PB7 & PB6 as Inputs
    PCICR |= (1<<PCIE0);                        // Pin-Change-Interrupt for each Pin in PCINT7:0
    PCMSK0 |= ((1<<PCINT7) | (1<<PCINT6));     // PB7 & PB6 (PCINT7/6) enable
}


