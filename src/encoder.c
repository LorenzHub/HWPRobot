#include <avr/io.h>
#include "encoder.h"

#define PIN_CHANGED(new, old, bit) (((new) ^ (old)) & (1 << (bit)))
#define PIN_IS_HIGH(pin, bit) ((pin) & (1 << (bit)))

static volatile uint8_t pinOld;
static volatile int16_t counterR;
static volatile int16_t counterL;

void encoder_init(){
    DDRB = ~((1<<DDB7) | (1<<DDB6) | (1<<DDB5) | (1<<DDB4));                    // PB7 to PB4 as Inputs
    PCICR |= (1<<PCIE0);                                                        // Pin-Change-Interrupt for each Pin in PCINT7:0
    PCMSK0 |= ((1<<PCINT7) | (1<<PCINT6))| (1<<PCINT5) | (1<<PCINT4);           // PB7 to PB4 (PCINT7/6/5/4) enable
    pinOld = PINB & ((1<<PCINT7) | (1<<PCINT6) | (1<<PCINT5) | (1<<PCINT4));
}


ISR(PCINT0_vect){
	uint8_t pinNew = PINB & ((1<<PCINT7) | (1<<PCINT6) | (1<<PCINT5) | (1<<PCINT4));
	
	//Right Encoder
	if( PIN_CHANGED(pinNew, pinOld, PCINT0) ) { // flank change on channel a
		if (PIN_IS_HIGH(pinNew, PCINT0)) {	    // flank change from LOW to HIGH
			if (PIN_IS_HIGH(pinNew, PCINT1))	// B is HIGH
				counterR--;
			else								// B is LOW
				counterR++;
		} 
		else {							        // flank change from HIGH to LOW
			if (PIN_IS_HIGH(pinNew, PCINT1))	// B is HIGH
				counterR++;
			else								// B is LOW
				counterR--;
		}
	}

	if( PIN_CHANGED(pinNew, pinOld, PCINT1) ) { // flank change on channel B
		if (PIN_IS_HIGH(pinNew, PCINT1)) {	    // flank change from LOW to HIGH
			if (PIN_IS_HIGH(pinNew, PCINT0)) 	// A is HIGH
				counterR++;
			else 						        // A is LOW
				counterR--;
			
		} 
		else {							        // flank change from HIGH to LOW
			if (PIN_IS_HIGH(pinNew, PCINT0)) 	// A is HIGH
				counterR--;
			else 								// A is LOW
				counterR++;
		}
	}
   
	
	//Left Encoder
	if( PIN_CHANGED(pinNew, pinOld, PCINT2) ) { // flank change on channel A
		if (PIN_IS_HIGH(pinNew, PCINT2)) {	    // flank change from LOW to HIGH
			if (PIN_IS_HIGH(pinNew, PCINT3)) 	// B is HIGH
				counterL++;
			else 								// B is LOW
				counterL--;
			
		} 
		else {								    // flank change from HIGH to LOW
			if (PIN_IS_HIGH(pinNew, PCINT3)) 	// B is HIGH
				counterL--;
			else 								// B is LOW
				counterL++;
		}
	}

	if(PIN_CHANGED(pinNew, pinOld, PCINT3)) {   // flank change on channel B
		if (PIN_IS_HIGH(pinNew, PCINT3)) {	    // flank change from LOW to HIGH
			if (PIN_IS_HIGH(pinNew, PCINT2)) 	// A is HIGH
				counterL--;
			else 								// A is LOW
				counterL++;
		} 
		else {								    // flank change from HIGH to LOW
			if (PIN_IS_HIGH(pinNew, PCINT2)) 	// A is HIGH
				counterL++;
			else 								// A is LOW
				counterL--;
		}
	}

    pinOld = pinNew;    
 }

 int16_t encoder_getCountR(){
  return counterR;
 }

 int16_t encoder_getCountL(){
  return counterL;
 }
