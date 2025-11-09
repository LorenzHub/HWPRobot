#include "timeTask.h"

#include <avr/io.h>         // AVR IO ports
#include <avr/interrupt.h>  // AVR Interrupts
#include <util/atomic.h>

#include <stdbool.h>


/**
 * An array carrying the last and current uptime in milliseconds as
 * uint16_t. The array needs to be indexed by the bit GPIOR0_INDEX_BIT
 * in GPIOR0.
 * Two uptimes are used due to concurrency reasons as the array is written
 * asynchronously in interrupt context. This eliminates the need for
 * disabling interrupts when timeTask_uptime is read from main context.
 *
 * This array is internally used by the #TIMETASK macro.
 */
uint16_t timeTask_uptime[2] = { 0, 65535 };

/**
 * timeTask_time_ms counts the uptime in milliseconds for use of execution
 * time measurement. This value is incremented by timer 5 compare match
 * interrupt running at 1kHz.
 */
volatile uint32_t timeTask_time_ms = 0;

// Several flags (see lib/tools/timeTask/GPIOR0Defs.h) are kept in the register
// GPIOR0. In order to ensure proper initialization, GPIOR0 is cleared in the
// .init3 section right after the zero register (r1) has been cleared and the
// stack pointer was set up.
void __attribute__ ((naked)) __attribute__ ((section (".init3"))) init_GPIOR0(void) {
    GPIOR0 = 0;
}


// initialize timer 5 to fire interrupt every millisecond at an 8MHz clock
void timeTask_init(void) {
    // prevent multiple initializations
    if ((GPIOR0 & _BV(GPIOR0_INIT_BIT)) == 0) {
        GPIOR0 |= _BV(GPIOR0_INIT_BIT);

        // disable power reduction of timer 5
        PRR1 &= ~_BV(PRTIM5);

        TCCR5B = 0x00;  // stop timer 5 by selecting no clock

        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { // disable interrupts just in case because of TEMP register used by TCNT5 and OCR5A, see Section 17.3 of datasheet
            // set counter value to zero
            TCNT5 = 0;

            // set timer 5 to CTC mode (clear timer on compare match with OCR5A)
            // use prescaler of 8 => timer frequency of 1MHz
            // TOP = max value of counter is defined by OCR5A
            // set OCR5A to 1000 = 0x03E8, produces a compare match interrupt frequency of 1kHz = 1ms
            OCR5A = 0x03E8;
        }

        TCCR5C = 0x00; // no force output compare
        TCCR5A = 0x00; // WGM51=0, WGM50=0, COM5A0/1=0, COM5B0/1=0, COM5C0/1=0 (no compare output mode)
        TIMSK5 = _BV(OCIE5A); // timer 5 output compare A match interrupt enable
        TCCR5B = _BV(WGM52) | _BV(CS51); // CTC mode (TOP=OCR5A), Clock Select 1 => Prescaler 8, Timer frequency: 1MHz, start timer
    }
}


void timeTask_getTimestamp(timeTask_time_t* timestamp) {
    register uint16_t time_us;
    register uint32_t time_ms;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        time_us = TCNT5;
        time_ms = timeTask_time_ms;
    }

    timestamp->time_ms = time_ms;
    timestamp->time_us = time_us;
}


uint32_t timeTask_getDuration(const timeTask_time_t* startTime, const timeTask_time_t* stopTime) {
    return ((stopTime->time_ms - startTime->time_ms) * 1000) + ((int16_t)stopTime->time_us - (int16_t)startTime->time_us);
}
