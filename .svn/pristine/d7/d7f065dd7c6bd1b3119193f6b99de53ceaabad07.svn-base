#include <io/led/led.h>

#include <avr/interrupt.h>          // AVR Interrupts
#include <util/delay.h>             // AVR Delay functions

/**
 * By default, undefined ISRs have the common implementation to abort the program,
 * i.e. the microcontroller executes an infinite loop. Thus, if an interrupt fires
 * for which no ISR has been defined, the program would terminate without giving
 * some feedback to the user.
 *
 * With BADISR_vect, an implementation for all undefined ISRs can be given, which
 * effectively serves as a catch-all. The corresponding ISR is executed whenever
 * an interrupt fires which has no ISR explicitly defined in the code.
 *
 * This implementation just enters an infinite loop and toggles both LEDs
 * alternately with a delay of 100ms in order to give the user a feedback.
 */
ISR(BADISR_vect) {
    LED1_ON();
    LED2_OFF();

    for (;;) {
        _delay_ms(100);
        LED1_TOGGLE();
        LED2_TOGGLE();
    }
}
