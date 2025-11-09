/**
 * @file powerSaver.h
 * @ingroup tools
 *
 * Power saving features for the ATmega1280.
 *
 * Power is saved by disabling all of the ATmega1280 features. If a
 * feature is required, it needs to be enabled explicitly by clearing the
 * corresponding bit in either PRR0 or PRR1. See the ATmega1280's datasheet
 * for details.
 *
 * Note that the call to powerSaver_init() must be the first call in
 * <code>main()</code>! This ensures that no feature gets disabled which
 * otherwise may have been initialized before.
 */

#ifndef POWERSAVER_H_
#define POWERSAVER_H_

#include <avr/io.h>


/**
 * Disable all of the ATmega1280's features to save power.
 * Note that the call to powerSaver_init() must be the first call in
 * <code>main()</code>!
 */
#define powerSaver_init() {                                                                                \
    /* shutdown: TWI, Timer 2, Timer 0, Timer 1, SPI, USART0, ADC */                                       \
    PRR0 = _BV(PRTWI) | _BV(PRTIM2) | _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRSPI) | _BV(PRUSART0) | _BV(PRADC); \
                                                                                                           \
    /* shutdown: Timer 5, Timer 4, Timer 3, USART3, USART2, USART1 */                                      \
    PRR1 = _BV(PRTIM5) | _BV(PRTIM4) | _BV(PRTIM3) | _BV(PRUSART3) | _BV(PRUSART2) | _BV(PRUSART1);        \
}


#endif /* POWERSAVER_H_ */
