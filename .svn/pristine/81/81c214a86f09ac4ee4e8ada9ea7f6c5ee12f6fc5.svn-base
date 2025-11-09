/**
 * @file led.h
 * @ingroup io
 *
 * Access the user LEDs on the HWP board
 */

#ifndef LED_H_
#define LED_H_

#include <avr/io.h>


/**
 * Definition of pin for LEDMC1
 */
#define LED1_PIN         PA7


/**
 * Definition of pin for LEDMC2
 */
#define LED2_PIN         PA6


/**
 * Definition of DDR register for LEDs
 */
#define LED_DDR         DDRA


/**
 * Definition of PORT register for LEDs
 */
#define LED_PORT        PORTA


/**
 * Switch on LEDMC1
 */
#define LED1_ON()        LED_PORT |= _BV(LED1_PIN)


/**
 * Switch off LEDMC1
 */
#define LED1_OFF()       LED_PORT &= ~_BV(LED1_PIN)


/**
 * Toggle LEDMC1
 */
#define LED1_TOGGLE()    LED_PORT ^= _BV(LED1_PIN)


/**
 * Switch on LEDMC2
 */
#define LED2_ON()        LED_PORT |= _BV(LED2_PIN)


/**
 * Switch off LEDMC2
 */
#define LED2_OFF()       LED_PORT &= ~_BV(LED2_PIN)


/**
 * Toggle LEDMC2
 */
#define LED2_TOGGLE()    LED_PORT ^= _BV(LED2_PIN)


/**
 * Initialize the LEDs.
 *
 * Configure the LED pins as output by writing to LED_DDR.
 */
#define LED_init()      LED_DDR |= _BV(LED1_PIN) | _BV(LED2_PIN)


#endif /* LED_H_ */
