/**
 * @file GPIOR0Defs.h
 * @ingroup tools
 *
 * Definitions for timeTask and ADC functions and interrupts concerning the use of the register GPIOR0
 */

#ifndef GPIOR0DEFS_H_
#define GPIOR0DEFS_H_

/**
 * Definition of bit in GPIOR0 used by #TIMETASK macro and ISR
 * <code>TIMER5_COMPA_vect</code>.
 * The bit defined by #GPIOR0_INDEX_BIT serves an array index (which can only
 * be 0 or 1) into an internally managed array timeTask_uptime[2]. This array,
 * keeping the current and the previous uptime, and the index bit are written
 * asynchronously in interrupt context. This information is used by the
 * #TIMETASK macro when reading the current uptime in milliseconds. With this
 * double buffering, there is no need to disable interrupts in main context
 * when reading from the array, as access to the array index is atomic.
 */
#define GPIOR0_INDEX_BIT 0


/**
 * Definition of bit in GPIOR0 used by timeTask_init().
 * The bit defined by GPIOR0_INIT_BIT serves as a flag which determines
 * if timeTask_init() has already been called in order to prevent multiple
 * initializations.
*/
#define GPIOR0_INIT_BIT 1


/**
 * Definition of bit in GPIOR0 used by ADC functions and the ISR
 * <code>TIMER5_COMPA_vect</code>.
 * The bit defined by GPIOR0_ADC_BIT determines if the ADC has been initialized
 * and is activated (modified by ADC_init(), ADC_pause() and ADC_resume()).
 * If this flag is set, ADC conversions are triggered by the ISR
 * <code>TIMER5_COMPA_vect</code>.
*/
#define GPIOR0_ADC_BIT 2


#endif /* GPIOR0DEFS_H_ */
