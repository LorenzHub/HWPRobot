/**
 * @file timeTask.h
 * @ingroup tools
 *
 * Functions for timed execution of code and measurement of execution time.
 *
 * Timer 5 is used for measuring the uptime. Its output compare match ISR
 * <code>TIMER5_COMPA_vect</code> is fired every millisecond.
 *
 * Note that the ISR <code>TIMER5_COMPA_vect</code> also has the responsibility
 * of starting AD conversions if the ADC library has been initialized by
 * ADC_init(). See lib/io/adc/adc.h for further details.
 *
 * Several bits of the register GPIOR0 are used for different purposes. The
 * bits are defined in lib/tools/timeTask/GPIOR0Defs.h.
 * - The bit defined by #GPIOR0_ADC_BIT determines if the ADC has been initialized
 *   and is activated (modified by ADC_init(), ADC_pause() and ADC_resume()).
 *   If this flag is set, ADC conversions are triggered by the ISR
 *   <code>TIMER5_COMPA_vect</code>.
 * - The bit defined by #GPIOR0_INIT_BIT serves as a flag which determines
 *   if timeTask_init() has already been called in order to prevent multiple
 *   initializations.
 * - The bit defined by #GPIOR0_INDEX_BIT serves an array index (which can only
 *   be 0 or 1) into an internally managed array timeTask_uptime[2]. This array,
 *   keeping the current and the previous uptime, and the index bit are written
 *   asynchronously in interrupt context. This information is used by the
 *   #TIMETASK macro when reading the current uptime in milliseconds. With this
 *   double buffering, there is no need to disable interrupts in main context
 *   when reading from the array, as access to the array index is atomic.
 *
 * The register GPIOR0 is cleared during startup by placing appropriate code
 * into the .init3 section.
 */

#ifndef TIMETASK_H_
#define TIMETASK_H_

#include "GPIOR0Defs.h"

#include <stdint.h>

#include <avr/io.h>         // AVR IO ports


/**
 * Execution of body block every 'cycles' loop cycles.
 * Needs NO initialization via timeTask_init() and hence no timer.
 *
 * <b>Usage:</b>
 * @code
 * for(;;) { // main loop
 *
 *     CYCLETASK(T1, 1000) { // execute every 1000 loop cycles
 *         // do some stuff
 *         static int16_t j = 0;
 *         j++;
 *         communication_log(LEVEL_INFO, "T1: %d\n", j);
 *      }
 *
 *      CYCLETASK(T2, 333) { // execute every 333 loop cycles
 *          // do some other stuff
 *          static int16_t j = 0;
 *          j++;
 *          communication_log(LEVEL_INFO, "T2: %d\n", j);
 *      }
 *  }
 *  @endcode
 */
#define CYCLETASK(name, cycles) static uint32_t name = 0; if (! (name = (name > cycles) ? 0 : (name + 1)))


/**
 * Timestamp structure which holds a timstamp acquired by timeTask_getTimestamp().
 *
 * The timestamp consists of a milliseconds field which stores the uptime in the range from 0 to 4294967295ms.
 * The microseconds field holds the microseconds in the range 0 to 999.
 * Thus, the range of the timestamp is from 0 to 4294967295999 microseconds (about
 * 1193 hours, before the timestamp overflows).
 *
 * The duration between two timestamps can be determined with timeTask_getDuration().
 */
typedef struct {
    uint32_t time_ms; ///< milliseconds in the range 0 to 4294967295
    uint16_t time_us; ///< microseconds in the range 0 to 999
} timeTask_time_t;


/**
 * Initialization for the #TIMETASK macro and for execution time measurement.
 * Uses timer 5 to fire output compare match interrupt every millisecond.
 * Global interrupts must be enabled manually after calling this function.
 *
 * Note that timeTask_init() is also called by ADC_init() to ensure that AD
 * conversions are started by <code>TIMER5_COMPA_vect</code>.
 */
void timeTask_init(void);


/**
 * Execution of body block with a minimum delay of 'interval_ms' milliseconds in
 * between.
 * Requires initialization via timeTask_init(). The function is robust concerning
 * overflow of uptime variable, since comparison with interval_ms is done as
 * unsigned using the full range of the variable.
 * Note that the actual interval may be larger and varying due to execution time of
 * other code in main loop as well as frequency and duration of interrupt service
 * routines.
 *
 * The maximum interval is 65535ms!
 *
 * <b>Usage:</b>
 * @code
 * for(;;) { // main loop
 *
 *     TIMETASK(T1, 100) { // execute block approximately every 100ms
 *         // do some stuff
 *         static int16_t j = 0;
 *         j++;
 *         communication_log(LEVEL_INFO, "T1: %d\n", j);
 *      }
 *
 *      TIMETASK(T2, 200) { // execute block approximately every 200ms
 *          // do some other stuff
 *          static int16_t j = 0;
 *          j++;
 *          communication_log(LEVEL_INFO, "T2: %d\n", j);
 *      }
 *  }
 *  @endcode
 */
#define TIMETASK(name, interval_ms)                                      \
    static uint16_t name = 0;                                            \
    register uint16_t tt_uptime_##name = timeTask_getUptime();           \
    register uint8_t tt_ex_##name = 0;                                   \
    if ((tt_uptime_##name - name) >= (uint16_t)interval_ms) {            \
        name = tt_uptime_##name;                                         \
        tt_ex_##name = 1;                                                \
    }                                                                    \
    if (tt_ex_##name)


/**
 * Get current uptime in milliseconds.
 * This function is internally used by the #TIMETASK macro when checking
 * if the time task needs to be executed.
 *
 * @return  current uptime in milliseconds
 */
static inline uint16_t __attribute__((always_inline)) timeTask_getUptime(void) {
    extern uint16_t timeTask_uptime[2];

    uint16_t uptime;

    asm volatile (
        "sbic %[INDEX], %[INDEX_BIT]"    "\n\t" // skip if bit GPIOR0_INDEX_BIT in GPIOR0 is cleared
        "rjmp L_getUptimeIndex1%="       "\n\t" // if bit is set, jump to L_getUptimeIndex1%=
        "lds %A[UPTIME], %[ADDR]"        "\n\t" // load low byte of timeTask_uptime[0]
        "lds %B[UPTIME], (%[ADDR])+1"    "\n\t" // load high byte of timeTask_uptime[0]
        "rjmp L_getUptimeEnd%="          "\n\t" // jump to L_getUptimeEnd%=
        "L_getUptimeIndex1%=:"           "\n\t"
        "lds %A[UPTIME], (%[ADDR])+2"    "\n\t" // load low byte of timeTask_uptime[1]
        "lds %B[UPTIME], (%[ADDR])+3"    "\n\t" // load high byte of timeTask_uptime[1]
        "L_getUptimeEnd%=:"              "\n\t"
        : [UPTIME] "=r" (uptime)
        : [ADDR] "i" (timeTask_uptime),
          [INDEX] "n" (_SFR_IO_ADDR(GPIOR0)),
          [INDEX_BIT] "n" (GPIOR0_INDEX_BIT)
        : // no clobber
    );

    return uptime;
}


/**
 * Get a timestamp for execution time measurement.
 *
 * This functions captures the uptime with a resolution of 1 microsecond.
 *
 * Use this function in conjunction with timeTask_getDuration() to get the
 * duration between two timestamps.
 *
 * @param   timestamp receives the current timestamp
 */
void timeTask_getTimestamp(timeTask_time_t* timestamp);


/**
 * Calculate the duration between timestamps stopTime and startTime.
 *
 * stopTime must represent a larger value than startTime. No overflow
 * of the timestamps are taken into account (which happens after an
 * uptime of about 1193 hours).
 * Although the timestamps have a larger range (see timeTask_time_t),
 * the maximum duration between two timestamps is limited by
 * 4294967295us (about 1.19 hours).
 *
 * @param   startTime first timestamp
 * @param   stopTime second timestamp (larger than startTime)
 * @return  duration in microseconds (max. value is 4294967295us)
 */
uint32_t timeTask_getDuration(const timeTask_time_t* startTime, const timeTask_time_t* stopTime);

#endif /* TIMETASK_H_ */
