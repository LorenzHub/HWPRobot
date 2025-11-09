/**
 * @file adc_cfg.h
 * @ingroup cfg
 *
 * User configuration for ADC library.
 */

#ifndef ADC_CFG_H_
#define ADC_CFG_H_


/**
 *  Number of ADC channels to be sampled.
 *  Corresponds to number of entries in array ADC_channels[], see #ADC_CHANNEL_INIT
 */
#define ADC_CHANNEL_COUNT 2


/**
 *  Maps the #ADC_CHANNEL_COUNT virtual channel numbers to the ADMUX register
 *  values of the physical channels for single-ended inputs. The values 0-15
 *  for the physical channel numbers correspond to pins ADC0 to ADC15.
 *  Notation:
 *  <code>ADC_channels[VIRTUAL_CHANNEL_NUMBER].channel = PHYSICAL_CHANNEL_NUMBER</code>,
 *
 *  All virtual channel numbers from 0 up to <code>ADC_CHANNEL_COUNT-1</code>
 *  must be set!
 *
 *  See ADMUX register description for further details on input channel
 *  selection.
 */
#define ADC_CHANNEL_INIT {   \
    ADC_channels[0].channel = 0; \
    ADC_channels[1].channel = 1; \
}


/**
 *  Size of the moving average filter for each ADC channel.
 *  Value must be a power of 2 with a maximum of 256.
 *  For implications of the filter and conversion timing, please see
 *  lib/io/adc/adc.h
 */
#define ADC_FILTER_SIZE 1


/**
 * ADC_ADCSRA_CONFIG specifies the value for the ADCSRA register which contains
 * the configuration of the ADC in terms of conversion mode, prescaler and
 * enabling the ADC complete interrupt. It is also responsible for enabling the
 * ADC and starting a conversion.
 *
 * This value is internally used by TIMER5_COMPA_vect (timer 5 compare match
 * interrupt, see lib/tools/timeTask/timeTask_isr.S) to start the conversion
 * of the next channel every two milliseconds.
 *
 * The configuration is as follows:
 * - single conversion mode
 * - single-ended inputs
 * - ADC complete interrupt is enabled
 * - prescaler of 128 results into a ADC clock frequency of 62.500kHz
 *   + normal ADC conversion takes 13 ADC clock cycles, i.e. 208us for a single channel
 *   + first ADC conversion takes 25 ADC clock cycles, i.e. 400us
 */
#define ADC_ADCSRA_CONFIG (_BV(ADEN) | _BV(ADSC) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0))


#endif /* ADC_CFG_H_ */
