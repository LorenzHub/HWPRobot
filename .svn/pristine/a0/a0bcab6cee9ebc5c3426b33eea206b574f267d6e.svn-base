#include "adc.h"
#include <communication/communication.h>
#include <tools/timeTask/timeTask.h>

#include <avr/interrupt.h>
#include <util/atomic.h>

#include <string.h>


// data structure for ADC channel configurations and moving average filter for each channel
typedef struct __attribute__((__packed__)) {
    volatile uint8_t pos;                       // index in history array with the latest sample
    uint8_t admux;                              // value for the ADMUX register for sampling this channel
    uint8_t adcsrb;                             // value for the ADCSRB register for sampling this channel
    uint8_t channel;                            // physical ADC channel in bits 0-3 with values: 0 => ADC0, 1 => ADC1, ..., 15 => ADC15; bit
    volatile uint32_t sum;                      // sum of the latest ADC_FILTER_SIZE samples
    volatile uint16_t history[ADC_FILTER_SIZE]; // storage for the latest ADC_FILTER_SIZE samples
} ADCChannel_t;


// moving average filter for all channels
ADCChannel_t ADC_channels[ADC_CHANNEL_COUNT];

// pointer into channels array with currently sampled channel
ADCChannel_t* ADC_currentChannel;


void ADC_init(const bool disableJTAG) {
    if (disableJTAG) {
        // atomically disable the JTAG interface
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            // read MCUCR and set bit for disabling JTAG
            uint8_t mcucr = MCUCR | _BV(JTD);

            // New value for MCUCR must be written twice within four clock cycles
            // in order to have JTAG disabled
            MCUCR = mcucr;
            MCUCR = mcucr;
        }
    }

    // disable power reduction of ADC
    PRR0 &= ~_BV(PRADC);

    // disable ADC for now
    ADCSRA = 0x00;
    // switch off analog comparator
    ACSR |= _BV(ACD);

    #if (ADC_FILTER_SIZE > 256)
        #error ADC_FILTER_SIZE exceeds limit of 256 bytes
    #endif
    #if (ADC_FILTER_SIZE & (ADC_FILTER_SIZE - 1))
        #error ADC_FILTER_SIZE is not a power of 2
    #endif

    // initialize data structure
    memset((void*)(ADC_channels), 0, sizeof(ADC_channels));

    ADC_currentChannel = &ADC_channels[0];

    // set mapping between virtual and physical ADC channels in data structure
    ADC_CHANNEL_INIT

    // Configure ADC_channels[i].admux and ADC_channels[i].adcsrb for all channels.
    // Also disable digital input buffers for used ADC channels in order to save
    // some power, especially if analog voltage is close to Vcc/2
    uint8_t
        didr0 = 0,
        didr2 = 0;
    for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; ++i) {
        // clear up-to-date flag and constrain channel numbers to range from 0 to 15
        ADC_channels[i].channel &= 0x0F;

        register uint8_t
            physicalChannel = ADC_channels[i].channel,
            physicalChannelAND0x07 = physicalChannel & 0x07;

        // voltage reference for ADC is AVCC pin (connected to 3.3V)
        ADC_channels[i].admux = _BV(REFS0) | physicalChannelAND0x07;

        if (physicalChannel & _BV(3)) {
            didr2 |= _BV(physicalChannelAND0x07);
            // if channel > 7, MUX5 (ADCSRB bit 3) has to be set, otherwise cleared
            ADC_channels[i].adcsrb = _BV(MUX5);
        } else {
            didr0 |= _BV(physicalChannelAND0x07);
            // note: ADC_channels[i].adcsrb = 0 per initialization
        }
    }
    DIDR0 = didr0;
    DIDR2 = didr2;

    // configure ADC to sample the first channel
    ADMUX = ADC_channels[0].admux;
    ADCSRB = ADC_channels[0].adcsrb;

    // ADC is now fully setup and the next run of TIMER5_COMPA_vect can start
    // the conversion. Bit GPIOR0_ADC_BIT of GPIOR0 is set to 1
    GPIOR0 |= _BV(GPIOR0_ADC_BIT);

    // initialize timeTask since starting of ADC conversion is triggered
    // by compare match interrupt of timer 5
    timeTask_init();
}


uint16_t ADC_getFilteredValue(const uint8_t channel) {
    // perform atomic read on ADC_channels[channel].sum

    ADCChannel_t *c = &ADC_channels[channel];

    // Tell the compiler that we want to use variable c now, although we don't
    // do this. This memory barrier allows the placement of the code for
    // accessing ADC_channels[channel] before interrupts are disabled in the atomic
    // block. Otherwise the compiler would place this code after cli. This
    // ensures that interrupts are disabled for the smallest possible time.
    asm volatile ("" : : "r" (c) : "memory");

    uint32_t sum;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        sum = c->sum;
    }

    // calculate average
    return (uint16_t)(sum / ADC_FILTER_SIZE);
}


uint16_t ADC_getLastValue(const uint8_t channel) {
    // read ADC_channels[channel].history[pos]

    // Note that access to the array index pos is atomic which alleviates the need
    // for disabling interrupts when reading from the history array.

    ADCChannel_t *c = &ADC_channels[channel];
    return c->history[c->pos];
}


bool ADC_isUpToDate(const uint8_t channel) {
    return (ADC_channels[channel].channel & _BV(7)) != 0;
}


void ADC_setUpToDate(const uint8_t channel) {
    ADC_channels[channel].channel |= _BV(7);
}
