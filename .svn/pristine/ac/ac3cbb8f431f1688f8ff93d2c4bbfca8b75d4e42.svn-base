// include ADC configuration
#include <cfg/io/adc/adc_cfg.h>

#include <avr/io.h>         // AVR IO ports

.extern ADC_currentChannel;
.extern ADC_channels;


/*
equivalent C-code compiles to 228 bytes, 169/180 clock cycles

ISR(ADC_vect) {
    // ADC conversion result is available

    // make a copy of ADC_currentChannel
    ADCChannel_t* c = ADC_currentChannel;

    // select next channel to convert
    if (++ADC_currentChannel == ADC_channels + ADC_CHANNEL_COUNT)
        ADC_currentChannel = &ADC_channels[0];

    // ADMUX is set for the next conversion as early as possible to give the
    // voltage some time to stabilize
    // configure ADC to sample the required channel
    ADMUX = ADC_currentChannel->admux;
    ADCSRB = ADC_currentChannel->adcsrb;

    // move position index in history array to next element
    uint8_t pos = (c->pos + 1) & (ADC_FILTER_SIZE - 1);
    c->pos = pos;

    uint16_t value = ADC; // read 10 bit ADC value

    // update sum of samples and history element
    c->sum = c->sum - c->history[pos] + value;
    c->history[pos] = value;

    // clear up-to-date flag
    c->channel &= 0x7F;
}

assembler implementation below uses 154 bytes, 110/115 clock cycles
*/


.global ADC_vect
ADC_vect:
    push r2
    in r2, _SFR_IO_ADDR(SREG)
    push r23
    push r24
    push r25
    push YL
    push YH
    push ZL
    push ZH

    // load ADC_currentChannel
    lds YL, ADC_currentChannel
    lds YH, ADC_currentChannel + 1
    // make a copy of ADC_currentChannel for later use
    movw ZL, YL

    // if (++ADC_currentChannel != ADC_channels + ADC_CHANNEL_COUNT)
    subi ZL, lo8(-(8 + 2 * ADC_FILTER_SIZE))
    sbci ZH, hi8(-(8 + 2 * ADC_FILTER_SIZE))
    ldi r24, hi8(ADC_channels + ADC_CHANNEL_COUNT * (8 + 2 * ADC_FILTER_SIZE))
    cpi ZL, lo8(ADC_channels + ADC_CHANNEL_COUNT * (8 + 2 * ADC_FILTER_SIZE))
    cpc ZH, r24
    breq ADC_RESET_CURRENT_CHANNEL

ADC_SET_CHANNEL:
    // ADMUX  = ADC_currentChannel->admux;
    // ADMUX is set for the next conversion as early as possible to give the
    // voltage some time to stabilize
    ldd r24, Z+1
    sts _SFR_MEM_ADDR(ADMUX), r24

    // ADCSRB  = ADC_currentChannel->adcsrb;
    ldd r24, Z+2
    sts _SFR_MEM_ADDR(ADCSRB), r24

    // store updated ADC_currentChannel
    sts ADC_currentChannel + 1, ZH
    sts ADC_currentChannel, ZL

    // Note: YH:YL contains "old" value of ADC_currentChannel to which result
    // of conversion corresponds to.

    // move position index in history array to next element
    // uint8_t pos = (ADC_currentChannel->pos + 1) & (ADC_FILTER_SIZE - 1);
    ld ZL, Y
    inc ZL
    andi ZL, ADC_FILTER_SIZE-1

    // ADC_currentChannel->pos = pos;
    st  Y, ZL

    // update sum of samples and history element
    // ADC_currentChannel->sum = ADC_currentChannel->sum - ADC_currentChannel->history[pos] + value;

    // calculate address of (ADC_currentChannel->history[pos]) - 8 in ZH:ZL
    eor ZH, ZH
    add ZL, ZL // ZH:ZL = pos*2
    adc ZH, ZH
    add ZL, YL // add address of ADC_currentChannel to ZH:ZL
    adc ZH, YH

    // interleaved execution of:
    //   load ADC_currentChannel->history[pos] (into r23,r24)
    //   uint16_t value = ADC; // read 10 bit ADC value (into r25, r25)
    //   ADC_currentChannel->history[pos] = value;
    //   subtract ADC_currentChannel->history[pos] - value
    ldd r23, Z+8
    lds r25, _SFR_MEM_ADDR(ADCL)
    std Z+8, r25
    sub r23, r25

    ldd r24, Z+9
    lds r25, _SFR_MEM_ADDR(ADCH)
    std Z+9, r25
    sbc r24, r25
    // result of subtraction as int16_t in r24:r23

    // convert int16_t to 24bit, result in r25:r24:r23
    mov r25, r24
    add r25, r25
    sbc r25, r25

    // interleaved execution of:
    //   load ADC_currentChannel->sum (24 bits only)
    //   subtract difference (history[pos]-value) from ADC_currentChannel->sum
    //   store updated value to ADC_currentChannel->sum
    ldd ZL, Y+4
    sub ZL, r23
    std Y+4, ZL

    ldd ZL, Y+5
    sbc ZL, r24
    std Y+5, ZL

    ldd ZL, Y+6
    sbc ZL, r25
    std Y+6, ZL

    // clear up-to-date flag (bit 7 of ADC_currentChannel->channel)
    ldd ZL, Y+3
    andi ZL, 0x7F
    std Y+3, ZL

    // starting a conversion is done in TIMER5_COMPA_vect
    // (see tools/timeTask/timeTask_isr.S)

    pop ZH
    pop ZL
    pop YH
    pop YL
    pop r25
    pop r24
    pop r23
    out _SFR_IO_ADDR(SREG), r2
    pop r2
    reti

ADC_RESET_CURRENT_CHANNEL:
    // ADC_currentChannel = &ADC_channels[0];
    ldi ZL, lo8(ADC_channels)
    ldi ZH, hi8(ADC_channels)
    rjmp ADC_SET_CHANNEL
