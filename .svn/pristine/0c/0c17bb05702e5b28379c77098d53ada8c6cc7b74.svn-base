#include "GPIOR0Defs.h"
#include <cfg/io/adc/adc_cfg.h>

#include <avr/io.h>         // AVR IO ports

.extern timeTask_uptime;
.extern timeTask_time_ms;


/*
equivalent C-code compiles to 136 bytes, max. 89 clock cycles

ISR(TIMER5_COMPA_vect) {
    // timer 5 has reached the TOP value defined by OCR5A, i.e. 1ms has passed
    ++timeTask_time_ms;

    // toggle bit GPIOR0_INDEX_BIT in GPIOR0
    uint8_t idx = GPIOR0;
    idx ^= _BV(GPIOR0_INDEX_BIT);
    GPIOR0 = idx;

    // determine array index of timeTask_uptime
    idx = (idx & _BV(GPIOR0_INDEX_BIT)) >> GPIOR0_INDEX_BIT;
    // increase the uptime at idx by 2
    timeTask_uptime[idx] += 2;

    // if ADC is enabled (bit GPIOR0_ADC_BIT of GPIOR0 is 1), start a conversion every 2 milliseconds
    if (((timeTask_uptime[idx] % 2) == 0) && ((GPIOR0 & _BV(GPIOR0_ADC_BIT)) != 0)) {
        // start ADC conversion of the next channel using the configuration described
        // by ADC_ADCSRA_CONFIG (see io/adc/adc.h for details)
        ADCSRA = ADC_ADCSRA_CONFIG;
    }
}

assembler implementation below uses 108 bytes, max. 72 clock cycles
*/


.global TIMER5_COMPA_vect
TIMER5_COMPA_vect:
    push r2
    in r2, _SFR_IO_ADDR(SREG)
    push YL
    push YH
    push ZL
    push ZH

    // timer 5 has reached the TOP value defined by OCR5A, i.e. 1ms has passed

    // ++timeTask_time_ms;
    eor ZH, ZH
    // load two lower bytes from uin32_t
    lds YL, timeTask_time_ms
    lds YH, timeTask_time_ms + 1
    // increment YH:YL by one
    adiw YL, 1
    // store result for two lower bytes
    sts timeTask_time_ms, YL
    sts timeTask_time_ms + 1, YH
    // load two higher bytes from uin32_t
    lds YL, timeTask_time_ms + 2
    lds YH, timeTask_time_ms + 3
    // add carry in case of overflow from lower bytes
    adc YL, ZH
    adc YH, ZH
    // store result for two higher bytes
    sts timeTask_time_ms + 2, YL
    sts timeTask_time_ms + 3, YH

    // if bit GPIOR0_INDEX_BIT in GPIOR0 is set, jump to TIMER5_COMPA_INDEX_1
    sbic _SFR_IO_ADDR(GPIOR0), GPIOR0_INDEX_BIT
    rjmp TIMER5_COMPA_INDEX_1

    // bit is 0, set it (idx = 1)
    sbi _SFR_IO_ADDR(GPIOR0), GPIOR0_INDEX_BIT
    // load address of timeTask_uptime[1]
    ldi ZL, lo8(timeTask_uptime + 2)
    ldi ZH, hi8(timeTask_uptime + 2)
    rjmp TIMER5_COMPA_UPTIME

TIMER5_COMPA_INDEX_1:
    // bit is 1, clear it (idx = 0)
    cbi _SFR_IO_ADDR(GPIOR0), GPIOR0_INDEX_BIT
    // load address of timeTask_uptime[0]
    ldi ZL, lo8(timeTask_uptime)
    ldi ZH, hi8(timeTask_uptime)

TIMER5_COMPA_UPTIME:
    // load uint16_t at timeTask_uptime[idx]
    ld YL, Z
    ldd YH, Z+1
    // increment YH:YL by 2
    adiw YL, 2
    // store back
    std Z+1, YH
    st Z, YL

    // if updated value of timeTask_uptime[idx] is odd, jump to end of this ISR
    // otherwise, start a conversion if ADC is enabled, i.e. every 2 milliseconds
    sbrc YL, 0
    rjmp TIMER5_COMPA_FINISH

    // if ADC is disabled (bit GPIOR0_ADC_BIT of GPIOR0 is cleared), jump to end of this ISR
    sbis _SFR_IO_ADDR(GPIOR0), GPIOR0_ADC_BIT
    rjmp TIMER5_COMPA_FINISH

    // start ADC conversion of the next channel using the configuration described
    // by ADC_ADCSRA_CONFIG (see io/adc/adc.h for details)
    ldi ZL, ADC_ADCSRA_CONFIG
    sts _SFR_MEM_ADDR(ADCSRA), ZL

TIMER5_COMPA_FINISH:
    pop ZH
    pop ZL
    pop YH
    pop YL
    out _SFR_IO_ADDR(SREG), r2
    pop r2
    reti

