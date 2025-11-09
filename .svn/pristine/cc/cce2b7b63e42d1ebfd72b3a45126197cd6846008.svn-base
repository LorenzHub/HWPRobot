#include "motor.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

void Motor_init(void) {
    // Configure timer 3 (for motor A) and timer 4 (for motor B)
    // to produce PWM signals on OC3A/OC3B and OC4A/OC4B with a
    // frequency of 488.759Hz in phase correct PWM mode.
    // Duty cycle is defined by OC4A/OC4B and OC3A/OC3B, respectively.
    // OC3A/OC3B and OC4A/OC4B are connected to the motor driver TI DRV8833
    // on pins AIN1/AIN2 and BIN2/BIN1, respectively.

    // Due to the phase correct PWM modes, updated values of OC3A/OC3B and
    // OC4A/OC4B are buffered and taken over only once during a PWM period
    // when the counter reaches TOP (0x1FFF).

    // DRV8833 requires two inputs for controlling a single motor with the
    // following logic:
    //
    //     A/BIN1 | A/BIN2 | A/BOUT1 | A/BOUT2 | Motor A/B
    //     -----------------------------------------------
    //     Low    | Low    | Z       | Z       | Coast/Bridge disabled (fast decay)
    //     High   | Low    | High    | Low     | Forward
    //     Low    | High   | Low     | High    | Reverse
    //     High   | High   | Low     | Low     | Break/Motor shorted (slow decay)
    //
    // For controlling a motor with a PWM signal, the slow-decay mode is used
    // since this provides a quite linear relationship between PWM duty cycle
    // and motor speed (under constant supply voltage and constant load conditions).
    // The input pins are driven as follows:
    //
    //     A/BIN1 | A/BIN2 | Motor A/B
    //     -----------------------------------------------
    //     High   | PWM    | Forward
    //     PWM    | High   | Reverse
    //
    // For stopping the motor, the fast-decay mode is chosen (Low on both inputs)
    // as this disables the H-bridge and the outputs A/BOUT1 and A/BOUT2 are of
    // high impedance.
    // Thus, stopping a motor in fast-decay mode with inputs both low is a difference
    // to stopping the motor in slow-decay mode with one input held high and the other
    // with a PWM signal of 100% duty cycle.
    //
    // Speed of motor is influenced by duty cycle of PWM signals but may change
    // due to variation in supply voltage and motor load.

    // disable power reduction of timers 3 and 4
    PRR1 &= ~(_BV(PRTIM3) | _BV(PRTIM4));

    // stop timers 3 and 4 by selecting no clock
    TCCR3B = 0;
    TCCR4B = 0;

    // set timers to phase correct PWM mode (dual slope operation)
    // ICRn defines TOP value (max value of counter) and is set to 0x1FFF (13 bits)
    // use prescaler of N=1 => timer frequency equals CPU clock of 8MHz
    // PWM frequency: 8MHz / (2*N*TOP) = 488.759Hz

    // WGMn1 (in TCCRnA) and WGMn3 (in TCCRnB): phase correct PWM mode (dual slope operation) with TOP defined by ICRn
    // COMnA1, COMnA0: set OCnA on compare match with OCRnA when up-counting, clear OCnA on compare match with OCRnA when down-counting
    // COMnB1, COMnB0: set OCnB on compare match with OCRnB when up-counting, clear OCnB on compare match with OCRnB when down-counting
    TCCR3A = _BV(COM3A1) | _BV(COM3A0) | _BV(COM3B1) | _BV(COM3B0) | _BV(WGM31);
    TCCR4A = _BV(COM4A1) | _BV(COM4A0) | _BV(COM4B1) | _BV(COM4B0) | _BV(WGM41);

    // no force output compare
    TCCR3C = 0;
    TCCR4C = 0;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // disable interrupts just in case because of TEMP register used by TCNTn, OCRnA and ICRn, see Section 17.3 of datasheet

        // set counter values to zero
        TCNT3 = 0;
        TCNT4 = 0;

        // set TOP to 0x1FFF (8191)
        ICR3 = 0x1FFF;
        ICR4 = 0x1FFF;

        // set OCRnA and OCRnB to TOP in order to have motors stopped in fast-decay mode after timers will be enabled
        OCR3A = 0x1FFF;
        OCR3B = 0x1FFF;
        OCR4A = 0x1FFF;
        OCR4B = 0x1FFF;

        // configure OC3A (PE3, AIN1) and OC3B (PE4, AIN2) as output for motor A
        DDRE |= _BV(DDE3) | _BV(DDE4);
        // configure OC4A (PH3, BIN2) and OC4B (PH4, BIN1) as output for motor B
        DDRH |= _BV(DDH3) | _BV(DDH4);

        // Selecting the clock starts the timer and generation of PWM signals.
        // A prescaler reset is performed while the clock sources are set.
        // Disabling interrupts during timer activation ensures that the prescaler
        // reset takes the smallest possible time as all timers except timer 2 are
        // affected. Then, the prescalers are activated simultaneously, so that
        // both timers start at the same time and updated values of the OCR registers
        // are taken over simultaneously during normal operation.

        // Perform prescaler reset
        GTCCR = _BV(TSM) | _BV(PSRSYNC);
        // Select clock sources and prescaler
        // CSn0: prescaler 1
        // WGMn3: phase correct PWM mode with TOP defined by ICRn
        TCCR3B = _BV(CS30) | _BV(WGM33);
        TCCR4B = _BV(CS40) | _BV(WGM43);
        // Enable prescaler, timers start running simultaneously
        GTCCR = 0;
    }
}


__attribute__ ((naked)) void Motor_setPWM_A(const int16_t pwm) {
    asm volatile (
        "sbrc %B[pwm], 7"               "\n\t" // if pwm < 0
        "rjmp L_pwmneg%="               "\n\t" //   then goto L_pwmneg
        "andi %B[pwm], 0x1F"            "\n\t" // pwm & 0x1FFF
        "in __tmp_reg__, __SREG__"      "\n\t" // save status register in temporary register
        "cli"                           "\n\t" // disable interrupts
        "sts %B[POCR3A], __zero_reg__"  "\n\t" // OCR3AH = 0, high byte first (OC3A high, AIN1)
        "sts %A[POCR3A], __zero_reg__"  "\n\t" // OCR3AL = 0, then low byte (OC3A high, AIN1)
        "sts %B[POCR3B], %B[pwm]"       "\n\t" // OCR3B = pwm & 0x1FFF, high byte first
        "sts %A[POCR3B], %A[pwm]"       "\n\t" // OCR3B = pwm & 0x1FFF, then low byte
        "out __SREG__, __tmp_reg__"     "\n\t" // restore status register
        "ret"                           "\n\t" // return
    "L_pwmneg%=:"                       "\n\t"
        "neg %B[pwm]"                   "\n\t" // two's complement for high byte of pwm
        "neg %A[pwm]"                   "\n\t" // two's complement for low byte of pwm
        "sbc %B[pwm], __zero_reg__"     "\n\t" // subtract 1 from high byte, if carry flag is set
        "andi %B[pwm], 0x1F"            "\n\t" // (-pwm) & 0x1FFF
        "in __tmp_reg__, __SREG__"      "\n\t" // save status register in temporary register
        "cli"                           "\n\t" // disable interrupts
        "sts %B[POCR3B], __zero_reg__"  "\n\t" // OCR3BH = 0, high byte first (OC3B high, AIN2)
        "sts %A[POCR3B], __zero_reg__"  "\n\t" // OCR3BL = 0, then low byte (OC3B high, AIN2)
        "sts %B[POCR3A], %B[pwm]"       "\n\t" // OCR3A = (-pwm) & 0x1FFF, high byte first
        "sts %A[POCR3A], %A[pwm]"       "\n\t" // OCR3A = (-pwm) & 0x1FFF, then low byte
        "out __SREG__, __tmp_reg__"     "\n\t" // restore status register
        "ret"                           "\n\t" // return
        : // no outputs
        : [POCR3A] "n" (& OCR3A),
          [POCR3B] "n" (& OCR3B),
          [pwm] "d" (pwm)
        : // no clobber
    );

    // almost equivalent C code when compiling with -O2
    //    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    //        if (pwm < 0) {
    //            OCR3B = 0; // OC3B high, AIN2
    //            OCR3A = (-pwm) & 0x1FFF;
    //        } else {
    //            OCR3A = 0; // OC3A high, AIN1
    //            OCR3B = pwm & 0x1FFF;
    //        }
    //    }
}


__attribute__ ((naked)) void Motor_setPWM_B(const int16_t pwm) {
    asm volatile (
        "sbrc %B[pwm], 7"               "\n\t" // if pwm < 0
        "rjmp L_pwmneg%="               "\n\t" //   then goto L_pwmneg
        "andi %B[pwm], 0x1F"            "\n\t" // pwm & 0x1FFF
        "in __tmp_reg__, __SREG__"      "\n\t" // save status register in temporary register
        "cli"                           "\n\t" // disable interrupts
        "sts %B[POCR4A], __zero_reg__"  "\n\t" // OCR4AH = 0, high byte first (OC4A high, BIN2)
        "sts %A[POCR4A], __zero_reg__"  "\n\t" // OCR4AL = 0, then low byte (OC4A high, BIN2)
        "sts %B[POCR4B], %B[pwm]"       "\n\t" // OCR4B = pwm & 0x1FFF, high byte first
        "sts %A[POCR4B], %A[pwm]"       "\n\t" // OCR4B = pwm & 0x1FFF, then low byte
        "out __SREG__, __tmp_reg__"     "\n\t" // restore status register
        "ret"                           "\n\t" // return
    "L_pwmneg%=:"                       "\n\t"
        "neg %B[pwm]"                   "\n\t" // two's complement for high byte of pwm
        "neg %A[pwm]"                   "\n\t" // two's complement for low byte of pwm
        "sbc %B[pwm], __zero_reg__"     "\n\t" // subtract 1 from high byte, if carry flag is set
        "andi %B[pwm], 0x1F"            "\n\t" // (-pwm) & 0x1FFF
        "in __tmp_reg__, __SREG__"      "\n\t" // save status register in temporary register
        "cli"                           "\n\t" // disable interrupts
        "sts %B[POCR4B], __zero_reg__"  "\n\t" // OCR4BH = 0, high byte first (OC4B high, BIN1)
        "sts %A[POCR4B], __zero_reg__"  "\n\t" // OCR4BL = 0, then low byte (OC4B high, BIN1)
        "sts %B[POCR4A], %B[pwm]"       "\n\t" // OCR4A = (-pwm) & 0x1FFF, high byte first
        "sts %A[POCR4A], %A[pwm]"       "\n\t" // OCR4A = (-pwm) & 0x1FFF, then low byte
        "out __SREG__, __tmp_reg__"     "\n\t" // restore status register
        "ret"                           "\n\t" // return
        : // no outputs
        : [POCR4A] "n" (& OCR4A),
          [POCR4B] "n" (& OCR4B),
          [pwm] "d" (pwm)
        : // no clobber
    );

    // almost equivalent C code when compiling with -O2
    //    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    //        if (pwm < 0) {
    //            OCR4B = 0; // OC4B high, BIN1
    //            OCR4A = (-pwm) & 0x1FFF;
    //        } else {
    //            OCR4A = 0; // OC4A high, BIN2
    //            OCR4B = pwm & 0x1FFF;
    //        }
    //    }
}


__attribute__ ((naked)) void Motor_setPWM(const int16_t pwmA, const int16_t pwmB) {
    asm volatile (
        "sbrc %B[pwmA], 7"              "\n\t" // if pwmA < 0
        "rjmp L_pwmAneg%="              "\n\t" //    then goto L_pwmAneg
        "andi %B[pwmA], 0x1F"           "\n\t" // pwmA & 0x1FFF
        "in __tmp_reg__, __SREG__"      "\n\t" // save status register in temporary register
        "cli"                           "\n\t" // disable interrupts
        "sts %B[POCR3A], __zero_reg__"  "\n\t" // OCR3AH = 0, high byte first (OC3A high, AIN1)
        "sts %A[POCR3A], __zero_reg__"  "\n\t" // OCR3AL = 0, then low byte (OC3A high, AIN1)
        "sts %B[POCR3B], %B[pwmA]"      "\n\t" // OCR3B = pwmA & 0x1FFF, high byte first
        "sts %A[POCR3B], %A[pwmA]"      "\n\t" // OCR3B = pwmA & 0x1FFF, then low byte
        "sbrc %B[pwmB], 7"              "\n\t" // if pwmB < 0
        "rjmp L_pwmBneg%="              "\n\t" //    then goto L_pwmBneg
    "L_pwmBpos%=:"                      "\n\t"
        "andi %B[pwmB], 0x1F"           "\n\t" // pwmB & 0x1FFF
        "sts %B[POCR4A], __zero_reg__"  "\n\t" // OCR4AH = 0, high byte first (OC4A high, BIN2)
        "sts %A[POCR4A], __zero_reg__"  "\n\t" // OCR4AL = 0, then low byte (OC4A high, BIN2)
        "sts %B[POCR4B], %B[pwmB]"      "\n\t" // OCR4B = pwmB & 0x1FFF, high byte first
        "sts %A[POCR4B], %A[pwmB]"      "\n\t" // OCR4B = pwmB & 0x1FFF, then low byte
        "out __SREG__, __tmp_reg__"     "\n\t" // restore status register
        "ret"                           "\n\t" // return
    "L_pwmAneg%=:"                      "\n\t"
        "neg %B[pwmA]"                  "\n\t" // two's complement for high byte of pwmA
        "neg %A[pwmA]"                  "\n\t" // two's complement for low byte of pwmA
        "sbc %B[pwmA], __zero_reg__"    "\n\t" // subtract 1 from high byte, if carry flag is set
        "andi %B[pwmA], 0x1F"           "\n\t" // (-pwmA) & 0x1FFF
        "in __tmp_reg__, __SREG__"      "\n\t" // save status register in temporary register
        "cli"                           "\n\t" // disable interrupts
        "sts %B[POCR3B], __zero_reg__"  "\n\t" // OCR3BH = 0, high byte first (OC3B high, AIN2)
        "sts %A[POCR3B], __zero_reg__"  "\n\t" // OCR3BL = 0, then low byte (OC3B high, AIN2)
        "sts %B[POCR3A], %B[pwmA]"      "\n\t" // OCR3A = (-pwmA) & 0x1FFF, high byte first
        "sts %A[POCR3A], %A[pwmA]"      "\n\t" // OCR3A = (-pwmA) & 0x1FFF, then low byte
        "sbrs %B[pwmB], 7"              "\n\t" // if pwmB >= 0
        "rjmp L_pwmBpos%="              "\n\t" //    then goto L_pwmBpos
    "L_pwmBneg%=:"                      "\n\t"
        "neg %B[pwmB]"                  "\n\t" // two's complement for high byte of pwmB
        "neg %A[pwmB]"                  "\n\t" // two's complement for low byte of pwmB
        "sbc %B[pwmB], __zero_reg__"    "\n\t" // subtract 1 from high byte, if carry flag is set
        "andi %B[pwmB], 0x1F"           "\n\t" // (-pwmB) & 0x1FFF
        "sts %B[POCR4B], __zero_reg__"  "\n\t" // OCR4BH = 0, high byte first (OC4B high, BIN1)
        "sts %A[POCR4B], __zero_reg__"  "\n\t" // OCR4BL = 0, then low byte (OC4B high, BIN1)
        "sts %B[POCR4A], %B[pwmB]"      "\n\t" // OCR4A = (-pwmB) & 0x1FFF, high byte first
        "sts %A[POCR4A], %A[pwmB]"      "\n\t" // OCR4A = (-pwmB) & 0x1FFF, then low byte
        "out __SREG__, __tmp_reg__"     "\n\t" // restore status register
        "ret"                           "\n\t" // return
        : // no outputs
        : [POCR3A] "n" (& OCR3A),
          [POCR3B] "n" (& OCR3B),
          [POCR4A] "n" (& OCR4A),
          [POCR4B] "n" (& OCR4B),
          [pwmA] "d" (pwmA),
          [pwmB] "d" (pwmB)
        : // no clobber
    );

    // almost equivalent C code when compiling with -O2
    //    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    //        if (pwmA < 0) {
    //            OCR3B = 0; // OC3B high, AIN2
    //            OCR3A = (-pwmA) & 0x1FFF;
    //        } else {
    //            OCR3A = 0; // OC3A high, AIN1
    //            OCR3B = pwmA & 0x1FFF;
    //        }
    //
    //        if (pwmB < 0) {
    //            OCR4B = 0; // OC4B high, BIN1
    //            OCR4A = (-pwmB) & 0x1FFF;
    //        } else {
    //            OCR4A = 0; // OC4A high, BIN2
    //            OCR4B = pwmB & 0x1FFF;
    //        }
    //    }
}


void Motor_stopA(void) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        OCR3A = 0x1FFF;
        OCR3B = 0x1FFF;
    }
}


void Motor_stopB(void) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        OCR4A = 0x1FFF;
        OCR4B = 0x1FFF;
    }
}

void Motor_stopAll(void) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        OCR3A = 0x1FFF;
        OCR3B = 0x1FFF;
        OCR4A = 0x1FFF;
        OCR4B = 0x1FFF;
    }
}
