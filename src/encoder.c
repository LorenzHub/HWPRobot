#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "encoder.h"

// Encoder counters (volatile because accessed from ISR)
static volatile int16_t encoder1_count = 0;
static volatile int16_t encoder2_count = 0;

// Previous state for quadrature decoding
static volatile uint8_t encoder1_last_state = 0;
static volatile uint8_t encoder2_last_state = 0;

// Quadrature encoder state table
// Index: (old_A << 1) | old_B, bits 2-3: (new_A << 1) | new_B
// Value: -1 = reverse, +1 = forward, 0 = invalid/no change
static const int8_t quadrature_table[16] = {
    0,  // 00 -> 00: no change
    +1, // 00 -> 01: forward
    -1, // 00 -> 10: reverse
    0,  // 00 -> 11: invalid
    -1, // 01 -> 00: reverse
    0,  // 01 -> 01: no change
    0,  // 01 -> 10: invalid
    +1, // 01 -> 11: forward
    +1, // 10 -> 00: forward
    0,  // 10 -> 01: invalid
    0,  // 10 -> 10: no change
    -1, // 10 -> 11: reverse
    0,  // 11 -> 00: invalid
    -1, // 11 -> 01: reverse
    +1, // 11 -> 10: forward
    0   // 11 -> 11: no change
};

// Pin Change Interrupt for Port D (Encoder 1: PD0, PD1)
ISR(PCINT2_vect) {
    // Read current pin states (PD0 = bit 0, PD1 = bit 1)
    uint8_t pin_state = PIND & ((1<<PD0) | (1<<PD1));
    uint8_t current_state = (pin_state >> PD0) & 0x03; // Extract bits 0 and 1
    uint8_t state_change = (encoder1_last_state << 2) | current_state;
    int8_t direction = quadrature_table[state_change];
    
    if (direction != 0) {
        encoder1_count += direction;
    }
    
    encoder1_last_state = current_state;
}

// Pin Change Interrupt for Port B (Encoder 2: PB7, PB6)
ISR(PCINT0_vect) {
    // Read current pin states (PB6 = bit 6, PB7 = bit 7)
    uint8_t pin_state = PINB & ((1<<PB7) | (1<<PB6));
    uint8_t current_state = (pin_state >> PB6) & 0x03; // Extract bits 6 and 7, shift to 0-1
    uint8_t state_change = (encoder2_last_state << 2) | current_state;
    int8_t direction = quadrature_table[state_change];
    
    if (direction != 0) {
        encoder2_count += direction;
    }
    
    encoder2_last_state = current_state;
}

void encoder_init() {
    // Configure PD0 and PD1 as inputs with pull-up resistors for encoder1
    DDRD &= ~((1<<PD0) | (1<<PD1));
    PORTD |= (1<<PD0) | (1<<PD1);
    
    // Configure PB7 and PB6 as inputs with pull-up resistors for encoder2
    DDRB &= ~((1<<PB7) | (1<<PB6));
    PORTB |= (1<<PB7) | (1<<PB6);
    
    // Read initial state
    encoder1_last_state = ((PIND & ((1<<PD0) | (1<<PD1))) >> PD0) & 0x03;
    encoder2_last_state = ((PINB & ((1<<PB7) | (1<<PB6))) >> PB6) & 0x03;
    
    // Enable Pin Change Interrupts
    // PCMSK2: Port D (PD0-PD7) - Encoder 1 uses PD0, PD1
    PCMSK2 = (1<<PCINT16) | (1<<PCINT17); // PCINT16 = PD0, PCINT17 = PD1
    // PCMSK0: Port B (PB0-PB7) - Encoder 2 uses PB7, PB6
    PCMSK0 = (1<<PCINT6) | (1<<PCINT7); // PCINT6 = PB6, PCINT7 = PB7
    
    // Enable Pin Change Interrupts in PCICR
    PCICR = (1<<PCIE2) | (1<<PCIE0); // PCIE2 for Port D, PCIE0 for Port B
    
    // Reset counters
    encoder1_count = 0;
    encoder2_count = 0;
}

int16_t encoder_getValue(uint8_t encoder_num) {
    int16_t value;
    
    if (encoder_num == 1) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            value = encoder1_count;
        }
    } else if (encoder_num == 2) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            value = encoder2_count;
        }
    } else {
        value = 0;
    }
    
    return value;
}

void encoder_reset(uint8_t encoder_num) {
    if (encoder_num == 1) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            encoder1_count = 0;
        }
    } else if (encoder_num == 2) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            encoder2_count = 0;
        }
    }
}

uint8_t encoder_getRawPins(uint8_t encoder_num) {
    if (encoder_num == 1) {
        return ((PIND & ((1<<PD0) | (1<<PD1))) >> PD0) & 0x03;
    } else if (encoder_num == 2) {
        return ((PINB & ((1<<PB7) | (1<<PB6))) >> PB6) & 0x03;
    }
    return 0;
}


