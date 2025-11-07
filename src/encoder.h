#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/**
 * Initialize encoder hardware and interrupts
 */
void encoder_init();

/**
 * Get current encoder value
 * @param encoder_num Encoder number (1 or 2)
 * @return Current encoder count (int16_t)
 */
int16_t encoder_getValue(uint8_t encoder_num);

/**
 * Reset encoder counter to zero
 * @param encoder_num Encoder number (1 or 2)
 */
void encoder_reset(uint8_t encoder_num);

/**
 * Get raw pin values for debugging
 * @param encoder_num Encoder number (1 or 2)
 * @return Raw pin state (bits 0-1: pin A and B states)
 */
uint8_t encoder_getRawPins(uint8_t encoder_num);

#endif /* ENCODER_H */