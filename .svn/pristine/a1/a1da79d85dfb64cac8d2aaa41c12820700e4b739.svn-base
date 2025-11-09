/**
 * @file i2c.h
 * @ingroup io
 *
 * Sending and receiving of I2C messages.
 *
 * Note that i2c.c must be compiled with the symbol USE_I2C defined in order to
 * have I2C functions from i2c.h and interrupt service routine for
 * <code>TWI_vect</code> implemented.
 */

#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>
#include <stdbool.h>


/**
 * Enumeration of I2C results of read and write operations
 */
typedef enum _I2C_result {
    I2C_SUCCESS,
    I2C_FAILURE,
    I2C_TIMEOUT,
    I2C_SIZE_OVERFLOW,
    I2C_BUSERROR,
    I2C_ARBLOST,
    I2C_NACK,
    I2C_BUSY,
    I2C_UNKOWN_STATE
} I2C_result;


/**
 * Initalize the I2C interface.
 * Global interrupts must be enabled manually after calling this function.
 */
void I2C_init(void);


/**
 * Send data, blocks task until message is sent.
 *
 * @param deviceID  the node ID
 * @param size      the size of the data
 * @param data      data to send
 * @param block     wait until I2C is ready
 * @return I2C_result
 */
I2C_result I2C_write(const uint8_t deviceID, const uint8_t size, uint8_t* data, const bool block);


/**
 * Receive data, blocks task until message is received.
 *
 * @param deviceID  the node ID
 * @param address   the register address to read from
 * @param size      the size of the data
 * @param data      data that was read
 * @param block     wait until I2C is ready
 * @return I2C_result
 */
I2C_result I2C_read(const uint8_t deviceID, uint8_t address, const uint8_t size, uint8_t* data, const bool block);


#endif /* I2C_H_ */
