/**
 * @file uart.h
 * @ingroup io
 *
 * UART library: controls UART 0,1,2,3 via interrupts and FIFO buffers.
 *
 * All functions are interrupt safe concerning UDRE and RX complete interrupt.
 */

#ifndef UART_H_
#define UART_H_

// include UART configuration
#include <cfg/io/uart/uart_cfg.h>

#include <stdint.h>
#include <stdbool.h>


/**
 * Initialize the UART library. Must be called before any other library function.
 * Global interrupts must be enabled manually after calling this function.
 *
 * See also the corresponding configuration file: src/cfg/io/uart/uart_cfg.h
 */
void uart_init(void);


/**
 * Read a single byte from the FIFO RX buffer of the corresponding UART. Blocks as
 * long as there are no bytes in the buffer. Check <code>uart_availableX()</code>
 * if you want to make sure <code>uart_readX()</code> won't block.
 *
 * Due to blocking, this function must not be called from interrupt context.
 *
 * @return  the retrieved byte
 */
uint8_t uart_read0(void);

/**
 * @copydoc uart_read0()
 */
uint8_t uart_read1(void);

/**
 * @copydoc uart_read0()
 */
uint8_t uart_read2(void);

/**
 * @copydoc uart_read0()
 */
uint8_t uart_read3(void);


/**
 * Write a single byte to the FIFO TX buffer of the corresponding UART. Blocks as
 * long as there is no space in the buffer. Check
 * <code>uart_TXBufSpaceAvailableX()</code> if you want to make sure
 * <code>uart_writeX()</code> won't block.
 *
 * Due to blocking, this function must not be called from interrupt context.
 *
 * Note: Blocking can be disabled by compiling with the symbol
 *       UART_NONBLOCKING_TRANSMIT defined. If the buffer is full in this case,
 *       the byte to be transmitted is discarded.
 *
 * @param   data   the byte to be placed into the TX buffer
 */
void uart_write0(const uint8_t data);

/**
 * @copydoc uart_write0()
 */
void uart_write1(const uint8_t data);

/**
 * @copydoc uart_write0()
 */
void uart_write2(const uint8_t data);

/**
 * @copydoc uart_write0()
 */
void uart_write3(const uint8_t data);


/**
 * Check if a single byte can be read from the receive FIFO buffer via
 * <code>uart_readX()</code> without blocking.
 *
 * @return  true if a single byte can be read without blocking, otherwise false
 */
bool uart_available0(void);

/**
 * @copydoc uart_available0()
 */
bool uart_available1(void);

/**
 * @copydoc uart_available0()
 */
bool uart_available2(void);

/**
 * @copydoc uart_available0()
 */
bool uart_available3(void);


/**
 * Wait until at least one byte is in RX buffer or until a timeout occurs.
 *
 * Due to blocking, this function must not be called from interrupt context.
 *
 * @param   timeout   timeout in milliseconds
 * @return  true  if at least one byte is available in RX buffer,
 *          false if timeout occurred
 */
bool uart_waitRXAvailable0(const uint8_t timeout);

/**
 * @copydoc uart_waitRXAvailable0()
 */
bool uart_waitRXAvailable1(const uint8_t timeout);

/**
 * @copydoc uart_waitRXAvailable0()
 */
bool uart_waitRXAvailable2(const uint8_t timeout);

/**
 * @copydoc uart_waitRXAvailable0()
 */
bool uart_waitRXAvailable3(const uint8_t timeout);


/**
 * Check if a single byte can be written to the FIFO TX buffer via
 * <code>uart_writeX()</code> without blocking.
 *
 * @return  true if a single byte can be written without blocking, otherwise false
 */
bool uart_TXBufSpaceAvailable0(void);

/**
 * @copydoc uart_TXBufSpaceAvailable0()
 */
bool uart_TXBufSpaceAvailable1(void);

/**
 * @copydoc uart_TXBufSpaceAvailable0()
 */
bool uart_TXBufSpaceAvailable2(void);

/**
 * @copydoc uart_TXBufSpaceAvailable0()
 */
bool uart_TXBufSpaceAvailable3(void);


/**
 * Determine the number of bytes which can be written to the FIFO TX buffer via
 * <code>uart_write()</code> without blocking.
 *
 * @return  number of bytes available in FIFO TX buffer
 */
uint8_t uart_getTXBufSpace0(void);

/**
 * @copydoc uart_getTXBufSpace0()
 */
uint8_t uart_getTXBufSpace1(void);

/**
 * @copydoc uart_getTXBufSpace0()
 */
uint8_t uart_getTXBufSpace2(void);

/**
 * @copydoc uart_getTXBufSpace0()
 */
uint8_t uart_getTXBufSpace3(void);


/**
 * Determine if an overflow of the UART RX buffer has occurred during the RX
 * complete ISR and clear the error flag.
 *
 * When this happens, additional data read from the UART is discarded and a
 * packet can not be fully received.
 *
 * @return  true if overflow occurred, false otherwise
 */
bool uart_isRXBufOverflow0(void);

/**
 * @copydoc uart_isRXBufOverflow0()
 */
bool uart_isRXBufOverflow1(void);

/**
 * @copydoc uart_isRXBufOverflow0()
 */
bool uart_isRXBufOverflow2(void);

/**
 * @copydoc uart_isRXBufOverflow0()
 */
bool uart_isRXBufOverflow3(void);


#endif /* UART_H_ */
