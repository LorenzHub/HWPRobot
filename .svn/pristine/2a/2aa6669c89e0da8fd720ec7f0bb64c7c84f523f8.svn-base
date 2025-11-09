/**
 * @file fifo.h
 * @ingroup tools
 *
 * Simple implementation of a FIFO buffer using a ring buffer (circular buffer)
 */

#ifndef FIFO_H_
#define FIFO_H_

#include <stdint.h>


/**
 * FIFO buffer with maximum size of 255 bytes
 */
typedef struct {
    uint8_t _tail;    ///< index in _buffer locating the tail
    uint8_t _head;    ///< index in _buffer locating the head
    uint8_t* _buffer; ///< data buffer
} FIFO_t;


/**
 * FIFO buffer with a fixed size of 63 bytes
 */
typedef struct {
    uint8_t _tail;       ///< index in _buffer locating the tail
    uint8_t _head;       ///< index in _buffer locating the head
    uint8_t _buffer[64]; ///< data buffer
} FIFO64_t;


/**
 * FIFO buffer with a fixed size of 127 bytes
 */
typedef struct {
    uint8_t _tail;        ///< index in _buffer locating the tail
    uint8_t _head;        ///< index in _buffer locating the head
    uint8_t _buffer[128]; ///< data buffer
} FIFO128_t;


/**
 * Check if data is available in the FIFO to be read
 *
 * @param   fifo   FIFO data type
 */
#define FIFO_available(fifo)    (fifo._tail != fifo._head)


/**
 * Read a byte from FIFO.
 * Reading also increments the tail. If the buffer is empty, return the byte
 * at tail position.
 *
 * @param   fifo   FIFO data type
 * @param   size   size of the FIFO
 */
#define FIFO_read(fifo, size)   ( FIFO_available(fifo) ? (fifo._buffer[(fifo._tail = (fifo._tail+1)&(size-1))]) : (fifo._buffer[fifo._tail]) )


/**
 * Write a byte to the FIFO.
 * Writing also increments the head. If no bytes are available, overwrite the
 * byte at tail position.
 *
 * @param   fifo   FIFO data type
 * @param   data   data byte to be written
 * @param   size   size of the FIFO
 */
#define FIFO_write(fifo, data, size) {                                                          \
    uint8_t tmphead = ( fifo._head + 1 ) & (size-1);    /* calculate buffer index */            \
    if (tmphead != fifo._tail)                          /* if buffer is not full */   \
        fifo._head = tmphead;                           /* store new index */                   \
    fifo._buffer[tmphead] = data;                       /* store data in buffer */              \
}


#endif /* FIFO_H_ */
