#include "communication.h"
#include <io/uart/uart.h>

#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

// Escape character
#define ESC 17

// Packet delimiter character
#define DELIM '+'


#ifndef COMM_UART
    #error COMM_UART is undefined
#endif

/**
 * Macro for concatenating two arguments a and b.
 * Used to construct function names including the UART number, see below
 */
#define CONCAT(a,b) a##b

/**
 * Macro for first expanding all macros in the arguments a and b and then
 * concatenating the result.
 * Used to construct function names including the UART number, see below
 */
#define EXPAND_AND_CONCAT(a,b) CONCAT(a,b)

/**
 * Macro for conveniently putting data into the TX buffer for the configured
 * UART (see COMM_UART).
 */
#define uart_write(b) EXPAND_AND_CONCAT(uart_write, COMM_UART)(b)

/**
 * Macro for conveniently checking if data is available for reading from the configured UART (see COMM_UART).
 */
#define uart_available() EXPAND_AND_CONCAT(uart_available, COMM_UART)()

/**
 * Macro for conveniently reading data from the configured UART (see COMM_UART).
 */
#define uart_read()   EXPAND_AND_CONCAT(uart_read, COMM_UART)()

// Macro for conveniently transmitting bytes
// if byte to transmit matches escape or delimiter character, it is automatically
// escaped
#define uart_writeEscaped(byte)           \
    if ((byte == ESC) || (byte == DELIM)) \
        uart_write(ESC);                  \
    uart_write(byte);


// Array with callback functions for each channel
static ChannelCallback_t communication_ChannelReceivers[COMM_MAX_CHANNELS];

// Buffer for incoming packets
static uint8_t inBuf[COMM_RECV_BUFFER_SIZE];

// Length of incoming packet buffer
static uint16_t inBufLen = 0;

// true if last incoming character was an escape character
static bool isESC = false;

// current checksum while reading incoming packet
static uint8_t inChksum = 0;

// Error flags, set by communication_readPackets()
static uint8_t errors = 0;


void communication_init(void) {
    // clear communication_ChannelReceivers
    memset(communication_ChannelReceivers, 0, sizeof(communication_ChannelReceivers));
}


void communication_setCallback(const Channel_t channel, const ChannelCallback_t callback) {
    communication_ChannelReceivers[channel] = callback;
}


void communication_clearCallback(const Channel_t channel) {
    communication_ChannelReceivers[channel] = 0;
}


void communication_log(const Level_t level, const char* format, ...) {
    va_list argp;
    va_start(argp, format);

    char* buff = (char*)malloc(258);
    if (buff == 0) {
        errors |= COMM_ERR_OUT_OF_MEMORY;
        return;
    }
    buff[0] = level;

    int size = vsnprintf(&buff[1], 257, format, argp);
    va_end(argp);

    // send packet to debug channel CH_OUT_DEBUG (0x00)
    communication_writePacket(CH_OUT_DEBUG, (uint8_t *)buff, size > 256 ? 257 : (size+1));
    free(buff);
}


void communication_log_P(const Level_t level, const char* format, ...) {
    va_list argp;
    va_start(argp, format);

    char* buff = (char*)malloc(258);
    if (buff == 0) {
        errors |= COMM_ERR_OUT_OF_MEMORY;
        return;
    }
    buff[0] = level;

    int size = vsnprintf_P(&buff[1], 257, format, argp);
    va_end(argp);

    // send packet to debug channel CH_OUT_DEBUG (0x00)
    communication_writePacket(CH_OUT_DEBUG, (uint8_t *)buff, size > 256 ? 257 : (size+1));
    free(buff);
}


void communication_writePacket(const Channel_t channel, const uint8_t* packet, const uint16_t size) {
    // while writing each byte, the global checksum is calculated over the whole
    // transmitted data including the header information

    // transmit low byte of payload size
    register uint8_t chksum = (uint8_t)size;
    uart_writeEscaped(chksum);

    // transmit high byte of payload size
    register uint8_t byte = (uint8_t)(size >> 8);
    uart_writeEscaped(byte);
    chksum ^= byte;

    // compute 4-bit checksum of size and place it in high nibble,
    // place channel number in low nibble and transmit
    byte = (((chksum << 4) & 0xFF) ^ (chksum & 0xF0)) | (channel & 0x0F);
    uart_writeEscaped(byte);
    chksum ^= byte;

    // transmit packet payload and update checksum
    for (uint16_t i = 0; i < size; i++) {
        register uint8_t tmp = packet[i];
        uart_writeEscaped(tmp);
        chksum ^= tmp;
    }

    // transmit checksum
    uart_writeEscaped(chksum);
    // transmit packet delimiter
    uart_write(DELIM);
}


static __attribute__ ((noinline)) void readPackets(void) {
    register uint8_t tmpChksum = inChksum;
    register uint16_t tmpBufLen = inBufLen;
    register uint8_t* tmpBuf = inBuf + tmpBufLen;
    register uint8_t tmpIsESC = isESC;

    // while some data is available in the UART RX buffer
    do {
        // get the data byte
        register uint8_t data = uart_read();

        if (tmpIsESC) { // if last data byte was the escape byte
            // no more escaping
            tmpIsESC = false;
        } else if (data == ESC) { // if data byte is the escape byte
            tmpIsESC = true; // set flag for indicating an escape sequence
            continue;
        } else if (data == DELIM) { // if data byte is delimiter, we should have a complete packet in the incoming buffer
            if (tmpBufLen >= 4) { // if number of bytes in buffer is at least the minimum size, we might have received a full packet

                register uint8_t chksumSize = inBuf[0]; // read low byte of payload size
                data = inBuf[1]; // read high byte of payload size
                uint16_t size = chksumSize | ((uint16_t)data << 8); // compute payload size

                // calculate 4-bit checksum for payload size in low nibble
                chksumSize ^= data;
                chksumSize = (chksumSize >> 4) ^ (chksumSize & 0x0F);

                data = inBuf[2]; // read checksum for payload size (bits 7-4) and channel ID (bits 3-0)

                if ((data >> 4) == chksumSize) { // check for mismatch of payload size checksum
                    if (size == tmpBufLen - 4) { // check packet length
                        if (tmpChksum == 0) { // if global checksum is ok
                            register uint8_t channel = data & 0x0F; // get channel number
                            // execute callback function
                            if (communication_ChannelReceivers[channel])
                                (communication_ChannelReceivers[channel])(inBuf+3, size);
                            else
                                errors |= COMM_ERR_UNREGISTEREDCHANNEL;
                        } else
                            errors |= COMM_ERR_CHECKSUM;
                    } else
                        errors |= COMM_ERR_SIZE_MISMATCH;
                } else
                    errors |= COMM_ERR_HEADER_CHECKSUM;
            } else
                errors |= COMM_ERR_TOO_SMALL;

            // clear incoming packet buffer and checksum
            tmpBufLen = 0;
            tmpChksum = 0;
            tmpBuf = inBuf;
            continue;
        }

        // place byte in incoming packet buffer
        *tmpBuf++ = data;
        tmpChksum ^= data;

        if (++tmpBufLen == COMM_RECV_BUFFER_SIZE) { // if incoming packet buffer is full
            // clear buffer
            tmpBufLen = 0;
            tmpChksum = 0;
            tmpBuf = inBuf;
            // set error flag
            errors |= COMM_ERR_BUFFERFULL;
        }
    } while (uart_available());

    // update current checksum and buffer position in memory before exiting
    inChksum = tmpChksum;
    inBufLen = tmpBufLen;
    isESC = tmpIsESC;
}


void communication_readPackets(void) {
    if (uart_available())
        readPackets();
}


uint8_t communication_getErrors(void) {
    // read errors
    uint8_t tmp = errors;
    // reset errors
    errors = 0;
    return tmp;
}
