/**
 * @file communication.h
 * @ingroup communication
 *
 * Functions enabling communication between robot and HWPCS.
 *
 * Receive buffer size and UART to be used are defined by the user configuration
 * in src/cfg/io/uart/uart_cfg.h.
 */

#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include "packetTypes.h"

#include <stdint.h>


/**
 * Log levels for distinguishing different categories of log messages.
 * Log levels correspond to Java log levels and are shown in the debug view of
 * HWPCS.
 * To be used in
 * ::communication_log(const Level_t level, const char* format, ...)
 */
typedef enum {
    LEVEL_SEVERE = 0, ///< message level indicating a serious failure
    LEVEL_WARNING = 1, ///< message level indicating a potential problem
    LEVEL_INFO = 2, ///< message level for informational messages
    LEVEL_FINE = 3, ///< message level providing tracing information
    LEVEL_FINER = 4, ///< message level indicating a fairly detailed tracing message
    LEVEL_FINEST = 5 ///< message level indicating a highly detailed tracing message
} Level_t;


/*
 * Communication errors resulting from communication_readPackets(),
 * communication_log() or communication_log_P ().
 * The errors can be read and cleared by communication_getErrors().
 * Error values can be or'ed.
 */

/** Receive buffer is full */
#define COMM_ERR_BUFFERFULL 1

/** Packet is too small */
#define COMM_ERR_TOO_SMALL 2

/** Header checksum mismatch */
#define COMM_ERR_HEADER_CHECKSUM 4

/** Packet size mismatch */
#define COMM_ERR_SIZE_MISMATCH 8

/** Global checksum mismatch */
#define COMM_ERR_CHECKSUM 16

/**
 * Packet received on a channel for which no callback function has been
 * registered
*/
#define COMM_ERR_UNREGISTEREDCHANNEL 32

/**
 * No memory for buffer in communication_log() or communication_log_P ()
 * could be allocated on heap
*/
#define COMM_ERR_OUT_OF_MEMORY 64

/**
 * Definition of the number of communication channels
 */
#define COMM_MAX_CHANNELS 16


/**
 * Type definition of a function pointer defining the channel callback functions for
 * incoming packages.
 *
 * The callback function is executed by communication_readPackets() when a packet
 * was received for the corresponding channel.
 *
 * Registering the callback function <code>cbf()</code> for channel i can be done by
 * {@link communication_setCallback}(i, cbf).
 *
 * <b>Example:</b>
 * @code
 * void cbf(const uint8_t* packet, const uint16_t size) {
 *     // parse packet and do some stuff
 * }
 * ...
 * communication_setCallback(i, cbf);
 * @endcode
 *
 * The signature of the callback function is:
 * @param   packet  pointer to the received packet (uint8_t*)
 * @param   size    size of the received packet (uint16_t)
 */
typedef void (*ChannelCallback_t)(const uint8_t*, const uint16_t);


/**
 * Initializes the communication protocol library. Must be invoked before any
 * other communication library function.
 * This function assumes that the UART to be used for communication was initialized
 * before (call uart_init() before this function).
 */
void communication_init(void);


/**
 * Register a callback function with prototype as defined by ChannelCallback_t
 * for a communication channel.
 * The callback function is executed by communication_readPackets() when a packet
 * was received for the corresponding channel.
 *
 * @param   channel   communication channel
 * @param   callback  function pointer to the callback function
 */
void communication_setCallback(const Channel_t channel, const ChannelCallback_t callback);


/**
 * Unregister the callback function for the given communication channel.
 *
 * @param   channel  communication channel
 */
void communication_clearCallback(const Channel_t channel);


/**
 * Send a packet to HWPCS on a specified channel. Blocks until all bytes have
 * been put into the UART transmit buffer.
 *
 * Due to blocking, this function must not be called from interrupt context.
 * See documentation of function uart_write0() on how to disable blocking.
 *
 * <b>Example:</b>
 * @code
 * Telemetry_t telemetry;
 * telemetry.infrared1 = ADC_getFilteredValue(0);
 * communication_writePacket(CH_OUT_TELEMETRY, (uint8_t*)&telemetry, sizeof(telemetry));
 * @endcode
 *
 * @param   channel   the channel to use, given as Channel_t (see packetTypes.h)
 * @param   packet    pointer to the packet to send (see packetTypes.h)
 * @param   size      size of the packet to send
 */
void communication_writePacket(const Channel_t channel, const uint8_t* packet, const uint16_t size);


/**
 * Read all available packets from the corresponding UART FIFO buffer.
 * Should be called periodically from the main loop context to poll received packages.
 * The function will internally invoke registered channel callback functions
 * which can be registered with communication_setCallback().
 *
 * The following error flags are set by this function, which can be fetched and
 * cleared with communication_getErrors(): #COMM_ERR_BUFFERFULL,
 * #COMM_ERR_TOO_SMALL, #COMM_ERR_HEADER_CHECKSUM, #COMM_ERR_SIZE_MISMATCH,
 * #COMM_ERR_CHECKSUM, #COMM_ERR_UNREGISTEREDCHANNEL
 */
void communication_readPackets(void);


/**
 * Send a log packet to HWPCS on channel #CH_OUT_DEBUG (0x00) which is shown in the
 * Debug View.
 * The resulting message string will be truncated after 256 characters excluding
 * null termination.
 * The function blocks until all bytes have been put into the UART transmit
 * buffer.
 *
 * Due to blocking, this function must not be called from interrupt context.
 * See documentation of function uart_write0() on how to disable blocking.
 *
 * @param   level   one of #Level_t
 * @param   format  a string containing the message format
 *                  (see <a href="https://www.nongnu.org/avr-libc/user-manual/group__avr__stdio.html" target="_blank">vfprintf</a>
 *                  from AVR Libc for details)
 */
void communication_log(const Level_t level, const char* format, ...);


/**
 * Same as communication_log(), however the format argument resides in flash
 * (program) memory.
 *
 * Send a log packet to HWPCS on channel #CH_OUT_DEBUG (0x00) which is shown in the
 * Debug View.
 * The resulting message string will be truncated after 256 characters excluding
 * null termination.
 * The function blocks until all bytes have been put into the UART transmit
 * buffer.
 *
 * Due to blocking, this function must not be called from interrupt context.
 * See documentation of function uart_write0() on how to disable blocking.
 *
 * This function should be used in conjunction with type modifiers from
 * <a href="https://www.nongnu.org/avr-libc/user-manual/group__avr__pgmspace.html" target="_blank"><avr/pgmspace.h></a>.
 * The first argument format must reside in flash (program) memory.
 * For statically allocated strings this can be done with
 *
 * \code
 * uint8_t i = 3;
 * char str[] = "some text";
 * communication_log_P(LEVEL_INFO, PSTR("Iteration i=%i Text=%s"), i, str);
 * \endcode
 *
 * For other possibilities and further details see
 * <a href="https://www.nongnu.org/avr-libc/user-manual/group__avr__pgmspace.html" target="_blank"><avr/pgmspace.h></a>.
 *
 * @param   level   one of #Level_t
 * @param   format  a string containing the message format
 *                  (see <a href="https://www.nongnu.org/avr-libc/user-manual/group__avr__stdio.html" target="_blank">vfprintf</a>
 *                  from AVR Libc for details)
 */
void communication_log_P(const Level_t level, const char* format, ...);


/**
 * Read and clear error state.
 * The error state is set by communication_readPackets() to indicate several
 * problems which may have occurred while reading the packets.
 * communication_log() and communication_log_P() can also set the error
 * #COMM_ERR_OUT_OF_MEMORY in case an internal buffer for creating the
 * log message cannot be allocated.
 *
 * @return  uint8_t containing the error bits as given by the following
 *                  definitions: #COMM_ERR_BUFFERFULL, #COMM_ERR_TOO_SMALL,
 *                  #COMM_ERR_HEADER_CHECKSUM, #COMM_ERR_SIZE_MISMATCH,
 *                  #COMM_ERR_CHECKSUM, #COMM_ERR_UNREGISTEREDCHANNEL,
 *                  #COMM_ERR_OUT_OF_MEMORY
 */
uint8_t communication_getErrors(void);


#endif /* COMMUNICATION_H_ */
