/**
 * @file uart_cfg.h
 * @ingroup cfg
 *
 * User configuration for UART and communication library.
 */

#ifndef UART_CFG_H_
#define UART_CFG_H_

/**
 * Constant identifying the UART number for USB connection
 */
#define USB         0

/**
 * Constant identifying the UART number for WIFI connection
 */
#define WIFI        1


/**
 * Definition of the UART to be used by the communication functions (lib/communication/communication.h).
 * Possible values: #USB, #WIFI
 */
//#define COMM_UART USB
#define COMM_UART WIFI

// enable additional UARTs if required
//#define ENABLE_USB
//#define ENABLE_WIFI


/**
 * Size of the packet buffer for reading incoming packets by
 * communication_readPackets().
 *
 * The buffer must be large enough to contain a full packet excluding escape
 * characters and packet delimiter character. That is, two bytes for packet
 * length, one byte for channel and header checksum, and another byte for the
 * checksum as well as the size of the payload.
 * In total: size of payload + 4 bytes.

 * Maximum size is theoretically 65535, however it is also limited by the amount
 * of available SRAM (= 8KB - static memory - dynamic memory - stack!)
 */
#define COMM_RECV_BUFFER_SIZE 1024


/// @cond

// enable UARTs depending on previous choice
#if( (COMM_UART == USB) || defined(ENABLE_USB) )
    #define USE_UART0
    #define UART0_TX_BUFFER_SIZE 256        // Power of 2!
    #define UART0_RX_BUFFER_SIZE 128        // Power of 2!
    #define BAUD_RATE0 500000
#endif

#if( (COMM_UART == WIFI) || defined(ENABLE_WIFI) )
    #define USE_UART1
    #define UART1_TX_BUFFER_SIZE 256        // Power of 2!
    #define UART1_RX_BUFFER_SIZE 128        // Power of 2!
    #define BAUD_RATE1 500000
#endif

/// @endcond

#endif /* UART_CFG_H_ */
