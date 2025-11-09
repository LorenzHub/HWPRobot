#include "uart.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


// Macro for defining non-blocking or blocking (default) transmit,
// used in uart_writeX()
#ifndef UART_NONBLOCKING_TRANSMIT
    #define WAIT_FOR_FREE_SPACE while (tmpHead == uart->txTail);
#else
    #define WAIT_FOR_FREE_SPACE
#endif


/**
 * Head and tail for RX and TX buffers (cyclic buffers)
 * The error flag rxBufOverflow indicates if a buffer overflow has occurred in
 * the RX complete ISR. This flag can be read and cleared with
 * uart_isRXBufOverflowX().
 *
 * Note that the usable buffer size is UART*_*_BUFFER_SIZE - 1 !
 */
typedef struct __attribute__((__packed__)) {
    uint8_t rxHead;
    uint8_t rxTail;
    uint8_t txHead;
    uint8_t txTail;
    uint8_t rxBufOverflow;
} uart_t;


/** Macros for defining all functions from uart.h based on ID X of UART
        uart_writeX(const uint8_t data)
        uint8_t uart_readX(void)
        bool uart_availableX(void)
        bool uart_TXBufSpaceAvailableX(void)
        uint8_t uart_getTXBufSpaceX(void)
        bool uart_waitRXAvailableX(const uint8_t timeout)
        bool uart_isRXBufOverflowX(void)

        Note that ISRs are implemented in a separate .S file.
*/

#define uart_writeMacro(uartID) \
    void uart_write##uartID (const uint8_t data) { \
        volatile uart_t* uart = &uart##uartID; \
        /* locate next index for writing to TX buffer */ \
        uint8_t tmpHead = (uart->txHead + 1) & UART##uartID##_TX_MASK; \
        /* if TX buffer is full, busy wait for free space */ \
        WAIT_FOR_FREE_SPACE \
        /* store data in TX buffer */ \
        uart##uartID##_TX_buf[tmpHead] = data; \
        /* make data available to txISR by updating head */ \
        uart->txHead = tmpHead; \
        /* enable UDRE interrupt */ \
        UCSR##uartID##B = _BV(UDRIE##uartID) | _BV(RXCIE##uartID) | _BV(TXEN##uartID) | _BV(RXEN##uartID); \
    }

#define uart_readMacro(uartID) \
    uint8_t uart_read##uartID(void) { \
        volatile uart_t* uart = &uart##uartID; \
        /* wait for incoming bytes if RX buffer is empty (head == tail) */ \
        while (uart->rxHead == uart->rxTail) ; \
        /* calculate new tail after reading byte from RX buffer */ \
        uint8_t tmpTail = (uart->rxTail + 1) & UART##uartID##_RX_MASK; \
        /* read data from RX buffer */ \
        uint8_t data = uart##uartID##_RX_buf[tmpTail]; \
        /* remove byte from RX buffer by updating tail */ \
        uart->rxTail = tmpTail; \
        /* return data */ \
        return data; \
    }

#define uart_availableMacro(uartID) \
    bool uart_available##uartID(void) { \
        volatile uart_t* uart = &uart##uartID; \
        return (uart->rxHead != uart->rxTail); \
    }

#define uart_TXBufSpaceAvailableMacro(uartID) \
    bool uart_TXBufSpaceAvailable##uartID(void) { \
        volatile uart_t* uart = &uart##uartID; \
        /* locate next index for writing to TX buffer */ \
        uint8_t tmpHead = (uart->txHead + 1) & UART##uartID##_TX_MASK; \
        /* if buffer is full, return false */ \
        return (tmpHead != uart->txTail); \
    }

#define uart_getTXBufSpaceMacro(uartID) \
    uint8_t uart_getTXBufSpace##uartID(void) { \
        volatile uart_t* uart = &uart##uartID; \
        uint8_t tmpHead = uart->txHead; \
        uint8_t tmpTail = uart->txTail; \
        if (tmpTail > tmpHead) { \
            return tmpTail - tmpHead - 1; \
        } else { \
            return (UART##uartID##_TX_MASK - tmpHead) + tmpTail; \
        } \
    }

#define uart_waitRXAvailableMacro(uartID) \
    bool uart_waitRXAvailable##uartID(const uint8_t timeout) { \
        uint8_t _timeout = timeout; \
        for (;;) { \
            if (uart_available##uartID()) \
                return true; \
            if (_timeout-- == 0) \
                return false; \
            _delay_ms(1); \
        } \
        return false; \
    }

#define uart_isRXBufOverflowMacro(uartID) \
    bool uart_isRXBufOverflow##uartID(void) { \
        volatile uart_t* uart = &uart##uartID; \
        uint8_t _overflow = uart->rxBufOverflow; \
        uart->rxBufOverflow = 0; \
        return _overflow != 0; \
    }

#define uart_initMacro(uartID) \
    /* calculate value of baud rate register to closely match BAUD_RATEX */ \
    UBRR##uartID = (uint16_t)(((uint32_t)F_CPU + (uint32_t)BAUD_RATE##uartID * 8UL) / ((uint32_t)BAUD_RATE##uartID * 16UL) - 1UL); \
    \
    /* clear TX complete flag */ \
    UCSR##uartID##A = _BV(TXC##uartID); \
    /* enable receiver and transmitter, enable RX complete interrupt (data register empty interrupt is activated when bytes need to be sent) */ \
    UCSR##uartID##B = _BV(RXCIE##uartID) | _BV(TXEN##uartID) | _BV(RXEN##uartID); \
    /* set 8-bit character size, 1 stop bit, no parity */ \
    UCSR##uartID##C = _BV(UCSZ##uartID##1) | _BV(UCSZ##uartID##0);


#ifdef USE_UART0
    // check buffer sizes for correctness
    #if (UART0_TX_BUFFER_SIZE > 256)
        #error TX0 buffer size exceeds limit of 256 bytes
    #endif
    #if (UART0_TX_BUFFER_SIZE & (UART0_TX_BUFFER_SIZE - 1))
        #error TX0 buffer size is not a power of 2
    #endif

    #if (UART0_RX_BUFFER_SIZE > 256)
        #error RX0 buffer size exceeds limit of 256 bytes
    #endif
    #if (UART0_RX_BUFFER_SIZE & (UART0_RX_BUFFER_SIZE - 1))
        #error RX0 buffer size is not a power of 2
    #endif

    // bit masks for RX and TX buffer management (used for head and tail updates)
    #define UART0_RX_MASK (UART0_RX_BUFFER_SIZE - 1)
    #define UART0_TX_MASK (UART0_TX_BUFFER_SIZE - 1)

    // initialize data structure for head and tail of RX and TX buffers
    volatile uart_t uart0 = { .rxHead = 0, .rxTail= 0, .txHead = 0, .txTail = 0, .rxBufOverflow = 0 };

    volatile uint8_t uart0_RX_buf[UART0_RX_BUFFER_SIZE]; // RX buffer
    volatile uint8_t uart0_TX_buf[UART0_TX_BUFFER_SIZE]; // TX buffer

    // Implement all functions for UART 0
    uart_writeMacro(0)
    uart_readMacro(0)
    uart_availableMacro(0)
    uart_TXBufSpaceAvailableMacro(0)
    uart_getTXBufSpaceMacro(0)
    uart_waitRXAvailableMacro(0)
    uart_isRXBufOverflowMacro(0)
#endif


#ifdef USE_UART1
    // check buffer sizes for correctness
    #if (UART1_TX_BUFFER_SIZE > 256)
        #error TX1 buffer size exceeds limit of 256 bytes
    #endif
    #if (UART1_TX_BUFFER_SIZE & (UART1_TX_BUFFER_SIZE - 1))
        #error TX1 buffer size is not a power of 2
    #endif

    #if (UART1_RX_BUFFER_SIZE > 256)
        #error RX1 buffer size exceeds limit of 256 bytes
    #endif
    #if (UART1_RX_BUFFER_SIZE & (UART1_RX_BUFFER_SIZE - 1))
        #error RX1 buffer size is not a power of 2
    #endif

    // bit masks for RX and TX buffer management (used for head and tail updates)
    #define UART1_RX_MASK (UART1_RX_BUFFER_SIZE - 1)
    #define UART1_TX_MASK (UART1_TX_BUFFER_SIZE - 1)

    // initialize data structure for head and tail of RX and TX buffers
    volatile uart_t uart1 = { .rxHead = 0, .rxTail= 0, .txHead = 0, .txTail = 0, .rxBufOverflow = 0 };

    volatile uint8_t uart1_RX_buf[UART1_RX_BUFFER_SIZE]; // RX buffer
    volatile uint8_t uart1_TX_buf[UART1_TX_BUFFER_SIZE]; // TX buffer

    // Implement all functions for UART 1
    uart_writeMacro(1)
    uart_readMacro(1)
    uart_availableMacro(1)
    uart_TXBufSpaceAvailableMacro(1)
    uart_getTXBufSpaceMacro(1)
    uart_waitRXAvailableMacro(1)
    uart_isRXBufOverflowMacro(1)
#endif


#ifdef USE_UART2
    // check buffer sizes for correctness
    #if (UART2_TX_BUFFER_SIZE > 256)
        #error TX2 buffer size exceeds limit of 256 bytes
    #endif
    #if (UART2_TX_BUFFER_SIZE & (UART2_TX_BUFFER_SIZE - 1))
        #error TX2 buffer size is not a power of 2
    #endif

    #if (UART2_RX_BUFFER_SIZE > 256)
        #error RX2 buffer size exceeds limit of 256 bytes
    #endif
    #if (UART2_RX_BUFFER_SIZE & (UART2_RX_BUFFER_SIZE - 1))
        #error RX2 buffer size is not a power of 2
    #endif

    // bit masks for RX and TX buffer management (used for head and tail updates)
    #define UART2_RX_MASK (UART2_RX_BUFFER_SIZE - 1)
    #define UART2_TX_MASK (UART2_TX_BUFFER_SIZE - 1)

    // initialize data structure for head and tail of RX and TX buffers
    volatile uart_t uart2 = { .rxHead = 0, .rxTail= 0, .txHead = 0, .txTail = 0, .rxBufOverflow = 0 };

    volatile uint8_t uart2_RX_buf[UART2_RX_BUFFER_SIZE]; // RX buffer
    volatile uint8_t uart2_TX_buf[UART2_TX_BUFFER_SIZE]; // TX buffer

    // Implement all functions for UART 2
    uart_writeMacro(2)
    uart_readMacro(2)
    uart_availableMacro(2)
    uart_TXBufSpaceAvailableMacro(2)
    uart_getTXBufSpaceMacro(2)
    uart_waitRXAvailableMacro(2)
    uart_isRXBufOverflowMacro(2)
#endif


#ifdef USE_UART3
    // check buffer sizes for correctness
    #if (UART3_TX_BUFFER_SIZE > 256)
        #error TX3 buffer size exceeds limit of 256 bytes
    #endif
    #if (UART3_TX_BUFFER_SIZE & (UART3_TX_BUFFER_SIZE - 1))
        #error TX3 buffer size is not a power of 2
    #endif

    #if (UART3_RX_BUFFER_SIZE > 256)
        #error RX3 buffer size exceeds limit of 256 bytes
    #endif
    #if (UART3_RX_BUFFER_SIZE & (UART3_RX_BUFFER_SIZE - 1))
        #error RX3 buffer size is not a power of 2
    #endif

    // bit masks for RX and TX buffer management (used for head and tail updates)
    #define UART3_RX_MASK (UART3_RX_BUFFER_SIZE - 1)
    #define UART3_TX_MASK (UART3_TX_BUFFER_SIZE - 1)

    // initialize data structure for head and tail of RX and TX buffers
    volatile uart_t uart3 = { .rxHead = 0, .rxTail= 0, .txHead = 0, .txTail = 0, .rxBufOverflow = 0 };

    volatile uint8_t uart3_RX_buf[UART3_RX_BUFFER_SIZE]; // RX buffer
    volatile uint8_t uart3_TX_buf[UART3_TX_BUFFER_SIZE]; // TX buffer

    // Implement all functions for UART 3
    uart_writeMacro(3)
    uart_readMacro(3)
    uart_availableMacro(3)
    uart_TXBufSpaceAvailableMacro(3)
    uart_getTXBufSpaceMacro(3)
    uart_waitRXAvailableMacro(3)
    uart_isRXBufOverflowMacro(3)
#endif


void uart_init() {

#ifdef USE_UART0
    // disable power reduction of USART0
    PRR0 &= ~_BV(PRUSART0);
    uart_initMacro(0)
#endif

#ifdef USE_UART1
    // disable power reduction of USART1
    PRR1 &= ~_BV(PRUSART1);
    uart_initMacro(1)
#endif

#ifdef USE_UART2
    // disable power reduction of USART2
    PRR1 &= ~_BV(PRUSART2);
    uart_initMacro(2)
#endif

#ifdef USE_UART3
    // disable power reduction of USART3
    PRR1 &= ~_BV(PRUSART3);
    uart_initMacro(3)
#endif
}
