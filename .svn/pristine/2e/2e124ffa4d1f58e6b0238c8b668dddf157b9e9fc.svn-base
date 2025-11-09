// include UART configuration
#include <cfg/io/uart/uart_cfg.h>

#include <avr/io.h>         // AVR IO ports


//#define uart_rxISRMacro(uartID)
//    ISR(USART##uartID##_RX_vect) {
//        /* read the received data */
//        uint8_t data = UDR##uartID;
//        volatile uart_t* uart = &uart##uartID;
//        /* locate next index for writing to RX buffer */
//        uint8_t tmpHead = (uart->rxHead + 1 ) & UART##uartID##_RX_MASK;
//        if (tmpHead == uart->rxTail) {
//            /* ERROR! Receive buffer overflow */
//            uart->rxBufOverflow = 1;
//        } else {
//            /* store data in RX buffer */
//            uart##uartID##_RX_buf[tmpHead] = data;
//            /* make data available to receiveByte by updating head */
//            uart->rxHead = tmpHead; /* store new index */
//        }
//    }

#define uart_rxISR(uartID) \
    .extern uart##uartID##_RX_buf $ \
    .extern uart##uartID $ \
    \
    .global USART##uartID##_RX_vect $ \
    USART##uartID##_RX_vect: $ \
        push r2 $ \
        in r2, _SFR_IO_ADDR(SREG)  $ \
        push r18 $ \
        push ZL $ \
        push ZH $ \
        \
        /* read the received data */ \
        /* uint8_t data = UDR##uartID; */ \
        lds r18, _SFR_MEM_ADDR(UDR##uartID) $ \
        \
        /* locate next index for writing to RX buffer */ \
        /* uint8_t tmpHead = (uart->rxHead + 1 ) & UART##uartID##_RX_MASK; */ \
        lds ZL, uart##uartID $ \
        inc ZL  $ \
        andi ZL, UART##uartID##_RX_BUFFER_SIZE - 1 $ \
        \
        /* if (tmpHead != uart->rxTail) { */ \
        lds ZH, uart##uartID + 1 $ \
        cp  ZH, ZL $ \
        breq RX_BUF_OVERFLOW##uartID  $ \
        \
        /* make data available to receiveByte by updating head */ \
        /* uart->rxHead = tmpHead;  store new index */ \
        sts uart##uartID, ZL $ \
        \
        /* store data in RX buffer */ \
        /* uart##uartID##_RX_buf[tmpHead] = data; */ \
        ldi ZH, 0x00 $ \
        subi ZL, lo8(-(uart##uartID##_RX_buf)) $ \
        sbci ZH, hi8(-(uart##uartID##_RX_buf)) $ \
        st  Z, r18 $ \
        \
    RX_ISR_END##uartID: $ \
        pop ZH $ \
        pop ZL $ \
        pop r18 $ \
        out _SFR_IO_ADDR(SREG), r2 $ \
        pop r2 $ \
        reti $ \
    RX_BUF_OVERFLOW##uartID: $ \
        ldi r18, 0x01 $ \
        sts uart##uartID + 4, r18 $ \
        rjmp RX_ISR_END##uartID


//#define txISRMacro(uartID)
//    ISR(USART##uartID##_UDRE_vect) {
//        volatile uart_t* uart = &uart##uartID;
//        /* check if data needs to be transmitted */
//        if (uart->txHead != uart->txTail) {
//            /* calculate new tail after reading byte from TX buffer */
//            uint8_t tmpTail = (uart->txTail + 1 ) & UART##uartID##_TX_MASK;
//            /* read data from TX buffer */
//            uint8_t data = uart##uartID##_TX_buf[tmpTail];
//            /* remove byte from TX buffer by updating tail */
//            uart->txTail = tmpTail;
//             /* start transmission */
//            UDR##uartID = data;
//        } else {
//            /* disable UDRE interrupt */
//            UCSR##uartID##B &= ~_BV(UDRIE##uartID);
//        }
//    }

#define uart_txISR(uartID) \
    .extern uart##uartID##_TX_buf $ \
    .extern uart##uartID $ \
    \
    .global USART##uartID##_UDRE_vect $ \
    USART##uartID##_UDRE_vect: $ \
        push r2 $ \
        in r2, _SFR_IO_ADDR(SREG)  $ \
        push ZL $ \
        push ZH $ \
        \
        /* check if data needs to be transmitted */ \
        /* if (uart->txHead != uart->txTail) { */ \
        lds ZH, uart##uartID + 2 $ \
        lds ZL, uart##uartID + 3 $ \
        cp  ZH, ZL $ \
        breq TX_ISR_DISABLE_UDRE##uartID $ \
        \
        /* calculate new tail after reading byte from TX buffer */ \
        inc ZL $ \
        andi ZL, UART##uartID##_TX_BUFFER_SIZE - 1 $ \
        \
        /* remove byte from TX buffer by updating tail */ \
        sts uart##uartID + 3, ZL $ \
        \
        /* read data from TX buffer */ \
        ldi ZH, 0x00 $ \
        subi ZL, lo8(-(uart##uartID##_TX_buf)) $ \
        sbci ZH, hi8(-(uart##uartID##_TX_buf)) $ \
        ld ZL, Z $ \
        \
        /* start transmission */ \
        sts _SFR_MEM_ADDR(UDR##uartID), ZL $ \
        \
    TX_ISR_END##uartID: $ \
        pop ZH $ \
        pop ZL $ \
        out _SFR_IO_ADDR(SREG), r2 $ \
        pop r2 $ \
        reti $ \
        \
    TX_ISR_DISABLE_UDRE##uartID: $ \
        /* disable UDRE interrupt */ \
        ldi ZL, _BV(RXCIE##uartID) | _BV(TXEN##uartID) | _BV(RXEN##uartID) $ \
        sts _SFR_MEM_ADDR(UCSR##uartID##B), ZL $ \
        rjmp TX_ISR_END##uartID


#ifdef USE_UART0
    uart_rxISR(0)
    uart_txISR(0)
#endif

#ifdef USE_UART1
    uart_rxISR(1)
    uart_txISR(1)
#endif

#ifdef USE_UART2
    uart_rxISR(2)
    uart_txISR(2)
#endif

#ifdef USE_UART3
    uart_rxISR(3)
    uart_txISR(3)
#endif
