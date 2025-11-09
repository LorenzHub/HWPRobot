/* BASIC INTERRUPT DRIVEN I2C PORT DRIVER. See AVR315 for reference */
#include "i2c.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdbool.h>

#ifdef USE_I2C

/*
 *******************************************************************************
 * PRIVATE DEFINITIONS
 *******************************************************************************
 */

#define _I2C_WRITE(addr)    ((addr<<1) & 0xFE)
#define _I2C_READ(addr)     ((addr<<1) | 0x01)

// state machine commands
#define _I2C_SEND_START     TWCR |= _BV(TWINT) | _BV(TWSTA) | _BV(TWIE);
#define _I2C_CLEAR_START    TWCR = (TWCR | _BV(TWINT)) & ~_BV(TWSTA)    // clear SEND_START, clear INT
#define _I2C_SEND_STOP      TWCR = (TWCR | _BV(TWINT) | _BV(TWSTO)) & ~_BV(TWIE)
#define _I2C_CLEAR_INT      TWCR |= _BV(TWINT)
#define _I2C_SEND_ACK       TWCR |= _BV(TWEA) | _BV(TWINT)              // clear INT, send ACK
#define _I2C_SEND_NACK      TWCR = (TWCR | _BV(TWINT)) & ~_BV(TWEA)     // clear INT, send NACK

// @brief This struct defines a I2C message.
typedef struct I2C_message {
    uint8_t     deviceID;                       // 7 bit receiver node ID
    uint8_t     pos;                            // pos in message
    uint8_t     sendSize;                       // length of send message
    uint8_t     recvSize;                       // length of receive message
    uint8_t*    sendBuf;                        // send buffer
    uint8_t*    recvBuf;                        // receive buffer
} I2C_message_t;

typedef enum _I2C_pres {
    I2C_PRES_1,
    I2C_PRES_4,
    I2C_PRES_16,
    I2C_PRES_64
} _I2C_pres_t;

typedef enum _I2C_status {
    I2C_BUS_ERROR,
    I2C_START_SENT,
    I2C_REP_START_SENT,
    I2C_ADDR_W_SENT_ACK,
    I2C_ADDR_W_SENT_NACK,
    I2C_DATA_SENT_ACK,
    I2C_DATA_SENT_NACK,
    I2C_ARB_LOST,
    I2C_ADDR_R_SENT_ACK,
    I2C_ADDR_R_SENT_NACK,
    I2C_DATA_RECV_ACK,
    I2C_DATA_RECV_NACK
} _I2C_status_t;

volatile struct _I2C_ {
    bool            transferSemphr;
    I2C_message_t   activeMessage;
    I2C_result      transferSuccess;
} _I2C;


/*
 *******************************************************************************
 * PRIVATE FUNCTIONS
 *******************************************************************************
 */

ISR(TWI_vect) {
    _I2C_status_t status = TWSR >> 3;

    switch(status) {
      case I2C_START_SENT:
      case I2C_REP_START_SENT:
        _I2C.activeMessage.pos = 0;
        if(_I2C.activeMessage.sendSize == 0)
            TWDR = _I2C_READ(_I2C.activeMessage.deviceID);          // load address
        else
            TWDR = _I2C_WRITE(_I2C.activeMessage.deviceID);         // load address
        _I2C_CLEAR_START;
        return;


      case I2C_ADDR_W_SENT_ACK:
      case I2C_DATA_SENT_ACK:
        if(_I2C.activeMessage.pos < _I2C.activeMessage.sendSize) {
            TWDR = _I2C.activeMessage.sendBuf[_I2C.activeMessage.pos++];    // load data
            _I2C_CLEAR_INT;                 // clear INT
        } else {
            if(_I2C.activeMessage.recvSize > 0) {
                _I2C.activeMessage.sendSize = 0;    // mark send as finished
                _I2C_SEND_START;                    // repeated START
            } else {
                _I2C_SEND_STOP;
            }
            _I2C.transferSuccess = I2C_SUCCESS;
            _I2C.transferSemphr = false;
        }
        return;


      case I2C_DATA_RECV_ACK:
        _I2C.activeMessage.recvBuf[_I2C.activeMessage.pos++] = TWDR;    // read data
        return;
      case I2C_ADDR_R_SENT_ACK:
        if(_I2C.activeMessage.pos < _I2C.activeMessage.recvSize-1) {
            _I2C_SEND_ACK;
        } else {
            _I2C_SEND_NACK;
        }
        return;

      case I2C_DATA_RECV_NACK:
        _I2C.activeMessage.recvBuf[_I2C.activeMessage.pos] = TWDR;  // read data
        _I2C_SEND_STOP;
        _I2C.transferSuccess = I2C_SUCCESS;
        _I2C.transferSemphr = false;
        return;


    /******************* error states ********************/

      case I2C_ADDR_W_SENT_NACK:
      case I2C_ADDR_R_SENT_NACK:
      case I2C_DATA_SENT_NACK:
        _I2C.transferSuccess = I2C_NACK;
        _I2C.transferSemphr = false;
        _I2C_SEND_STOP;
        return;

      case I2C_BUS_ERROR:
        _I2C.transferSuccess = I2C_BUSERROR;
        _I2C.transferSemphr = false;
        _I2C_SEND_STOP;
        return;

      case I2C_ARB_LOST:
        _I2C.transferSuccess = I2C_ARBLOST;
        _I2C.transferSemphr = false;
        _I2C_SEND_STOP;
        return;

      default:
        _I2C.transferSuccess = I2C_UNKOWN_STATE;
        _I2C.transferSemphr = false;
        _I2C_SEND_STOP;
    }
}


/*
 *******************************************************************************
 * PUBLIC FUNCTIONS
 *******************************************************************************
 */

void I2C_init(void) {
    // disable power reduction of TWI
    PRR0 &= ~_BV(PRTWI);

    // BAUD = F_CPU / (16 + 2*TWBR*I2C_PRES)
    // 200kHz = 8MHz / 40 -> I2C_PRES_1 , TWBR=12
    TWSR = I2C_PRES_1;
    TWBR = 12;

    TWCR = _BV(TWEN) | _BV(TWIE);               // enable interface and INT

    DDRD &= ~(_BV(1) | _BV(0));                 // pins as input
    PORTD |= _BV(1) | _BV(0);                   // enable pullups

    _I2C.transferSemphr = false;
}


I2C_result I2C_write(const uint8_t deviceID, const uint8_t size, uint8_t* data, const bool block) {
    if(_I2C.transferSemphr) {
        if(block)
            while(_I2C.transferSemphr);
        else
            return I2C_BUSY;
    }
    _I2C.transferSemphr = true;

    _I2C.activeMessage.deviceID = deviceID;             // set deviceID
    _I2C.activeMessage.sendSize = size;                 // set transfer size (address + data)
    _I2C.activeMessage.sendBuf = data;
    _I2C.activeMessage.recvSize = 0;
    _I2C.activeMessage.recvBuf = NULL;

    TWCR |= _BV(TWSTA) | _BV(TWIE);                     // start transfer

    while(_I2C.transferSemphr);
    return _I2C.transferSuccess;
}


I2C_result I2C_read(const uint8_t deviceID, uint8_t address, const uint8_t size, uint8_t* data, const bool block) {
    if(_I2C.transferSemphr) {
        uint16_t timeout = 0x0fff;
        if(block)
            while(_I2C.transferSemphr & timeout--);
        else
            return I2C_BUSY;
    }

    _I2C.transferSemphr = true;

    // send address
    _I2C.activeMessage.deviceID = deviceID;             // set deviceID
    _I2C.activeMessage.sendSize = 1;                    // set transfer size (address + data)
    _I2C.activeMessage.sendBuf = &address;              // we want to transmit the address

    _I2C.activeMessage.recvSize = size;
    _I2C.activeMessage.recvBuf = data;

    TWCR |= _BV(TWSTA) | _BV(TWIE);                     // start transfer

    while(_I2C.transferSemphr)  __asm__ __volatile__ ("nop");
    _delay_us(100);
    return _I2C.transferSuccess;
}


#endif
