/**
 * @file variablesAccess.h
 * @ingroup tools
 *
 * Provision of bitset data types and access to memory locations as different data types.
 */

#ifndef VARIABLESACCESS_H_
#define VARIABLESACCESS_H_

#include <stdint.h>


/**
 * Get the float value starting at the memory address of v
 */
#define Float(v) (*((float*)(&(v))))


/**
 * Get the int8_t value at the memory address of v
 */
#define SInt8(v) (*((uint8_t*)(&(v))))


/**
 * Get the uint8_t value at the memory address of v
 */
#define UInt8(v) (*((uint8_t*)(&(v))))


/**
 * Get the int16_t value starting at the memory address of v
 */
#define SInt16(v) (*((int16_t*)(&(v))))


/**
 * Get the uint16_t value starting at the memory address of v
 */
#define UInt16(v) (*((uint16_t*)((&v))))


/**
 * Convert the address of v to a pointer of target type uint8_t
 */
#define UInt8Buf(v) ((uint8_t*)(&v))


/**
 * Definition of a variable-size array including size parameter
 */
typedef struct {
    uint8_t* data; ///< pointer to array data
    uint8_t size;  ///< size of the array
} array8_t;


/**
 * Definition of a set of 8 bits with individual access.
 */
typedef struct {
    uint8_t bit0 : 1; ///< access to bit 0
    uint8_t bit1 : 1; ///< access to bit 1
    uint8_t bit2 : 1; ///< access to bit 2
    uint8_t bit3 : 1; ///< access to bit 3
    uint8_t bit4 : 1; ///< access to bit 4
    uint8_t bit5 : 1; ///< access to bit 5
    uint8_t bit6 : 1; ///< access to bit 6
    uint8_t bit7 : 1; ///< access to bit 7
} bitset8;


/**
 * Definition of a set of 8 bits with individual and uint8_t access.
 */
typedef union {
    uint8_t value;  ///< access to bits as uint8_t
    bitset8 bitset; ///< access to bits as bitset8
} bitset8_t;


/**
 * Definition of a set of 16 bits with individual access.
 */
typedef struct {
    uint16_t bit0 : 1;  ///< access to bit 0
    uint16_t bit1 : 1;  ///< access to bit 1
    uint16_t bit2 : 1;  ///< access to bit 2
    uint16_t bit3 : 1;  ///< access to bit 3
    uint16_t bit4 : 1;  ///< access to bit 4
    uint16_t bit5 : 1;  ///< access to bit 5
    uint16_t bit6 : 1;  ///< access to bit 6
    uint16_t bit7 : 1;  ///< access to bit 7
    uint16_t bit8 : 1;  ///< access to bit 8
    uint16_t bit9 : 1;  ///< access to bit 9
    uint16_t bit10 : 1; ///< access to bit 10
    uint16_t bit11 : 1; ///< access to bit 11
    uint16_t bit12 : 1; ///< access to bit 12
    uint16_t bit13 : 1; ///< access to bit 13
    uint16_t bit14 : 1; ///< access to bit 14
    uint16_t bit15 : 1; ///< access to bit 15
} bitset16;


/**
 * Definition of a set of 16 bits with individual and uint16_t access.
 */
typedef union {
    uint16_t value;  ///< access to bits as uint16_t
    bitset16 bitset; ///< access to bits as bitset16
} bitset16_t;


#endif /* VARIABLESACCESS_H_ */
