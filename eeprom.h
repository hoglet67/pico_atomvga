/*
    Routines for driving an i2c / iic eeprom.
    Only tested on 24c02x series.
*/
#ifndef __EEPROM_IIC_H__
#define __EEPROM_IIC_H__

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 20
#define I2C_SCL 21

// Address
#define EE_ADDRESS  0x50

// Initialize IIC pins ready for eeprom
void init_ee(void);

// Read a byte from eeprom
// If return value is -ve an error occured.
int read_ee(uint        iic_addr,       // IIC address
            uint16_t    ee_addr,        // Byte address withing the eeprom 
            uint8_t     *toread);       // Pointer to the byte to read into

// Read a block of bytes from eeprom
// If return value is -ve an error occured.
int read_ee_bytes(uint      iic_addr,       // IIC address
                  uint16_t  ee_addr,        // Byte address within the eeprom 
                  uint8_t   *toread,        // Pointer to the destination buffer
                  uint16_t  bytecount);     // count of bytes to read  

// Write a byte to eeprom
// If return value is -ve an error occured.
int write_ee(uint       iic_addr,       // IIC address
             uint16_t   ee_addr,        // Byte address withing the eeprom 
             uint8_t    towrite);       // Byte to be written

// Write a block of bytes from eeprom
// If return value is -ve an error occured.
int write_ee_bytes(uint     iic_addr,       // IIC address
                   uint16_t ee_addr,        // Byte address within the eeprom 
                   uint8_t  *towrite,       // Pointer to the source buffer
                   uint16_t bytecount);     // count of bytes to write  

// Write a string of bytes to eeprom
// If return value is -ve an error occured.
int write_str_ee(uint       iic_addr,       // IIC address
                 uint16_t   ee_addr,        // Byte address withing the eeprom 
                 char       *towrite);      // String to write


#endif
