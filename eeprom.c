/*
    Routines for driving an i2c / iic eeprom.
    Only tested on 24c02x series.
*/

#include "eeprom.h"

// Initialize IIC pins ready for eeprom
void init_ee(void)
{
    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
}

// Read a byte from eeprom
// If return value is -ve an error occured.
int read_ee(uint        iic_addr,       // IIC address
            uint16_t    ee_addr,        // Byte address within the eeprom 
            uint8_t     *toread)        // Pointer to the byte to read into
{
    uint8_t from = (ee_addr & 0x00FF);
    int     result;

    // Send the read command along with the address to read
    result = i2c_write_blocking(i2c0,iic_addr,&from,1,true);

    // if that worked read the data
    if(result >= 0)
    {
        result = i2c_read_blocking(i2c0,iic_addr,toread,1,false);
    }

    return result;
}

// Read a block of bytes from eeprom
// If return value is -ve an error occured.
int read_ee_bytes(uint      iic_addr,       // IIC address
                  uint16_t  ee_addr,        // Byte address within the eeprom 
                  uint8_t   *toread,        // Pointer to the destination buffer
                  uint16_t  bytecount)      // count of bytes to read  
{
    int         result;

    do
    {
        result = read_ee(iic_addr,ee_addr,toread);
        toread++;
        ee_addr++;
        bytecount--;
    } while ((result >= 0) && (bytecount > 0));

    return result;
}

// Write a byte to eeprom
// If return value is -ve an error occured.
int write_ee(uint       iic_addr,       // IIC address
             uint16_t   ee_addr,        // Byte address within the eeprom 
             uint8_t    towrite)        // Byte to be written
{
    int     result;
    uint8_t wrbuf[2];

    int     retries;

    // 2 byte write buffer containing address to write and data to write
    wrbuf[0] = ee_addr & 0x00FF;
    wrbuf[1] = towrite;

    // initiate the write    
    result = i2c_write_blocking(i2c0,iic_addr,wrbuf,sizeof(wrbuf),false);

    // poll the eeprom checking to see if the write has completed
    retries=3;
    do
    {
        sleep_ms(10);
        result = i2c_write_blocking(i2c0,iic_addr,wrbuf,1,false);
        retries--;
    } while ((result < 0 ) && (retries != 0));

    return result;
}


// Write a block of bytes from eeprom
// If return value is -ve an error occured.
int write_ee_bytes(uint     iic_addr,       // IIC address
                   uint16_t ee_addr,        // Byte address within the eeprom 
                   uint8_t  *towrite,       // Pointer to the source buffer
                   uint16_t bytecount)      // count of bytes to write  
{
    int         result;

    do
    {
        result =     write_ee(iic_addr,ee_addr,*towrite);
        towrite++;
        ee_addr++;
        bytecount--;
    } while ((result >= 0) && (bytecount > 0));

    return result;
}



// Write a string of bytes to eeprom
// If return value is -ve an error occured.
int write_str_ee(uint       iic_addr,       // IIC address
                 uint16_t   ee_addr,        // Byte address withing the eeprom 
                 char       *towrite)       // String to write
{
    uint    Idx;
    int     result = 0;

    for(Idx=0; Idx < strlen(towrite); Idx++)
    {
        result = write_ee(EE_ADDRESS,ee_addr++,towrite[Idx]);
    }

    return result;
}
