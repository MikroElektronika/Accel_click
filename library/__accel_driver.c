/*
    __accel_driver.c

-----------------------------------------------------------------------------

  This file is part of mikroSDK.

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

#include "__accel_driver.h"
#include "__accel_hal.c"

/* ------------------------------------------------------------------- MACROS */



/* ---------------------------------------------------------------- VARIABLES */

#ifdef   __ACCEL_DRV_I2C__
static uint8_t _slaveAddress;
#endif

// ADXL345 Register Definition
const uint8_t _ACCEL_DEVID          = 0x00;
const uint8_t _ACCEL_POWER_CTL      = 0x2D;
const uint8_t _ACCEL_DATA_FORMAT    = 0x31;
const uint8_t _ACCEL_BW_RATE        = 0x2C;
const uint8_t _ACCEL_DATAX0         = 0x32;
const uint8_t _ACCEL_DATAX1         = 0x33;
const uint8_t _ACCEL_DATAY0         = 0x34;
const uint8_t _ACCEL_DATAY1         = 0x35;
const uint8_t _ACCEL_DATAZ0         = 0x36;
const uint8_t _ACCEL_DATAZ1         = 0x37;
const uint8_t _ACCEL_FIFO_CTL       = 0x38;
const uint8_t _ACCEL_SPEED          = 0x0F;
const uint8_t _ACCEL_ERROR          = 0x02;
const uint8_t _ACCEL_I2C_ADDRESS_1    = 0x3A;         //address 0x3A
const uint8_t _ACCEL_I2C_ADDRESS_0    = 0x53;         //alternate 0x53


/* -------------------------------------------- PRIVATE FUNCTION DECLARATIONS */



/* --------------------------------------------- PRIVATE FUNCTION DEFINITIONS */



/* --------------------------------------------------------- PUBLIC FUNCTIONS */

#ifdef   __ACCEL_DRV_SPI__

void accel_spiDriverInit(T_ACCEL_P gpioObj, T_ACCEL_P spiObj)
{
    hal_spiMap( (T_HAL_P)spiObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif
#ifdef   __ACCEL_DRV_I2C__

void accel_i2cDriverInit(T_ACCEL_P gpioObj, T_ACCEL_P i2cObj, uint8_t slave)
{
    _slaveAddress = slave;
    hal_i2cMap( (T_HAL_P)i2cObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif
#ifdef   __ACCEL_DRV_UART__

void accel_uartDriverInit(T_ACCEL_P gpioObj, T_ACCEL_P uartObj)
{
    hal_uartMap( (T_HAL_P)uartObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif



/* ----------------------------------------------------------- IMPLEMENTATION */


/* Generic write data function */
void accel_writeData( uint8_t address, uint8_t writeCommand )
{
    uint8_t buffer[2];
    buffer[0]= address;
    buffer[1]= writeCommand;

    hal_i2cStart();
    hal_i2cWrite( _slaveAddress, buffer, 2, END_MODE_STOP );
}

/* Generic read data function */
uint8_t accel_readData( uint8_t address )
{
    uint8_t temp;
    temp = address;

    hal_i2cStart();
    hal_i2cWrite( _slaveAddress, &temp, 1, END_MODE_RESTART );
    hal_i2cRead( _slaveAddress, &temp, 1, END_MODE_STOP );

    return temp;
}

/* Initializes function */
void accel_Init()
{
    accel_writeData( _ACCEL_POWER_CTL , 0x00 );         // POWER_CTL reg: standby mode to configure the device
    accel_writeData( _ACCEL_DATA_FORMAT , 0x08 );       // Full resolution, +/-2g, 4mg/LSB, right justified
    accel_writeData( _ACCEL_BW_RATE , 0x0A );           // Set 100 Hz data rate
    accel_writeData( _ACCEL_FIFO_CTL , 0x80 );          // Stream mode
    accel_writeData( _ACCEL_POWER_CTL , 0x08 );         // POWER_CTL reg: measurement mode
}

/* Function check Accel ID */
uint8_t accel_checkId()
{
    uint8_t id = 0x00;
    accel_writeData( _ACCEL_POWER_CTL, 0x00 );
    
    id = accel_readData( _ACCEL_DEVID );

    if (id != 0xE5)
    {
        return _ACCEL_ERROR;
    }
    else
    {
        accel_Init();
        return _ACCEL_DEVID;
    }
}

/* Function read X axis */
uint16_t accel_readXaxis()
{
    uint16_t Out_x;
    uint8_t buffer[2];
    Out_x = 0x0000;

    buffer[0] = accel_readData( _ACCEL_DATAX1 );
    buffer[1] = accel_readData( _ACCEL_DATAX0 );

    Out_x = buffer[0];
    Out_x <<= 8;
    Out_x |= buffer[1];

    return Out_x;
}

/* Function read Y axis */
uint16_t accel_readYaxis()
{
    uint16_t Out_y;
    uint8_t buffer[2];
    Out_y = 0x0000;

    buffer[0] = accel_readData( _ACCEL_DATAY1 );
    buffer[1] = accel_readData( _ACCEL_DATAY0 );

    Out_y = buffer[0];
    Out_y <<= 8;
    Out_y |= buffer[1];


    return Out_y;
}

/* Function read Z axis */
uint16_t accel_readZaxis()
{
    uint16_t Out_z;
    uint8_t buffer[2];
    Out_z = 0x0000;

    buffer[0] = accel_readData( _ACCEL_DATAZ1 );
    buffer[1] = accel_readData( _ACCEL_DATAZ0 );

    Out_z = buffer[0];
    Out_z <<= 8;
    Out_z |= buffer[1];

    return Out_z;
}




/* -------------------------------------------------------------------------- */
/*
  __accel_driver.c

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement:
   This product includes software developed by the MikroElektonika.

4. Neither the name of the MikroElektonika nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY MIKROELEKTRONIKA ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL MIKROELEKTRONIKA BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------- */