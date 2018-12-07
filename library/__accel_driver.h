/*
    __accel_driver.h

-----------------------------------------------------------------------------

  This file is part of mikroSDK.
  
  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

/**
@file   __accel_driver.h
@brief    Accel Driver
@mainpage Accel Click
@{

@image html libstock_fb_view.jpg

@}

@defgroup   ACCEL
@brief      Accel Click Driver
@{

| Global Library Prefix | **ACCEL** |
|:---------------------:|:-----------------:|
| Version               | **1.0.0**    |
| Date                  | **Aug 2018.**      |
| Developer             | **Nenad Filipovic**     |

*/
/* -------------------------------------------------------------------------- */

#include "stdint.h"

#ifndef _ACCEL_H_
#define _ACCEL_H_

/** 
 * @macro T_ACCEL_P
 * @brief Driver Abstract type 
 */
#define T_ACCEL_P    const uint8_t*

/** @defgroup ACCEL_COMPILE Compilation Config */              /** @{ */

//  #define   __ACCEL_DRV_SPI__                            /**<     @macro __ACCEL_DRV_SPI__  @brief SPI driver selector */
   #define   __ACCEL_DRV_I2C__                            /**<     @macro __ACCEL_DRV_I2C__  @brief I2C driver selector */                                          
// #define   __ACCEL_DRV_UART__                           /**<     @macro __ACCEL_DRV_UART__ @brief UART driver selector */ 

                                                                       /** @} */
/** @defgroup ACCEL_VAR Variables */                           /** @{ */

extern const uint8_t _ACCEL_DEVID;
extern const uint8_t _ACCEL_POWER_CTL;
extern const uint8_t _ACCEL_DATA_FORMAT;
extern const uint8_t _ACCEL_BW_RATE;
extern const uint8_t _ACCEL_DATAX0;
extern const uint8_t _ACCEL_DATAX1;
extern const uint8_t _ACCEL_DATAY0;
extern const uint8_t _ACCEL_DATAY1;
extern const uint8_t _ACCEL_DATAZ0;
extern const uint8_t _ACCEL_DATAZ1;
extern const uint8_t _ACCEL_FIFO_CTL;
extern const uint8_t _ACCEL_SPEED;
extern const uint8_t _ACCEL_ERROR;
extern const uint8_t _ACCEL_I2C_ADDRESS_1;
extern const uint8_t _ACCEL_I2C_ADDRESS_0;

                                                                       /** @} */
/** @defgroup ACCEL_TYPES Types */                             /** @{ */



                                                                       /** @} */
#ifdef __cplusplus
extern "C"{
#endif

/** @defgroup ACCEL_INIT Driver Initialization */              /** @{ */

#ifdef   __ACCEL_DRV_SPI__
void accel_spiDriverInit(T_ACCEL_P gpioObj, T_ACCEL_P spiObj);
#endif
#ifdef   __ACCEL_DRV_I2C__
void accel_i2cDriverInit(T_ACCEL_P gpioObj, T_ACCEL_P i2cObj, uint8_t slave);
#endif
#ifdef   __ACCEL_DRV_UART__
void accel_uartDriverInit(T_ACCEL_P gpioObj, T_ACCEL_P uartObj);
#endif


/** @defgroup ACCEL_FUNC Driver Functions */                   /** @{ */

/**
 * @brief Function set command
 *
 * @param[in] address         Register address
 *
 * @param[in] writeCommand    Command to write
 *
 * Function write byte of data to Accel
 */
void accel_writeData( uint8_t address, uint8_t writeData );

/**
 * @brief Function read from register address
 *
 * @param[in] address         Register address
 *
 * @return    Data from addressed register in Accel
 *
 * Function read byte of data from register address of Accel
 */
uint8_t accel_readData( uint8_t address );

/**
 * @brief Initializes function
 *
 * Function initializes Accel register
 */
void accel_Init();

/**
 * @brief Check Accel ID
 *
 * @return      0 for OK; 1 for ERROR
 *
 * Function check Accel ID
 */
uint8_t accel_checkId();

/**
 * @brief Function read X axis
 *
 * @return         Value X axis
 *
 * Function read X axis from Accel
 */
uint16_t accel_readXaxis();

/**
 * @brief Function read Y axis
 *
 * @return         Value Y axis
 *
 * Function read Y axis from Accel
 */
uint16_t accel_readYaxis();

/**
 * @brief Function read Z axis
 *
 * @return         Value Z axis
 *
 * Function read Z axis from Accel
 */
uint16_t accel_readZaxis();





                                                                       /** @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif

/**
    @example Click_Accel_STM.c
    @example Click_Accel_TIVA.c
    @example Click_Accel_CEC.c
    @example Click_Accel_KINETIS.c
    @example Click_Accel_MSP.c
    @example Click_Accel_PIC.c
    @example Click_Accel_PIC32.c
    @example Click_Accel_DSPIC.c
    @example Click_Accel_AVR.c
    @example Click_Accel_FT90x.c
    @example Click_Accel_STM.mbas
    @example Click_Accel_TIVA.mbas
    @example Click_Accel_CEC.mbas
    @example Click_Accel_KINETIS.mbas
    @example Click_Accel_MSP.mbas
    @example Click_Accel_PIC.mbas
    @example Click_Accel_PIC32.mbas
    @example Click_Accel_DSPIC.mbas
    @example Click_Accel_AVR.mbas
    @example Click_Accel_FT90x.mbas
    @example Click_Accel_STM.mpas
    @example Click_Accel_TIVA.mpas
    @example Click_Accel_CEC.mpas
    @example Click_Accel_KINETIS.mpas
    @example Click_Accel_MSP.mpas
    @example Click_Accel_PIC.mpas
    @example Click_Accel_PIC32.mpas
    @example Click_Accel_DSPIC.mpas
    @example Click_Accel_AVR.mpas
    @example Click_Accel_FT90x.mpas
*/                                                                     /** @} */
/* -------------------------------------------------------------------------- */
/*
  __accel_driver.h

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