/****************************************************************************
* Title                 :   SPI Hardware Access Layer
* Filename              :   spi_hal.h
* Author                :   RL
* Origin Date           :   28/08/2015
* Notes                 :   None
*****************************************************************************/
/**************************CHANGE LIST **************************************
*
*    Date    Software Version    Initials   Description
*  23/08/15          0.1         RL      Generic SPI Interface
*
*****************************************************************************/
/**
 *  @file spi_hal.h
 *  @brief This represents a generic SPI read/write interface
 *  for multiple platforms
 *
 *  This implimentation assumes that all SPI bus hardware has been
 *  initialized outside of this module.
 *
 *  @date 25 Aug 2015
 *  @author Richard Lowe
 *  @copyright GNU Public License
 *
 *  @version .1 - Initial testing and verification
 *
 *  @note Test configuration:
 *   MCU:             STM32F107VC
 *   Dev.Board:       EasyMx Pro v7
 *   Oscillator:      72 Mhz internal
 *   Ext. Modules:    GPS Click
 *   SW:              ARM 4.5.2
 *
 */
#ifndef ADXL345_HAL_H
#define ADXL345_HAL_H

/******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>

/******************************************************************************
* Preprocessor Constants
*******************************************************************************/

/******************************************************************************
* Configuration Constants
*******************************************************************************/

/******************************************************************************
* Macros
*******************************************************************************/

/******************************************************************************
* Typedefs
*******************************************************************************/
typedef enum
{
    ACCEL_I2C,
    ACCEL_SPI4,
    ACCEL_SPI3
} accel_mode_t;


/******************************************************************************
* Variables
*******************************************************************************/

/******************************************************************************
* Function Prototypes
*******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief accel_hal_init
 * @param mode
 * @param address_id
 * @return
 */
int accel_hal_init( accel_mode_t mode, uint8_t address_id );

/**
 * @brief hw_spi_hal_read
 * @param address
 * @param num
 * @param buff
 */
void accel_hal_write( uint8_t address, uint16_t num,
                      uint8_t *data_out );

/**
 * @brief accel_hal_read
 * @param address
 * @param num
 * @param data_in
 */
void accel_hal_read( uint8_t address, uint16_t num, uint8_t *data_in );


#ifdef __cplusplus
} // extern "C"
#endif

#endif /*File_H_*/

/*** End of File **************************************************************/