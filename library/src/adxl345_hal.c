/*******************************************************************************
* Title                 :   System Initialization
* Filename              :   sys_init.c
* Author                :   RBL
* Origin Date           :   08/28/2015
* Notes                 :   None
*******************************************************************************/
/*************** MODULE REVISION LOG ******************************************
*
*    Date    Software Version    Initials   Description
*  XX/XX/XX    XXXXXXXXXXX         JWB      Module Created.
*
*******************************************************************************/
/**
 * @file adxl345_hal.c
 * @brief This module contains the
 *
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include "adxl345_hal.h"
#include "adxl345_common.h"
#include <string.h>

#if defined( __GNUC__ )
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#endif

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/
#define WRITE 0
#define READ 1

/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/


/******************************************************************************
* Module Typedefs
*******************************************************************************/


/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
#if defined( __MIKROC_PRO_FOR_ARM__ )
#if defined( STM32 )
static unsigned int( *i2c_start_p )( void );
static unsigned int( *i2c_write_p )( unsigned char slave_address,
                                     unsigned char *buffer,
                                     unsigned long count,
                                     unsigned long end_mode );
static void( *i2c_read_p )( unsigned char slave_address,
                            unsigned char *buffer,
                            unsigned long count,
                            unsigned long end_mode );
static unsigned int ( *spi_read_p )( unsigned int buffer );
static void ( *spi_write_p )( unsigned int data_out );
#elif defined( TI )
static void( *i2c_enable_p )( void );
static void( *i2c_disable_p )( void );
static void( *i2c_set_slave_address_p )( unsigned char slave_address,
        unsigned char dir );
static void( *i2c_write_p )( unsigned char data_out,
                             unsigned char mode );
static void( *i2c_read_p )( unsigned char *data,
                            unsigned char mode );
static unsigned int ( *spi_read_p )( unsigned int buffer );
static void ( *spi_write_p )( unsigned int data_out );
#else
static unsigned int( *i2c_start_p )( void );
static unsigned int( *i2c_write_p )( unsigned char slave_address,
                                     unsigned char *buffer,
                                     unsigned long count,
                                     unsigned long end_mode );
static void( *i2c_read_p )( unsigned char slave_address,
                            unsigned char *buffer,
                            unsigned long count,
                            unsigned long end_mode );
static unsigned int ( *spi_read_p )( unsigned int buffer );
static void ( *spi_write_p )( unsigned int data_out );
#endif

#elif  defined( __MIKROC_PRO_FOR_AVR__ )
static unsigned char( *i2c_busy_p )( void );
static unsigned char( *i2c_status_p )( void );
static unsigned char( *i2c_start_p )( void );
static void( *i2c_stop_p )( void );
static void( *i2c_close_p )( void );
static void( *i2c_write_p )( unsigned char data_out );
static unsigned char( *i2c_read_p )( unsigned char ack );
static unsigned char( *spi_read_p )( unsigned char rx );
static unsigned char( *spi_write_p )( unsigned char rx );

#elif  defined( __MIKROC_PRO_FOR_PIC__ )
static unsigned char( *is_idle_i2c_p )( void );
static unsigned char( *i2c_start_p )( void );
static void( *i2c_stop_p )( void );
static void( *i2c_restart_p )( void );
static unsigned char( *i2c_write_p )( unsigned char data_out );
static unsigned char( *i2c_read_p )( unsigned char ack );
static unsigned char( *spi_read_p )( unsigned char _data );
static void ( *spi_write_p )( unsigned char data_out );

#elif defined( __MIKROC_PRO_FOR_PIC32__ )
static unsigned int( *is_idle_i2c_p )( void );
static unsigned int( *i2c_start_p )( void );
static void( *i2c_stop_p )( void );
static unsigned int( *i2c_restart_p )( void );
static unsigned int( *i2c_write_p )( unsigned char data_out );
static unsigned char( *i2c_read_p )( unsigned int ack );
static unsigned long( *spi_read_p )( unsigned long buffer );
static void ( *spi_write_p )( unsigned long data_out );

#elif defined( __MIKROC_PRO_FOR_DSPIC__ )
static unsigned int( *is_idle_i2c_p )( void );
static unsigned int( *i2c_start_p )( void );
static void( *i2c_stop_p )( void );
static void( *i2c_restart_p )( void );
static unsigned int( *i2c_write_p )( unsigned char data_out );
static unsigned char( *i2c_read_p )( unsigned int ack );
static unsigned int( *spi_read_p )( unsigned int buffer );
static void ( *spi_write_p )( unsigned int data_out );

#elif defined( __MIKROC_PRO_FOR_8051__ )
static unsigned char( *i2c_busy_p )( void );
static unsigned char ( *i2c_status_p )( void );
static unsigned char( *i2c_start_p )( void );
static void( *i2c_stop_p )( void );
static void( *i2c_close_p )( void );
static void( *i2c_write_p )( unsigned char data_out );
static unsigned char( *i2c_read_p )( unsigned char ack );

#elif defined( __MIKROC_PRO_FOR_FT90x__ )
static void( *i2c_soft_reset_p )( void );
static void( *i2c_set_slave_address_p )( unsigned char slave_address );
static unsigned char( *i2c_write_p )( unsigned char data_out );
static unsigned char( *i2c_read_p )( unsigned char *data_in );
static unsigned char( *i2c_write_bytes_p )( unsigned char *buffer,
        unsigned int count );
static unsigned char( *i2c_read_bytes_p )( unsigned char *buffer,
        unsigned int count );
static unsigned char( *i2c_write_10bit_p )( unsigned char data_out,
        unsigned int address_10bit );
static unsigned char( *i2c_read_10bit_p )( unsigned char *data_in,
        unsigned int address_10bit );
static unsigned char( *spi_read_p )( unsigned char dummy );
static void( *spi_write_p )( unsigned char dataOut );

#endif

#if defined( __MIKROC_PRO_FOR_ARM__ ) || \
    defined( __MIKROC_PRO_FOR_AVR__ ) || \
    defined( __MIKROC_PRO_FOR_PIC__ ) || \
    defined( __MIKROC_PRO_FOR_PIC32__ ) || \
    defined( __MIKROC_PRO_FOR_DSPIC__ ) || \
    defined( __MIKROC_PRO_FOR_8051__ )  || \
    defined( __MIKROC_PRO_FOR_FT90x__ )
extern sfr sbit ACCEL_CS;
#endif

static accel_mode_t current_mode;
static uint8_t i2c_address;

/******************************************************************************
* Function Prototypes
*******************************************************************************/
static void cs_low( void );
static void cs_high( void );

#if defined( __GNUC__ )
static void transfer( int fd, uint8_t const *tx,
                      uint8_t const *rx, size_t len );
#endif
/******************************************************************************
* Function Definitions
*******************************************************************************/
static void cs_low()
{
#if __GNUC__

#else
    ACCEL_CS = 0;
    Delay_10us();
#endif
}

static void cs_high()
{
#if __GNUC__

#else
    Delay_10us();
    ACCEL_CS = 1;
#endif
}

#if defined( __GNUC__ )
static int spi_transfer( int fd, uint8_t const *tx,
                         uint8_t const *rx, size_t len )
{
    int ret = 0;

    //    struct spi_ioc_transfer tr = {
    //        .tx_buf = ( unsigned long )tx;
    //        .rx_buf = ( unsigned long )rx;
    //        .len = len;
    //        .delay_usecs = delay;
    //        .speed_hz = speed;
    //        .bits_per_word = bit;
    //    };

    //    ret = ioctl( fd, SPI_IOC_MESSAGE(1), &tr );

    return ret;

}
#endif

/********************************
 *******Implimentations**********
 *******************************/
int accel_hal_init( accel_mode_t mode, uint8_t address_id )
{
    current_mode = MIN( ACCEL_SPI3, mode );

    switch( current_mode )
    {
        case ACCEL_I2C:
#if defined( __MIKROC_PRO_FOR_ARM__ )
#if defined( STM32 )
            i2c_start_p = I2C_Start_Ptr;
            i2c_write_p = I2C_Write_Ptr;
            i2c_read_p = I2C_Read_Ptr;
#elif defined( TI )
            i2c_enable_p = I2C_Enable_Ptr;
            i2c_disable_p = I2C_Disable_Ptr;
            i2c_set_slave_address_p = I2C_Master_Slave_Addr_Set_Ptr;
            i2c_write_p = I2C_Write_Ptr;
            i2c_read_p = I2C_Read_Ptr;
#endif

#elif defined( __MIKROC_PRO_FOR_AVR__ )
            i2c_busy_p = TWI_Busy;
            i2c_status_p = TWI_Status;
            i2c_close_p = TWI_Close;
            i2c_start_p = TWI_Start;
            i2c_stop_p = TWI_Stop;
            i2c_write_p = TWI_Write;
            i2c_read_p = TWI_Read;

#elif defined( __MIKROC_PRO_FOR_PIC__ )


#elif defined( __MIKROC_PRO_FOR_PIC32__ )
            i2c_is_idle_p = I2C_Is_Idle_Ptr;
            i2c_start_p = I2C_Start_Ptr;
            i2c_stop_p = I2C_Stop_Ptr;
            i2c_restart_p = I2C_Restart_Ptr;
            i2c_write_p = I2C_Write_Ptr;

#elif defined( __MIKROC_PRO_FOR_DSPIC__ )


#elif defined( __MIKROC_PRO_FOR_8051__ )
            i2c_busy_p = TWI_Busy;
            i2c_status_p = TWI_Status;
            i2c_close_p = TWI_Close;
            i2c_start_p = TWI_Start;
            i2c_stop_p = TWI_Stop;
            i2c_write_p = TWI_Write;
            i2c_read_p = TWI_Read;

#elif defined( __MIKROC_PRO_FOR_FT90x__ )
            i2c_soft_reset_p = I2CM_Soft_Reset_Ptr;
            i2c_set_slave_address_p = I2CM_Set_Slave_Address_Ptr;
            i2c_write_p = I2CM_Write_Ptr;
            i2c_read_p = I2CM_Read_Ptr;
            i2c_write_p_bytes = I2CM_Write_Bytes_Ptr;
            i2c_read_p_bytes = I2CM_Read_Bytes_Ptr;
            i2c_write_p_10bit = I2CM_Write_10Bit_Ptr;
            i2c_read_p_10bit = I2CM_Read_10Bit_Ptr;
#endif

#if defined( __MIKROC_PRO_FOR_ARM__ ) || defined( __MIKROC_PRO_FOR_FT90x__ )
            i2c_address = address_id;
#else
            i2c_address = ( address_id << 1 );
#endif
            break;
        case ACCEL_SPI4:

        case ACCEL_SPI3:
        #ifdef __GNUC__

        #else
        spi_read_p = SPI_Rd_Ptr;
        spi_write_p = SPI_Wr_Ptr;
        #endif
            break;
    };
    
    cs_high();

    return 0;
}


void accel_hal_write( uint8_t address, uint16_t num, uint8_t *data_out )
{
    uint8_t *ptr = data_out;
    
    switch( current_mode )
    {
        case ACCEL_I2C:
        {
            uint8_t buffer[6];

            buffer[0] = address;
            memcpy( &buffer[1], ptr, num );
#if defined(__MIKROC_PRO_FOR_ARM__)
            i2c_start_p();
            i2c_write_p( i2c_address, buffer, num + 1, END_MODE_STOP );

#elif defined(__MIKROC_PRO_FOR_FT90x__)
            i2c_set_slave_sddress_p( i2c_address );
            i2c_write_p( buffer, num + 1 );

#elif defined(__MIKROC_PRO_FOR_AVR__)   || \
defined(__MIKROC_PRO_FOR_8051__)  || \
defined(__MIKROC_PRO_FOR_DSPIC__) || \
defined(__MIKROC_PRO_FOR_PIC32__) || \
defined(__MIKROC_PRO_FOR_PIC__)
            i2c_start_p();
            i2c_write_p( i2c_address | WRITE );
            i2c_write_p( address );

            while( num-- )
                i2c_write_p( *( ptr++ ) );

            i2c_stop_p();

#elif defined( __GNUC__)
            printf( "Start\n" );
            printf( "Address: 0x%02x\n", address );
            while( num-- )
                printf( "\tData: 0x%02x\n", *ptr++ );
#endif
        }
        break;
        case ACCEL_SPI4:

            cs_low();
            spi_write_p( address );
            
            while( num-- )
                spi_write_p( *( ptr++ ) );
            
            cs_high();
            
            break;
        case ACCEL_SPI3:

            break;
    };
}


void accel_hal_read( uint8_t address, uint16_t num, uint8_t *data_in )
{
    switch( current_mode )
    {
        case ACCEL_I2C:
        {
            uint8_t buffer[1];
            buffer[0] = address;

#if defined( __MIKROC_PRO_FOR_ARM__ )
            i2c_start_p();              // issue I2C start signal
            i2c_write_p( i2c_address, buffer, 1, END_MODE_RESTART );
            i2c_read_p( i2c_address, data_in, num, END_MODE_STOP );

#elif defined(__MIKROC_PRO_FOR_FT90x__)
            I2CM1_Set_Slave_Address( i2c_address );

            i2c_write_p( buffer[0], 1 );
            i2c_read_p( data_in, num );

#elif defined(__MIKROC_PRO_FOR_AVR__) || \
defined(__MIKROC_PRO_FOR_PIC32__)
            i2c_start_p();
            i2c_write_p( i2c_address | WRITE );
            i2c_write_p( buffer[0] );
            i2c_start_p();

            i2c_write_p( i2c_address | READ );

            while( num >= 2 )
            {
                *( data_in++ ) = i2c_read_p( 1 );
                num--;
            }

            *buff = i2c_read_p( 0 );
            i2c_stop_p();

#elif defined(__MIKROC_PRO_FOR_PIC__)  || \
defined(__MIKROC_PRO_FOR_DSPIC__)
            i2c_start_p();
            i2c_write_p( i2c_address | WRITE );
            i2c_write_p( buffer[0] );
            i2c_repeat_p();
            i2c_write_p( i2c_address | READ );

            while( num >= 2 )
                *( data_in++ ) = i2c_read_p( 1 );

            *data_in = i2c_read_p( 0 );
            i2c_stop_p();

#elif defined( __GNUC__ )
            int input;

            printf( "Read from address 0x%02x\n", buffer[0] );

            while( num-- )
            {
                scanf( "%d", &input );
                *( data_in++ ) = input;
            }
#endif
        }
        break;
        case ACCEL_SPI4:
            cs_low();
            
            spi_read_p( address );
            
            while( num-- )
                *( data_in++ ) = spi_read_p( 0x00 );
            
            cs_high();
            break;
        case ACCEL_SPI3:

            break;
    };
}
/*************** END OF FUNCTIONS ***************************************************************************/