/*******************************************************************************
* Title                 :   ADXL345 Driver
* Filename              :   adxl345.c
* Author                :   Richard Lowe
* Origin Date           :   19/08/2015
* Notes                 :   None
*******************************************************************************/
/*************** MODULE REVISION LOG ******************************************
*
*    Date    Software Version    Initials   Description
*  19/08/15         .1           RL         Module Created
*
*******************************************************************************/
/**
 * @file adxl345.c
 * @brief Implimetation of adxl345
 *
 *
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include <math.h>
#include <stddef.h>
#include "adxl345.h"
#include "adxl345_hal.h"
#include "adxl345_common.h"

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/
/**
 *  @defgroup Registers
 *  <strong>REGISTER DEFINITIONS</strong>
 *        @{
 */
/**< 0x00 - Device ID | Type R | Reset Value - 11100101 */
#define ADXL345_DEVID               0x00

/**< 0x01 - 0x1c Reserved; do not access*/
#define ADXL345_RESERVED1           0x01
#define ADXL345_THRESH_TAP          0x1d /**< 0x1d - Tap threshold | Type R/W | Reset Value - 00000000 | @ref THRESH_TAP "Threshold Description" */
#define ADXL345_OFSX                0x1e /**< 0x1e - X-axis offset | Type R/W | Reset Value - 00000000 | @ref OFFSET "Offset Description" */
#define ADXL345_OFSY                0x1f /**< 0x1f - Y-axis offset | Type R/W | Reset Value - 00000000 | @ref OFFSET "Offset Description" */
#define ADXL345_OFSZ                0x20 /**< 0x20 - Z-axis offset | Type R/W | Reset Value - 00000000 | @ref OFFSET "Offset Description" */
#define ADXL345_DUR                 0x21 /**< 0x21 - Tap duration | Type R/W | Reset Value - 00000000 | @ref DUR "Duration Description */
#define ADXL345_LATENT              0x22 /**< 0x22 - Tap latency | Type R/W | Reset Value - 00000000 | @ref LATENT "Latency Description */
#define ADXL345_WINDOW              0x23 /**< 0x23 - Tap window | Type R/W | Reset Value - 00000000 */
#define ADXL345_THRESH_ACT          0x24 /**< 0x24 - Activity threshold | Type R/W | Reset Value - 00000000 */
#define ADXL345_THRESH_INACT        0x25 /**< 0x25 - Inactivity threshold | Type R/W | Reset Value - 00000000 */
#define ADXL345_TIME_INACT          0x26 /**< 0x26 - Inactivity time | Type R/W | Reset Value - 00000000 */
/**< 0x27 - Axis enable control for activity and inactivity
 * detection | Type R/W | Reset Value - 00000000
*/
#define ADXL345_ACT_INACT_CTL       0x27
#define ADXL345_ACT_INACT_CTL_INACT_Z_BIT 0
#define ADXL345_ACT_INACT_CTL_INACT_Y_BIT 1
#define ADXL345_ACT_INACT_CTL_INACT_X_BIT 2
#define ADXL345_ACT_INACT_CTL_INACT_ACDC_BIT 3
#define ADXL345_ACT_INACT_CTL_ACT_Z_BIT   4
#define ADXL345_ACT_INACT_CTL_ACT_Y_BIT   5
#define ADXL345_ACT_INACT_CTL_ACT_X_BIT   6
#define ADXL345_ACT_INACT_CTL_ACT_ACDC_BIT 7
/**< 0x28 - Free-fall threshold | Type R/W | Reset Value - 00000000 */
#define ADXL345_THRESH_FF           0x28
#define ADXL345_TIME_FF             0x29 /**< 0x29 - Free-fall time | Type R/W | Reset Value - 00000000 */
/**< 0x2a - Axis control for single tap/double
 * tap | Type R/W | Reset Value - 00000000
*/
#define ADXL345_TAP_AXES            0x2a
#define ADXL345_TAP_AXES_TAP_Z_BIT    0
#define ADXL345_TAP_AXES_TAP_Y_BIT    1
#define ADXL345_TAP_AXES_TAP_X_BIT    2
#define ADXL345_TAP_AXES_SUPPRESS_BIT 3
/**< 0x2b - Source of single tap/double tap
 * | Type R | Reset Value - 00000000
*/
#define ADXL345_ACT_TAP_STATUS      0x2b
#define ADXL345_ACT_TAP_STATUS_TAPZ_BIT 0
#define ADXL345_ACT_TAP_STATUS_TAPY_BIT 1
#define ADXL345_ACT_TAP_STATUS_TAPX_BIT 2
#define ADXL345_ACT_TAP_STATUS_ASLEEP_BIT 3
#define ADXL345_ACT_TAP_STATUS_ACTZ_BIT 4
#define ADXL345_ACT_TAP_STATUS_ACTY_BIT 5
#define ADXL345_ACT_TAP_STATUS_ACTX_BIT 6
/**< 0x2c - Data rate and power mode control|
 * Type R/W | Reset Value - 00001010
*/
#define ADXL345_BW_RATE             0x2c
#define ADXL345_BW_RATE_RATE        0x0F
#define ADXL345_BW_RATE_LOW_POWER_BIT 4
/**< 0x2d - Power-saving features control | Type R/W
 * | Reset Value - 00000000
*/
#define ADXL345_POWER_CTL           0x2d
#define ADXL345_POWER_CTL_WAKEUP    0x03
#define ADXL345_POWER_CTL_SLEEP_BIT   2
#define ADXL345_POWER_CTL_MEASURE_BIT 3
#define ADXL345_POWER_CTL_AUTOSLEEP_BIT 4
#define ADXL345_POWER_CTL_LINK_BIT    5
/**< 0x2e - Interrupt enable control | Type R/W |
 * Reset Value - 00000000
*/
#define ADXL345_INT_ENABLE          0x2e
/**< 0x2f - Interrupt mapping control | Type R/W |
 * Reset Value - 00000000
*/
#define ADXL345_INT_MAP             0x2f
/**< 0x30 - Source of interrupts | Type R |
 * Reset Value - 00000010
*/
#define ADXL345_INT_SOURCE          0x30
/**< 0x31 - Data format control | Type R/W |
 * Reset Value - 00000000 */
#define ADXL345_DATA_FORMAT         0x31
#define ADXL345_DATA_FORMAT_RANGE   0x03
#define ADXL345_DATA_FORMAT_JUSTIFY_BIT    2
#define ADXL345_DATA_FORMAT_FULL_RES_BIT   3
#define ADXL345_DATA_FORMAT_INT_INVERT_BIT 5
#define ADXL345_DATA_FORMAT_SPI_BIT        6
#define ADXL345_DATA_FORMAT_SELF_TEST_BIT  7
/**< 0x32 - X-Axis Data 0 | Type R | Reset Value - 00000000 */
#define ADXL345_DATAX0              0x32
#define ADXL345_DATAX1              0x33 /**< 0x33 - X-Axis Data 1 | Type R | Reset Value - 00000000 */
#define ADXL345_DATAY0              0x34 /**< 0x34 - Y-Axis Data 0 | Type R | Reset Value - 00000000 */
#define ADXL345_DATAY1              0x35 /**< 0x35 - Y-Axis Data 1 | Type R | Reset Value - 00000000 */
#define ADXL345_DATAZ0              0x36 /**< 0x36 - Z-Axis Data 0 | Type R | Reset Value - 00000000 */
#define ADXL345_DATAZ1              0x37 /**< 0x37 - Z-Axis Data 1 | Type R | Reset Value - 00000000 */
/**< 0x38 - FIFO control | Type R/W | Reset Value - 00000000 */
#define ADXL345_FIFO_CTL            0x38
#define ADXL345_FIFO_CTL_FIFO_MODE  0xC0
#define ADXL345_FIFO_CTL_TRIGGER_BIT 5
#define ADXL345_FIFO_CTL_SAMPLES    0x1F
/**< 0x39 - FIFO status | Type R | Reset Value - 00000000 */
#define ADXL345_FIFO_STATUS         0x39
/**@}*/

/**
 * @defgroup InterruptPosition
 * @{
*/
#define ADXL345_INT_DATA_READY_BIT 0x07
#define ADXL345_INT_SINGLE_TAP_BIT 0x06
#define ADXL345_INT_DOUBLE_TAP_BIT 0x05
#define ADXL345_INT_ACTIVITY_BIT   0x04
#define ADXL345_INT_INACTIVITY_BIT 0x03
#define ADXL345_INT_FREE_FALL_BIT  0x02
#define ADXL345_INT_WATERMARK_BIT  0x01
#define ADXL345_INT_OVERRUNY_BIT   0x00
/**@}*/

#define ADXL345_TO_READ 6  // num of bytes we are going to read each time (two bytes for each axis)


/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
static double gains[3] = 
{
    0.00376390,
    0.00376009,
    0.00349265 
}; // counts to Gs

/******************************************************************************
* Function Prototypes
*******************************************************************************/
/**
 * @brief Sets register bit value
 * @param Address
 * @param bit position
 * @param state - true or false
 */
static inline void set_register_bit(uint8_t reg_address, uint8_t bit_pos,
                                     bool state );
/**
 * @brief Retrieve register bit value
 * @param Address
 * @param bit possition
 * @return
 */
static inline bool get_register_bit(uint8_t reg_address, uint8_t bit_pos );
/**
 *  @brief Sets the SPI bit
 *
 *  @param[in] bool spiBit - true sets the device to 3-wire mode, false sets the device to 4-wire SPI mode
 *
 */
static inline void set_spi_bit( bool spiBit );

/******************************************************************************
* Function Definitions
*******************************************************************************/
// Private
static inline void set_register_bit(uint8_t reg_address, uint8_t bit_pos,
                                     bool state )
{
    uint8_t reading = 0;

    accel_hal_read( reg_address, 1, &reading );

    if( state )
        // forces nth bit of _b to be 1. all other bits left alone.
        reading |= ( 1 << bit_pos );
    else
        // forces nth bit of _b to be 0. all other bits left alone.
        reading &= ~( 1 << bit_pos );

    accel_hal_write( reg_address, 1, &reading );
}

static inline bool get_register_bit(uint8_t reg_address, uint8_t bit_pos )
{
    uint8_t reading = 0;

    accel_hal_read( reg_address, 1, &reading );

    return ( ( reading >> bit_pos ) & 0x01 );
}

// Sets the SPI bit
// if set to true it sets the device to 3-wire mode
// if set to false it sets the device to 4-wire SPI mode
static inline void set_spi_bit( bool spiBit )
{
    set_register_bit( ADXL345_DATA_FORMAT,
                      ADXL345_DATA_FORMAT_SPI_BIT,
                      spiBit );
}

/***********************************
 *****  Implimentations ************
 ***** ****************************/
int adxl345_init( adxl345_mode_t mode, uint8_t address )
{
    uint8_t setting = 0;

    if( accel_hal_init( mode, address ) )
        return ADXL345_ERROR;
    
    accel_hal_read( ADXL345_DEVID, 1, &setting );
    
    if( setting != 0xE5 )
        return ADXL345_ERROR;
    
    set_spi_bit( ( mode == ADXL345_MODE_SPI3 ) ? true : false );




    adxl345_enable_measure( false );
    adxl345_set_full_resolution( true );
    adxl345_set_rate( ADXL345_RATE_50HZ );
    
    return ADXL345_OK;
}

void adx1345_wakeup(adxl345_reading_rate_t reads_while_sleep)
{
    uint8_t reading;

    reads_while_sleep = MIN( reads_while_sleep, ADXL345_SLEEP_1HZ );
    accel_hal_read( ADXL345_POWER_CTL, 1, &reading );
    reading &= ~ADXL345_POWER_CTL_WAKEUP;
    reading |= reads_while_sleep;
    accel_hal_write( ADXL345_POWER_CTL, 1, &reading );
}

void adxl345_enable_sleep( bool sleep )
{
    set_register_bit( ADXL345_POWER_CTL,
                      ADXL345_POWER_CTL_SLEEP_BIT,
                      sleep );
}

void adxl345_enable_measure( bool measure )
{
    set_register_bit( ADXL345_POWER_CTL,
                      ADXL345_POWER_CTL_MEASURE_BIT,
                      measure );
}

void adx1345_enable_autosleep( bool autosleep )
{
    adxl345_enable_measure( false );
    set_register_bit( ADXL345_POWER_CTL,
                      ADXL345_POWER_CTL_AUTOSLEEP_BIT,
                      autosleep );
    adxl345_enable_measure( true );
}


// Reads the acceleration into three variable x, y and z
void adxl345_read_accelxyz( int16_t *xyz )
{
    adxl345_read_accel( xyz, xyz + 1, xyz + 2 );
}

void adxl345_read_accel( int16_t *x, int16_t *y, int16_t *z )
{
    uint8_t reading_buffer[ADXL345_TO_READ] = {0};  // 6 chars buffer
    
    //read the acceleration data from the ADXL345
    accel_hal_read( ADXL345_DATAX0, ADXL345_TO_READ, reading_buffer );

    // each axis reading comes in > 8 bit resolution, ie 2 chars.
    // Least Significat  char first!!
    // thus we are converting both  chars in to one int
    *x = ( ( ( int16_t ) reading_buffer[1] ) << 8 ) | reading_buffer[0];
    *y = ( ( ( int16_t ) reading_buffer[3] ) << 8 ) | reading_buffer[2];
    *z = ( ( ( int16_t ) reading_buffer[5] ) << 8 ) | reading_buffer[4];
}

void adxl345_read_g_xyz( double *xyz )
{
    int i = 0;
    int16_t xyz_int[3] = {0};

    adxl345_read_accelxyz( xyz_int );

    for( i = 0; i < 3; i++ )
        xyz[i] = ( double )xyz_int[i] * gains[i];
}


// Sets the THRESH_TAP  char value
// it should be between 0 and 255
// the scale factor is 62.5 mg/LSB
// A value of 0 may result in undesirable behavior
void adxl345_set_tap_threshold( uint8_t tap_threshold )
{  
    tap_threshold = CONSTRAIN( tap_threshold, 1, 255 );

    accel_hal_write( ADXL345_THRESH_TAP, 1, &tap_threshold );
}

// Gets the THRESH_TAP  char value
// return value is comprised between 0 and 255
// the scale factor is 62.5 mg/LSB
uint8_t adxl345_get_tap_threshold()
{
    uint8_t reading = 0;

    accel_hal_read( ADXL345_THRESH_TAP, 1, &reading );
    return reading;
}

// set/get the gain for each axis in Gs / count
void adxl345_set_axis_gains( double *_gains )
{
    int i = 0;
    if( _gains == NULL )
        return;
    for ( i = 0; i < 3; i++ )
        gains[i] = _gains[i];
}


void adxl345_get_axis_gains( double *_gains )
{
    int i = 0;
    if( _gains == NULL )
        return;
    for( i = 0; i < 3; i++ )
        gains[i] = _gains[i];
}

/* Sets the OFSX, OFSY and OFSZ  chars
   OFSX, OFSY and OFSZ are user offset adjustments in
   twos complement format with
   a scale factor of 15,6mg/LSB
   OFSX, OFSY and OFSZ should be comprised between */
void adxl345_set_axis_offset( int8_t x, int8_t y, int8_t z )
{
    accel_hal_write( ADXL345_OFSX, 1, &x );
    accel_hal_write( ADXL345_OFSY, 1, &y );
    accel_hal_write( ADXL345_OFSZ, 1, &z );
}

// Gets the OFSX, OFSY and OFSZ  chars
void adxl345_get_axis_offset( int8_t *x, int8_t *y, int8_t *z )
{
    uint8_t reading = 0;

    accel_hal_read( ADXL345_OFSX, 1, &reading );
    *x = reading;
    accel_hal_read( ADXL345_OFSY, 1, &reading );
    *y = reading;
    accel_hal_read( ADXL345_OFSZ, 1, &reading );
    *z = reading;
}

// Sets the DUR  char
// The DUR  char contains an unsigned time value representing the maximum time
// that an event must be above THRESH_TAP threshold to qualify as a tap event
// The scale factor is 625µs/LSB
// A value of 0 disables the tap/double tap funcitons. Max value is 255.
void adxl345_set_tap_duration( uint8_t tap_duration )
{
    accel_hal_write( ADXL345_DUR, 1, &tap_duration );
}

// Gets the DUR  char
uint8_t adxl345_get_tap_duration()
{
    uint8_t reading = 0;

    accel_hal_read( ADXL345_DUR, 1, &reading );
    return reading;
}

// Sets the latency (latent register) which contains an unsigned time value
// representing the wait time from the detection of a tap event to the start
// of the time window, during which a possible second tap can be detected.
// The scale factor is 1.25ms/LSB. A value of 0 disables the double tap function.
// It accepts a maximum value of 255.
void adxl345_set_double_tap_latency( uint8_t double_tap_latency )
{
    accel_hal_write( ADXL345_LATENT, 1, &double_tap_latency );
}

// Gets the Latent value
uint8_t adxl345_get_double_tap_latency()
{
    uint8_t reading = 0;

    accel_hal_read( ADXL345_LATENT, 1, &reading );

    return reading;
}

// Sets the Window register, which contains an unsigned time value representing
// the amount of time after the expiration of the latency time (Latent register)
// during which a second valud tap can begin. The scale factor is 1.25ms/LSB. A
// value of 0 disables the double tap function. The maximum value is 255.
void adxl345_set_double_tap_window( uint8_t double_tap_window )
{
    accel_hal_write( ADXL345_WINDOW, 1, &double_tap_window );
}

// Gets the Window register
uint8_t adxl345_get_double_tap_window()
{
    uint8_t reading = 0;

    accel_hal_read( ADXL345_WINDOW, 1, &reading );

    return reading;
}

// Sets the THRESH_ACT  char which holds the threshold value for detecting activity.
// The data format is unsigned, so the magnitude of the activity event is compared
// with the value is compared with the value in the THRESH_ACT register. The scale
// factor is 62.5mg/LSB. A value of 0 may result in undesirable behavior if the
// activity interrupt is enabled. The maximum value is 255.
void adxl345_set_activity_threshold( uint8_t activity_threshold )
{
    activity_threshold = CONSTRAIN( activity_threshold, 1, 255 );
    accel_hal_write( ADXL345_THRESH_ACT, 1, &activity_threshold );
}

// Gets the THRESH_ACT  char
uint8_t adxl345_get_activity_threshold()
{
    uint8_t reading = 0;

    accel_hal_read( ADXL345_THRESH_ACT, 1, &reading );

    return reading;
}

// Sets the THRESH_INACT  char which holds the threshold value for detecting inactivity.
// The data format is unsigned, so the magnitude of the inactivity event is compared
// with the value is compared with the value in the THRESH_INACT register. The scale
// factor is 62.5mg/LSB. A value of 0 may result in undesirable behavior if the
// inactivity interrupt is enabled. The maximum value is 255.
void adxl345_set_inactivity_threshold( uint8_t inactivity_threshold )
{
    inactivity_threshold = CONSTRAIN( inactivity_threshold, 1, 255 );
    accel_hal_write( ADXL345_THRESH_INACT, 1, &inactivity_threshold );
}

// Gets the THRESH_INACT  char
uint8_t adxl345_get_inactivity_threshold()
{
    uint8_t reading = 0;

    accel_hal_read( ADXL345_THRESH_INACT, 1, &reading );

    return reading;
}

// Sets the TIME_INACT register, which contains an unsigned time value representing the
// amount of time that acceleration must be less thant the value in the THRESH_INACT
// register for inactivity to be declared. The scale factor is 1sec/LSB. The value must
// be between 0 and 255.
void adxl345_set_inactivity_time( uint8_t time_inactivity )
{
    accel_hal_write( ADXL345_TIME_INACT, 1, &time_inactivity );
}

// Gets the TIME_INACT register
uint8_t adxl345_get_inactivity_time()
{
    uint8_t reading = 0;

    accel_hal_read( ADXL345_TIME_INACT, 1, &reading );

    return reading;
}

// Sets the THRESH_FF register which holds the threshold value, in an unsigned format, for
// free-fall detection. The root-sum-square (RSS) value of all axes is calculated and
// compared whith the value in THRESH_FF to determine if a free-fall event occured. The
// scale factor is 62.5mg/LSB. A value of 0 may result in undesirable behavior if the free-fall
// interrupt is enabled. The maximum value is 255.
void adxl345_set_free_fall_threshold( uint8_t free_fall_threshold )
{
    free_fall_threshold = CONSTRAIN( free_fall_threshold, 1, 255 );

    accel_hal_write( ADXL345_THRESH_FF, 1, &free_fall_threshold );
}

// Gets the THRESH_FF register.
uint8_t adxl345_get_free_fall_threshold()
{
    uint8_t reading = 0;

    accel_hal_read( ADXL345_THRESH_FF, 1, &reading );

    return reading;
}

// Sets the TIME_FF register, which holds an unsigned time value representing the minimum
// time that the RSS value of all axes must be less than THRESH_FF to generate a free-fall
// interrupt. The scale factor is 5ms/LSB. A value of 0 may result in undesirable behavior if
// the free-fall interrupt is enabled. The maximum value is 255.
void adxl345_set_free_fall_duration( uint8_t free_fall_duration )
{
    uint8_t reading;

    free_fall_duration = CONSTRAIN( free_fall_duration, 1, 255 );

    accel_hal_read( ADXL345_THRESH_FF, 1, &reading );

    if( reading < free_fall_duration )
        free_fall_duration = reading - 1;

    accel_hal_write( ADXL345_TIME_FF, 1, &free_fall_duration );
}

// Gets the TIME_FF register.
uint8_t adxl345_get_free_fall_duration()
{
    uint8_t reading = 0;

    accel_hal_read( ADXL345_TIME_FF, 1, &reading );

    return reading;
}

void adxl345_set_fifo_mode( adxl_fifo_mode_t mode )
{
    uint8_t reading = 0;
    mode = MIN( mode, ADXL345_FIFO_TRIGGER );

    accel_hal_read( ADXL345_FIFO_CTL, 1, &reading );
    reading &= ~( ADXL345_FIFO_CTL_FIFO_MODE );
    reading |= ( mode << 6 );
    accel_hal_write( ADXL345_FIFO_CTL, 1, &reading );
}

void adxl345_set_fifo_trigger( uint8_t int1_int2 )
{
    int1_int2 = MIN( int1_int2, 1 );

    set_register_bit( ADXL345_FIFO_CTL,
                      ADXL345_FIFO_CTL_TRIGGER_BIT,
                      int1_int2);
}

void adxl345_set_fifo_samples( uint8_t water_mark )
{
   uint8_t reading = 0;

   water_mark = MIN( water_mark, 27 );

   accel_hal_read( ADXL345_FIFO_CTL, 1, &reading );
   reading &= ~( ADXL345_FIFO_CTL_SAMPLES );
   reading |= water_mark;
   accel_hal_write( ADXL345_FIFO_CTL, 1, &reading );
}

uint8_t adxl345_get_fifo_count()
{
    uint8_t reading = 0;

    accel_hal_read( ADXL345_FIFO_STATUS, 1, &reading );
    reading &= ( 1 << 6 ) | ( 1 << 7 );

    return reading;
}

bool adxl345_is_fifo_triggered()
{
    return get_register_bit( ADXL345_FIFO_STATUS, 7 );
}

void adxl345_set_activity_x( bool state )
{
    set_register_bit( ADXL345_ACT_INACT_CTL,
                      ADXL345_ACT_INACT_CTL_ACT_X_BIT,
                      state );
}

void adxl345_set_activity_y( bool state )
{
    set_register_bit( ADXL345_ACT_INACT_CTL,
                      ADXL345_ACT_INACT_CTL_ACT_Y_BIT,
                      state );
}

void adxl345_set_activity_z( bool state )
{
    set_register_bit( ADXL345_ACT_INACT_CTL,
                      ADXL345_ACT_INACT_CTL_ACT_Z_BIT,
                      state );
}

void adxl345_set_inactivity_x( bool state )
{
    set_register_bit( ADXL345_ACT_INACT_CTL,
                      ADXL345_ACT_INACT_CTL_INACT_X_BIT,
                      state );
}

void adxl345_set_inactivity_y( bool state )
{
    set_register_bit( ADXL345_ACT_INACT_CTL,
                      ADXL345_ACT_INACT_CTL_INACT_Y_BIT,
                      state );
}

void adxl345_set_inactivity_z( bool state )
{
    set_register_bit( ADXL345_ACT_INACT_CTL,
                      ADXL345_ACT_INACT_CTL_INACT_Z_BIT,
                      state );
}

void adxl345_set_activity_ac( bool state )
{
    set_register_bit( ADXL345_ACT_INACT_CTL,
                      ADXL345_ACT_INACT_CTL_ACT_ACDC_BIT,
                      state );
}

void adxl345_set_inactivity_ac( bool state )
{
    set_register_bit( ADXL345_ACT_INACT_CTL,
                      ADXL345_ACT_INACT_CTL_INACT_ACDC_BIT,
                      state );
}

void adxl345_suppress_double_tap( bool state )
{
    set_register_bit( ADXL345_TAP_AXES,
                      ADXL345_TAP_AXES_SUPPRESS_BIT,
                      state );
}

void adxl345_set_tap_detection_on_x( bool state )
{
    set_register_bit( ADXL345_TAP_AXES,
                      ADXL345_TAP_AXES_TAP_X_BIT,
                      state );
}

void adxl345_set_tap_detection_on_y( bool state )
{
    set_register_bit( ADXL345_TAP_AXES,
                      ADXL345_TAP_AXES_TAP_Y_BIT,
                      state );
}

void adxl345_set_tap_detection_on_z( bool state )
{
    set_register_bit( ADXL345_TAP_AXES,
                      ADXL345_TAP_AXES_TAP_Z_BIT,
                      state );
}

bool adxl345_is_activity_source_on_y()
{
    return get_register_bit( ADXL345_ACT_TAP_STATUS,
                             ADXL345_ACT_TAP_STATUS_ACTY_BIT );
}

bool adxl345_is_activity_source_on_z()
{
    return get_register_bit( ADXL345_ACT_TAP_STATUS,
                             ADXL345_ACT_TAP_STATUS_ACTZ_BIT );
}

bool adxl345_is_tap_source_on_x()
{
    return get_register_bit( ADXL345_ACT_TAP_STATUS,
                             ADXL345_ACT_TAP_STATUS_TAPX_BIT );
}

bool adxl345_is_tap_source_on_y()
{
    return get_register_bit( ADXL345_ACT_TAP_STATUS,
                             ADXL345_ACT_TAP_STATUS_TAPY_BIT );
}

bool adxl345_is_tap_source_on_z()
{
    return get_register_bit( ADXL345_ACT_TAP_STATUS,
                             ADXL345_ACT_TAP_STATUS_TAPZ_BIT );
}

adxl345_rate_t adxl345_get_rate()
{
    uint8_t reading = 0;

    accel_hal_read( ADXL345_BW_RATE, 1, &reading );
    return ( reading &= 0x0f );
}

void adxl345_set_rate(adxl345_rate_t rate )
{
    uint8_t reading;
    rate = MIN( rate, ADXL345_RATE_1600HZ );

    accel_hal_read( ADXL345_BW_RATE, 1, &reading );
    reading &= ~0x0F;
    reading |= rate;
    accel_hal_write( ADXL345_BW_RATE, 1, &reading );
}

//Used to check if action was triggered in interrupts
//Example triggered(interrupts, ADXL345_SINGLE_TAP);
bool adxl345_triggered( adxl345_int_source_t interrupt )
{
    uint8_t reading = 0;
    
    interrupt = MIN( interrupt, ADXL345_INT_DATA_READY );
    
    accel_hal_read( ADXL345_INT_SOURCE, 1, &reading );
    
    return ( reading & ( 1 << interrupt ) )? true : false;
}






/*
 ADXL345_DATA_READY
 ADXL345_SINGLE_TAP
 ADXL345_DOUBLE_TAP
 ADXL345_ACTIVITY
 ADXL345_INACTIVITY
 ADXL345_FREE_FALL
 ADXL345_WATERMARK
 ADXL345_OVERRUNY
 */
adxl345_int_source_t adxl345_get_interrupt_source()
{
    uint8_t reading = 0;

    accel_hal_read( ADXL345_INT_SOURCE, 1, &reading );

    return reading;
}

// Set the mapping of an interrupt to pin1 or pin2
// eg: setInterruptMapping(ADXL345_INT_DOUBLE_TAP_BIT,ADXL345_INT2_PIN);
void adxl345_set_interrupt_mapping( adxl345_int_source_t interrupt,
                                    uint8_t pin )
{
    set_register_bit( ADXL345_INT_MAP, interrupt, ( bool )pin );
}

bool adxl345_is_interrupt_enabled( adxl345_int_source_t interrupt )
{
    return get_register_bit( ADXL345_INT_ENABLE, interrupt );
}

void adxl345_enable_interrupt( adxl345_int_source_t interrupt, bool state )
{
    set_register_bit( ADXL345_INT_ENABLE, interrupt, state );
}












// Gets the range setting and return it into rangeSetting
// it can be 2, 4, 8 or 16
adxl345_range_t adxl345_get_range_setting( void )
{
    uint8_t reading = 0;

    accel_hal_read( ADXL345_DATA_FORMAT, 1, &reading );
    reading &= 0x03;
    return reading;
}

// Sets the range setting, possible values are: 2, 4, 8, 16
void adxl345_set_range_setting( adxl345_range_t val )
{
    uint8_t reading = 0;

    val = MIN( val, ADXL345_RANGE_16G );

    accel_hal_read( ADXL345_DATA_FORMAT, 1, &reading );
    reading &= ~0xEC;
    reading |= val;
    accel_hal_write( ADXL345_DATA_FORMAT, 1, &reading );
}

// Sets the SELF-TEST bit
// if set to 1 it applies a self-test force to the sensor causing a shift in the output data
// if set to false it disables the self-test force
void adxl345_enable_self_test( bool self_test_bit )
{
    set_register_bit( ADXL345_DATA_FORMAT,
                      ADXL345_DATA_FORMAT_SELF_TEST_BIT,
                      self_test_bit );
}

// Sets the INT_INVERT bit
// if set to false sets the interrupts to active high
// if set to true sets the interrupts to active low
void adxl345_set_interrupt_level(bool interrupt_level_bit )
{
    set_register_bit( ADXL345_DATA_FORMAT,
                      ADXL345_DATA_FORMAT_INT_INVERT_BIT,
                      interrupt_level_bit );
}

/* Sets the FULL_RES bit
   if set to 1, the device is in full resolution mode, where the output
   resolution increases with the
   g range set by the range bits to maintain a 4mg/LSB scal factor
   if set to false, the device is in 10-bit mode, and the range buts determine the
   maximum g range
   and scale factor */
void adxl345_set_full_resolution( bool full_res )
{
    set_register_bit( ADXL345_DATA_FORMAT,
                      ADXL345_DATA_FORMAT_FULL_RES_BIT,
                      full_res );
}

// Gets the state of the justify bit
bool adxl345_get_justify_bit()
{
    return get_register_bit( ADXL345_DATA_FORMAT,
                             ADXL345_DATA_FORMAT_JUSTIFY_BIT );
}

// Sets the JUSTIFY bit
// if sets to true selects the left justified mode
// if sets to false selects right justified mode with sign extension
void adxl345_set_justify_bit(bool justify )
{
    set_register_bit( ADXL345_DATA_FORMAT,
                      ADXL345_DATA_FORMAT_JUSTIFY_BIT,
                      justify );
}

/*************** END OF FUNCTIONS ***************************************************************************/