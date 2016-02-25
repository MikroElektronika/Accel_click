/****************************************************************************
* Title                 :   Accel Click   
* Filename              :   adxl345.h
* Author                :   RBL
* Origin Date           :   20/08/2015
* Notes                 :   None
*****************************************************************************/
/**************************CHANGE LIST **************************************
*
*    Date    Software Version    Initials   Description 
*  20/02/16         .1           RBL        Interface Created.
*
*****************************************************************************/
/**
 * @file adxl345.h
 *
 * @brief Accelerometer ADXL345 Driver
 *
 * This driver was adapted from the arduino library for the same sensor.  It
 * supports both SPI and I2C bus communications as well as full featured
 * access to the registers.
 *
 * @author Richard Lowe
 * @copyright AlphaLoewe
 *
 * @date 16/11/2015
 *
 * @version 1.0 - Initial
 *
 * @note
 * <strong>Test configuration:</strong></br>
 *   <ul>
 *   <li>MCU:             ATMega32</li>
 *   <li>Dev.Board:       EasyAVR v7</li>
 *   <li>Oscillator:      8Mhz</li>
 *   <li>Ext. Modules:    x</li>
 *   <li>SW:              MikroC v6.0</li>
 *  </ul>
 *  </br>
 *  @par The ADXL345 operates in a 100 Hz ODR with a DATA_READY
 *  interrupt on the INT1 pin during this start-up sequence. When
 *  setting other interrupts or using the FIFO, it is recommended
 *  that those registers used are set before the POWER_CTL and
 *  INT_ENABLE registers
 *
 * @mainpage
 *  @section Documentation Quick Links
 *  <ul>
 *        <li>@subpage Intro
 *        <li>@subpage ADXL345_Registers
 *  </ul>
 *  @section Features
 *  <ul>
 *  <li>@par Ultralow power: as low as 23 µA in measurement mode and
 *  0.1 µA in standby mode at VS = 2.5 V (typical)</li>
 *  <li>@par Power consumption scales automatically with bandwidth</li>
 *  <li>@par User-selectable resolution</br>
 *        Fixed 10-bit resolution</br>
 *  Full resolution, where resolution increases with g range,
 *  up to 13-bit resolution at ±16 g (maintaining 4 mg/LSB
 *  scale factor in all g ranges)</li>
 *  <li>@par Patent pending, embedded memory management system
 *  with FIFO technology minimizes host processor load</li>
 *  <li>@par Single tap/double tap detection</li>
 *  <li>@par Activity/inactivity monitoring</li>
 *  <li>@par Free-fall detection</li>
 *  <li>@par Supply voltage range: 2.0 V to 3.6 V</li>
 *  <li>@par I/O voltage range: 1.7 V to VS</li>
 *  <li>@par SPI (3- and 4-wire) and I2C digital interfaces</li>
 *  <li>@par Flexible interrupt modes mappable to either interrupt pin</li>
 *  <li>@par Measurement ranges selectable via serial command</li>
 *  <li>@par Bandwidth selectable via serial command</li>
 *  <li>@par Wide temperature range (−40°C to +85°C)</li>
 *  <li>@par 10,000 g shock survival</li>
 *  </ul>
 */
#ifndef ADXL345_H
#define ADXL345_H

/******************************************************************************
* Includes
*******************************************************************************/
#include <stdbool.h>
#include <stdint.h>

/******************************************************************************
* Preprocessor Constants
*******************************************************************************/
/**
 * @defgroup Required_PINs
 * @{
 */
#define ADXL345_INT1_PIN 0x00 /**< Enable INT1 pin */
#define ADXL345_INT2_PIN 0x01 /**< Enable INT2 pin */
/**@}*/

#define ADXL345_OK          0  // no error
#define ADXL345_ERROR      -1   // indicates error is predent

/**
 * @defgroup Slave_Addresses
 * @{
 */
#define ADXL345_ADDR        0x1D
#define ADXL345_ALT_ADDR    0x53
/**@}*/
/******************************************************************************
* Configuration Constants
*******************************************************************************/


/******************************************************************************
* Macros
*******************************************************************************/


    
/******************************************************************************
* Typedefs
*******************************************************************************/
/**
 * @defgroup Measurement_Range
 * @{
 */
typedef enum
{
   ADXL345_RANGE_2G = 0,
   ADXL345_RANGE_4G,
   ADXL345_RANGE_8G,
   ADXL345_RANGE_16G
} adxl345_range_t;
/**@}*/

/**
 *  @defgroup Interrupt_Sources
 *  @{
 */
typedef enum
{
    ADXL345_INT_OVERRUN = 0,
    ADXL345_INT_WATERMARK,
    ADXL345_INT_FREE_FALL,
    ADXL345_INT_INACTIVITY,
    ADXL345_INT_ACTIVITY,
    ADXL345_INT_DOUBLE_TAP,
    ADXL345_INT_SINGLE_TAP,
    ADXL345_INT_DATA_READY
} adxl345_int_source_t;
/**@}*/

/**
 * @defgroup Bandwidth
 * @{
 */
typedef enum
{
    ADXL345_RATE_0_05HZ = 0,
    ADXL345_RATE_0_10HZ,
    ADXL345_RATE_0_20HZ,
    ADXL345_RATE_0_39HZ,
    ADXL345_RATE_0_78HZ,
    ADXL345_RATE_1_56HZ,
    ADXL345_RATE_3_13HZ,
    ADXL345_RATE_6_25HZ,
    ADXL345_RATE_12_5HZ,
    ADXL345_RATE_25HZ,
    ADXL345_RATE_50HZ,
    ADXL345_RATE_100HZ,
    ADXL345_RATE_200HZ,
    ADXL345_RATE_400HZ,
    ADXL345_RATE_800HZ,
    ADXL345_RATE_1600HZ
} adxl345_rate_t;
/**@}*/

/**
 * @defgroup Sleep_Read_Rate
 * @{
 */
typedef enum
{
    ADXL345_SLEEP_8HZ = 0,
    ADXL345_SLEEP_4HZ,
    ADXL345_SLEEP_2HZ,
    ADXL345_SLEEP_1HZ
} adxl345_reading_rate_t;
/**@}*/

/**
 * @defgroup FIFO_Buffer
 * @{
 */

/**
 * @enum adxl_fifo_mode_t
 */
typedef enum
{
    ADXL345_FIFO_BYPASSED, /**< Default - FIFO is bypassed. */
    ADXL345_FIFO_ENABLE,   /**< FIFO collects up to 32 values and then stops
                                collecting data, collecting new data only when
                                FIFO is not full. */
    ADXL345_FIFO_STREAM,   /**< FIFO holds the last 32 data values. When FIFO
                                is full, the oldest data is overwritten with
                                newer data. */
    ADXL345_FIFO_TRIGGER   /**< When triggered by the trigger bit, FIFO holds
                                the last data samples before the trigger event
                                and then continues to collect data until full.
                                New data is collected onlywhen FIFO is not
                                full. */
} adxl_fifo_mode_t;
/**@}*/

/**
 * @enum Modes of operation
 *
 * With the ALT ADDRESS pin high, the 7-bit I2C address for the device is
 * 0x1D, followed by the R/W bit. This translates to 0x3A for a write and 0x3B
 * for a read. An alternate I2C address of 0x53 (followed by the R/W bit)
 * can be chosen by grounding the ALT ADDRESS pin (Pin 12).
 * This translates to 0xA6 for a write and 0xA7 for a read.
 */
typedef enum
{
    ADXL345_MODE_I2C = 0,
    ADXL345_MODE_SPI4,
    ADXL345_MODE_SPI3,
} adxl345_mode_t;

/******************************************************************************
* Variables
*******************************************************************************/


/******************************************************************************
* Function Prototypes
*******************************************************************************/
#ifdef __cplusplus
extern "C"{
#endif

/**
 *  @defgroup General_Use_Functions
 *  @{
 */

/**
 *  @ingroup General_Use_Functions
 *  @brief Initializes the Sensor
 *
 *  @pre TWI_Init or SPI_Init needs to be called before.
 *
 */
int adxl345_init( adxl345_mode_t mode, uint8_t address );

/**
 * @brief adx1345_wakeup
 * @param reads_while_sleep
 */
void adx1345_wakeup( adxl345_reading_rate_t reads_while_sleep );

/**
 * @brief adxl345_enable_sleep
 * @param sleep
 */
void adxl345_enable_sleep( bool sleep );

/**
 * @brief adxl345_enable_measure
 * @param measure
 */
void adxl345_enable_measure( bool measure );

/**
 * @brief adx1345_enable_autosleep
 * @param autosleep
 */
void adx1345_enable_autosleep( bool autosleep );

/**
 *  @brief Reads the acceleration into an array of 3 ints
 *
 *  @param[out] xyz - array of 3 int variables
 */
void adxl345_read_accelxyz( int16_t *xyx );

/**
 *  @brief Reads the acceleration into seperate int variables
 *
 *  @param[out] x,y,z Values of sensor axis
 *
 */
void adxl345_read_accel( int16_t *x, int16_t *y, int16_t *z );

/**
 *  @brief Gets the G forces on axis x,y,z into array of double
 *
 *  @param[out] xyz - array of 3 double variables
 *
 */
void adxl345_read_g_xyz( double *xyz );

/**@}*/

/**
 *  @defgroup Behavior_Settings
 *        @{
 */
/**
 *  @brief Set the tap threshold
 *  @ref THRESH_TAP
 *  @param[in] tapThreshold - values from 0 - 255
 */
void adxl345_set_tap_threshold( uint8_t tap_threshold );

/**
 *  @brief Get the tap threshold
 *  @ref THRESH_TAP
 *  @returns int
 *      @retval 0 - 255
 */
uint8_t adxl345_get_tap_threshold( void );

/**
 *  @brief Set the axis gain
 *  @param[in,out] _gains -
 */
void adxl345_set_axis_gains( double *gains );

/**
 *  @brief Get the axis gain
 *  @param[out] _gains
 */
void adxl345_get_axis_gains( double *gains );

/**
 *  @brief Set axis offset
 *  @param[in] x,y,z - Offset of axis x, y, and z
 */
void adxl345_set_axis_offset( int8_t x, int8_t y, int8_t z );

/**
 *  @brief Get axis offset
 *  @param[out] x,y,z - axis offsets
 */
void adxl345_get_axis_offset( int8_t *x, int8_t *y, int8_t *z );

/**

 *  @brief Set tap duration
 *  @param[in] int tapDuration
 *  @parblock
 *  Sets the DUR(duration)
 *        The DUR contains an unsigned time value representing the maximum time
 *  that an event must be above THRESH_TAP threshold to qualify as a tap event
 *  The scale factor is 625µs/LSB
 *        A value of 0 disables the tap/double tap funcitons. Max value is 255.
 *  @endparblock
 */
void adxl345_set_tap_duration( uint8_t tap_duration );

/**
 *  @brief Get tap duration
 *
 *  @returns int
 *  @retval false - - 255
 */
uint8_t adxl345_get_tap_duration( void );
/**
 * @brief adxl345_set_double_tap_latency
 * @param double_tap_latency
 */
void adxl345_set_double_tap_latency( uint8_t double_tap_latency );

/**
 * @brief adxl345_get_double_tap_latency
 * @return
 */
uint8_t adxl345_get_double_tap_latency( void );

/**
 * @brief adxl345_set_double_tap_window
 * @param double_tap_window
 */
void adxl345_set_double_tap_window( uint8_t double_tap_window );

/**
 * @brief adxl345_get_double_tap_window
 * @return
 */
uint8_t adxl345_get_double_tap_window( void );

/**
 * @brief adxl345_set_activity_threshold
 * @param activity_threshold
 */
void adxl345_set_activity_threshold( uint8_t activity_threshold );

/**
 * @brief adxl345_get_activity_threshold
 * @return
 */
uint8_t adxl345_get_activity_threshold( void );

/**
 * @brief adxl345_set_inactivity_threshold
 * @param inactivity_threshold
 */
void adxl345_set_inactivity_threshold( uint8_t inactivity_threshold );

/**
 * @brief adxl345_get_inactivity_threshold
 * @return
 */
uint8_t adxl345_get_inactivity_threshold( void );

/**
 * @brief adxl345_set_inactivity_time
 * @param time_inactivity
 */
void adxl345_set_inactivity_time( uint8_t time_inactivity );

/**
 * @brief adxl345_get_inactivity_time
 * @return
 */
uint8_t adxl345_get_inactivity_time( void );

/**
 *  @brief Sets the freefall threshold
 *
 *  @param[in] int freeFallThreshold
 *  @parblock
 *  Sets the TIME_FF register, which holds an unsigned time value representing
 *  the minimum time that the RSS value of all axes must be less than
 *  THRESH_FF to generate a free-fall interrupt. The scale factor is 5ms/LSB.
 *  The maximum value is 255.
 *  @endparblock
 *
 *  @note
 *  A value of 0 may result in undesirable behavior if the free-fall interrupt is enabled.
 */
void adxl345_set_free_fall_threshold( uint8_t free_fall_threshold );

/**
 *  @brief Get freefall threshold
 *
 *  @returns uint8_t
 *  @retval Value 0 - 255
 *
 */
uint8_t adxl345_get_free_fall_threshold( void );

/**
 *  @brief Sets the TIME_FF register
 *
 *  @param[in] int freeFallDuration - The scale factor is 5ms/LSB. A value of 0 may result in undesirable behavior if
 *  the free-fall interrupt is enabled. The maximum value is 255.
 *
 *  @note
 *  Time that the RSS value of all axes must be less than THRESH_FF to generate a free-fall interrupt.
 *
 */
void adxl345_set_free_fall_duration( uint8_t free_fall_duration );

/**
 *  @brief Gets the current freefall duration
 *
 *  @returns uint8_t
 *  @retval 0-255 rate is 5ms*value
 */
uint8_t adxl345_get_free_fall_duration( void );

/**
 * @ingroup FIFO_Buffer
 * @brief Sets mode of the 32 level fifo buffer
 * @param mode
 */
void adxl345_set_fifo_mode( adxl_fifo_mode_t mode );

void adxl345_set_fifo_trigger( uint8_t int1_int2 );

/**
 * @brief adxl345_set_fifo_samples
 * @param water_mark - fifo count when waterlevel will trigger event max 27
 *
 * @note Undesirable operation may occur if a value of 0 is used for the
 * samples bits when trigger mode is used.
 */
void adxl345_set_fifo_samples( uint8_t water_mark );

/**
 * @brief adxl345_get_fifo_count
 * @return
 */
uint8_t adxl345_get_fifo_count( void );

/**
 * @brief adxl345_is_fifo_triggered
 * @return
 */
bool adxl345_is_fifo_triggered( void );

/**@}*/

/**
 * @defgroup Registry_Functions
 * @{
 */
void adxl345_set_activity_ac( bool state );
void adxl345_set_inactivity_ac( bool state );
void adxl345_suppress_double_tap( bool state );
void adxl345_set_tap_detection_on_x( bool state );
void adxl345_set_tap_detection_on_y( bool state );
void adxl345_set_tap_detection_on_z( bool state );

void adxl345_set_activity_x( bool state );
void adxl345_set_activity_y( bool state );
void adxl345_set_activity_z( bool state );
void adxl345_set_inactivity_x( bool state );
void adxl345_set_inactivity_y( bool state );
void adxl345_set_inactivity_z( bool state );

bool adxl345_is_activity_source_on_x( void );
bool adxl345_is_activity_source_on_y( void );
bool adxl345_is_activity_source_on_z( void );
bool adxl345_is_tap_source_on_x( void );
bool adxl345_is_tap_source_on_y( void );
bool adxl345_is_tap_source_on_z( void );

adxl345_rate_t adxl345_get_rate( void );
void adxl345_set_rate( adxl345_rate_t rate );

/**
 *  @brief Used to check if action was triggered in interrupts
 *
 *  @param[in] interrupts -
 *  @param[in] mask -
 *
 *  @returns bool
 *  @retval true - interrupt triggered
 *  @retval false - interrupt not triggered
 *
 *  @code
 *  if( triggered( ADXL345_INT_SINGLE_TAP ) )
 *     printf( "Triggered" );

 *  @endcode
 *
 */
bool adxl345_triggered( adxl345_int_source_t interrupts );

/**
 *  @brief Gets the source of interrupt triggered
 *
 *  @returns char
 *  @retval ADXL345_DATA_READY
 *  @retval ADXL345_SINGLE_TAP
 *  @retval ADXL345_DOUBLE_TAP
 *  @retval ADXL345_ACTIVITY
 *  @retval ADXL345_INACTIVITY
 *  @retval ADXL345_FREE_FALL
 *  @retval ADXL345_WATERMARK
 *  @retval ADXL345_OVERRUNY
 */
adxl345_int_source_t adxl345_get_interrupt_source( void );

/**
 *  @brief Set the mapping of an interrupt to pin1 or pin2
 *
 *  @param[in] interruptBit
 *  @param[in] interruptPin
 *
 *  @code
 *  setInterruptMapping(ADXL345_INT_DOUBLE_TAP_BIT,ADXL345_INT2_PIN);
 *  @endcode
 *
 */
void adxl345_set_interrupt_mapping( adxl345_int_source_t interrupt,
                                    uint8_t pin );

/**
 * @brief adxl345_isInterruptEnabled
 * @param interruptBit
 * @return
 */
bool adxl345_is_interrupt_enabled( adxl345_int_source_t interrupt );

/**
 * @brief adxl345_setInterrupt
 * @param interruptBit
 * @param state
 */
void adxl345_enable_interrupt( adxl345_int_source_t interrupt, bool state );


/**
 *  @brief Gets the range setting and return it into rangeSetting
 *

 *  @param[in,out] rangeSetting
 *
 */
adxl345_range_t adxl345_get_range_setting( void );

/**
 *  @brief Sets the range setting
 *
 *  @param[in] int val - , possible values are: 2, 4, 8, 16
 *
 */
void adxl345_set_range_setting( adxl345_range_t range );

/**
 *  @brief Sets the SELF-TEST bit
 *
 *  @param[in] bool -
 *  true - applies a self-test force to the sensor causing a shift in the
 *  output data
 *  false - disables the self-test force
 *
 */
void adxl345_enable_self_test(bool self_test_bit );

/**
 *  @brief Sets the INT_INVERT bit
 *
 *  @param[in] bool interruptLevelBit - false sets the interrupts to active
 *   high, true sets the interrupts to active low
 *
 */
void adxl345_set_interrupt_level(bool interrupt_level_bit );

/**
 *  @brief Sets the FULL_RES bit
 *
 *  @param[in] bool fullResBit
 *  @parblock
 *  true = the device is in full resolution mode, where the output resolution increases with the
 *  g range set by the range bits to maintain a 4mg/LSB scal factor
 *  @endparblock
 *  false = the device is in 10-bit mode, range bits determine the maximum g range
 *  and scale factor
 */
void adxl345_set_full_resolution(bool full_res );

/**
 *  @brief Gets the state of the justify bit
 *
 *  @returns bool
 *          @retval 1 left justified mode
 *  @retval false - right justified with sign
 */
bool adxl345_get_justify_bit( void );

/**
 *  @brief Sets the JUSTIFY bit
 *
 *  @param[in] bool justifyBit
 *  @parblock
 *  1 = left justified mode
 *        0 = right justified mode with sign extension
 *  @endparblock
 */
void adxl345_set_justify_bit( bool justify );

/**@}*/

#ifdef __cplusplus
} // extern "C"
#endif

#endif /*File_H_*/

/*** End of File **************************************************************/
/**
 *  @page ADXL345_Registers
 *  @section Register Descriptions
 *        Threshold Tap reference @ref THRESS_TAP "Threshold Tap"
 *
 */

/**
   @page Intro

        @par Introduction to the ADXL345
        @par
        A small, thin, ultralow power, 3-axis accelerometer with high resolution (13-bit) measurement at up to ±16 g. Digital output
        data is formatted as 16-bit twos complement and is accessible through either a SPI (3- or 4-wire) or I 2 C digital interface.
        @par
        The ADXL345 is well suited for mobile device applications. It measures the static acceleration of gravity in tilt-sensing applications, as well as
        dynamic acceleration resulting from motion or shock. Its high resolution (3.9 mg/LSB) enables measurement
         of inclination changes less than 1.0°.
        @par
        Several special sensing functions are provided. Activity and inactivity sensing detect the presence or lack of motion by comparing the acceleration
        on any axis with user-set thresholds. Tap sensing detects single and double taps in any direction. Free
        fall sensing detects if the device is falling. These functions can be mapped individually to either of
        two interrupt output pins.  An integrated, patent pending memory management system with a 32-level first in, first out (FIFO) buffer can be used to store data to minimize host
        processor activity and lower overall system power consumption.
        @par
        Low power modes enable intelligent motion-based power
         management with threshold sensing and active acceleration measurement at extremely low power
        dissipation.
*/


/**
         @page THRESH_TAP

           @par The THRESH_TAP register
           is eight bits and holds the threshold
         value for tap interrupts. The data format is unsigned, therefore,
         the magnitude of the tap event is compared with the value
         in THRESH_TAP for normal tap detection. The scale factor is
         62.5 mg/LSB (that is, 0xFF = 16 g). A value of 0 may result in
         undesirable behavior if single tap/double tap interrupts are
         enabled.
 */

/**
   @page OFFSET

   @par The OFSX, OFSY, and OFSZ registers
   are each eight bits and offer user-set offset adjustments in twos complement format
with a scale factor of 15.6 mg/LSB (that is, 0x7F = 2 g). The
value stored in the offset registers is automatically added to the
acceleration data, and the resulting value is stored in the output
data registers. For additional information regarding offset
calibration and the use of the offset registers, refer to the Offset
Calibration section.
*/

/**
          @page DUR

          @par The DUR register
          is eight bits and contains an unsigned time
          value representing the maximum time that an event must be
        above the THRESH_TAP threshold to qualify as a tap event. The
        scale factor is 625 µs/LSB. A value of 0 disables the single tap/
        double tap functions.
*/

/**
        @page LATENT

        @par The latent register
        is eight bits and contains an unsigned time
        value representing the wait time from the detection of a tap
        event to the start of the time window (defined by the window
        register) during which a possible second tap event can be detected.
        The scale factor is 1.25 ms/LSB. A value of 0 disables the double tap
        function.
*/