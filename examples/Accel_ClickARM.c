/*******************************************************************************
* Title                 :   Accel click Example
* Filename              :   Acccel_ClickARM.c
* Author                :   RBL
* Origin Date           :   20/08/2015
* Notes                 :   None
*******************************************************************************/
/*************** MODULE REVISION LOG ******************************************
*
*    Date    Software Version    Initials   Description 
*  20/02/16         .1           RBL      Module Created.
*
*******************************************************************************/
/** 
 *  @file Accel_ClickARM.c
 *  @brief Example of multiple features of the Accel click 
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include "adxl345.h"
#include "built_in.h"
#include <stdbool.h>

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/

/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
int16_t volatile test_accel[128];
bool volatile go_flag;

/* The CS pin should always be tied high to V DD I/O or be driven by an 
   external controller because there is no default mode if the CS pin is left 
   unconnected. Therefore, not taking these precautions may result in an 
   inability to communicate with the part. */
sbit ACCEL_CS at GPIOD_ODR.B13;

/******************************************************************************
* Function Prototypes
*******************************************************************************/
void system_init( void );

/******************************************************************************
* Function Definitions
*******************************************************************************/
void system_init( )
{
    DisableInterrupts();
    
    /* APB1 Peripheral clock is Mhz.  Max is 5 Mhz on radio */
    SPI3_Init_Advanced( _SPI_FPCLK_DIV64,
                        _SPI_MASTER | _SPI_8_BIT | _SPI_CLK_IDLE_HIGH |
                        _SPI_FIRST_CLK_EDGE_TRANSITION | _SPI_MSB_FIRST |
                        _SPI_SS_DISABLE | _SPI_SSM_ENABLE | _SPI_SSI_1,
                        &_GPIO_MODULE_SPI3_PC10_11_12 );

    I2C1_Init_Advanced( 100000, &_GPIO_MODULE_I2C1_PB67 );

    GPIO_Digital_Output( &GPIOD_BASE, _GPIO_PINMASK_13 );
    GPIO_Digital_Input( &GPIOD_BASE, _GPIO_PINMASK_10 );

    RCC_APB2ENR |= ( 1 << SYSCFGEN );  /* Enable clock for alternate pin functions */
    SYSCFG_EXTICR3 |= ( 1 << EXTI100 ) | ( 1 << EXTI101 ); /* Map external interrupt on PD10 */
    EXTI_FTSR |= ( 1 << TR10 );  /* Falling event for pin PD10 */
    //EXTI_RTSR |= ( 1 << TR10 );  /* Rising edge for pin PD10 */
    EXTI_IMR |= ( 1 << MR10 );       /* Mask pin PB1 for interrupt */
    //EXTI_EMR |= ( 1 << MR10 );    /* Event mask on pin PD10 */
    NVIC_IntEnable( IVT_INT_EXTI15_10 );   /* Radio interrupt */
    
    UART1_Init_Advanced( 57600, _UART_8_BIT_DATA, _UART_NOPARITY,
                         _UART_ONE_STOPBIT, &_GPIO_MODULE_USART1_PA9_10 );
    Delay_ms( 100 );
    UART_Write_Text( "System Start\r\n" );
    
    if( adxl345_init( ADXL345_MODE_SPI4, ADXL345_ADDR ) )
    {
        UART_Write_Text( "Error\r\n" );
        while( 1 );
    }
    
    adxl345_set_fifo_samples( 16 );
    adxl345_set_fifo_mode( ADXL345_FIFO_STREAM );
    adxl345_set_interrupt_level( true ); // Set interrupts to active low
    adxl345_set_fifo_mode( ADXL345_FIFO_ENABLE );

    //set activity/ inactivity thresholds (0-255)
    adxl345_set_activity_threshold( 75 );   //62.5mg per increment
    adxl345_set_inactivity_threshold( 75 ); //62.5mg per increment
    adxl345_set_inactivity_time( 10 );      // how many seconds of no activity is inactive?

    //look of activity movement on this axes true == on; false == off
    adxl345_set_activity_x( true );
    adxl345_set_activity_y( true );
    adxl345_set_activity_z( true );

    //look of tap movement on this axes true == on; false == off
    adxl345_set_tap_detection_on_x( false );
    adxl345_set_tap_detection_on_y( false );
    adxl345_set_tap_detection_on_z( true );

    //set values for what is a tap, and what is a double tap (0-255)
    adxl345_set_tap_threshold( 10 );     //62.5mg per increment
    adxl345_set_tap_duration( 15 );      //625\u03bcs per increment
    adxl345_set_double_tap_latency( 80 ); //1.25ms per increment
    adxl345_set_double_tap_window( 200 ); //1.25ms per increment

    //set values for what is considered freefall (0-255)
    adxl345_set_free_fall_threshold( 7 ); //(5 - 9) recommended - 62.5mg per increment
    adxl345_set_free_fall_duration( 15 ); //(20 - 70) recommended - 5ms per increment

    //register interupt actions true == on; false == off
    adxl345_enable_interrupt( ADXL345_INT_SINGLE_TAP, true );
    adxl345_enable_interrupt( ADXL345_INT_DOUBLE_TAP, true );
    adxl345_enable_interrupt( ADXL345_INT_FREE_FALL, true );
    adxl345_enable_interrupt( ADXL345_INT_ACTIVITY, true );
    adxl345_enable_interrupt( ADXL345_INT_INACTIVITY, true );
    adxl345_enable_interrupt( ADXL345_INT_DATA_READY, true );
    adxl345_enable_interrupt( ADXL345_INT_WATERMARK, true );
    
    adxl345_enable_measure( true );
    UART_Write_Text( "Ready\r\n" );
}


void main() 
{
    int16_t avg_accel[3];
    static adxl345_int_source_t volatile source = 0;
    double avg_g_s[3];
    char tmp_text[80];
    int i;
    
    system_init();
    EnableInterrupts();

    while( 1 )
    {
        int count = adxl345_get_fifo_count();
        int16_t *ptr = test_accel;

        for( i = 0; i < count; i++ )
        {
            adxl345_read_accelxyz( ptr );
            ptr += 3;
        }
        
        if( go_flag )
        {
            for( i = 0; i < 128 / 3; i++ )
            {
                sprinti( tmp_text, "X:%d Y:%d Z:%d\r\n", 
                         test_accel[i], test_accel[i+1], test_accel[i+2] );
                UART_Write_Text( tmp_text );
            }
            go_flag = false;
        } 

        for( i = 0; i < 10; i++ )
        {
            int16_t accel[3];
            double g_s[3];
            
            adxl345_read_accelxyz( accel );
            adxl345_read_g_xyz( g_s );
            
            avg_accel[0] += accel[0];
            avg_accel[1] += accel[1];
            avg_accel[2] += accel[2];
            
            avg_g_s[0] += g_s[0];
            avg_g_s[1] += g_s[1];
            avg_g_s[2] += g_s[2];
        }
        
        for( i = 0; i < 3; i++ )
        {
             avg_g_s[i] /= 10;
             avg_accel[i] /= 10;
        }
        
        source = adxl345_get_interrupt_source();

        if( source > 2 )
        {
            sprinti( tmp_text, "Interrupt Source is: %d\r\n", source );
            UART_Write_Text( tmp_text );
        }
        
        sprinti( tmp_text, "X:%d Y:%d Z:%d\r\n", avg_accel[0], avg_accel[1], avg_accel[2] );
        UART_Write_Text( tmp_text );
        sprintf( tmp_text, "XGs:%f YGs:%f Z:Gs%f\r\n", avg_g_s[0], avg_g_s[1], avg_g_s[2] );
        UART_Write_Text( tmp_text );


        Delay_ms( 2500 );
    }
}

void accel_ISR() iv IVT_INT_EXTI15_10 ics ICS_AUTO
{
    EXTI_PR |= ( 1 << PR10 ); // Clear interrupt flag
    
    UART_Write_Text( "Interrupt\r\n" );
    
    if( adxl345_triggered( ADXL345_INT_WATERMARK ) )
    {
        int i;
        int count = adxl345_get_fifo_count();
        int16_t *ptr = test_accel;
        
        for( i = 0; i < count; i++ )
        {
            adxl345_read_accelxyz( ptr );
            ptr += 3;
        }
        
        go_flag = true;
    }
    
    if( adxl345_triggered( ADXL345_INT_SINGLE_TAP ) )
            UART_Write_Text( "Triggered\r\n" );
}

/*************** END OF FUNCTIONS ***************************************************************************/
