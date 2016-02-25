![mikroe_logo] 
# by [MikroElektronika]
![accel_image] 
# More information about [Accel click] is found here.
---
## Installation
>If installing from package, [Package manager] required to install to your IDE.  

## Looking for a [tutorial?][Accel Tutorial]

### Example
```
void system_init( )
{
    ...
    
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
}


void main() 
{
    int16_t avg_accel[3];
    adxl345_int_source_tsource = 0;
    double avg_g_s[3];
    char tmp_text[80];
    int i;
    
    system_init();
    
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

        ....
        Delay_ms( 2500 );
    }
}

```

[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)

   [MikroElektronika]: <http://www.mikroe.com/>
   [mikroe_logo]: <http://www.mikroe.com/img/designs/beta/logo_small.png>
   [Accel click]: <http://www.mikroe.com/click/accel/>
   [accel_image]: <http://www.mikroe.com/img/development-tools/accessory-boards/click/accel/accel_click_main.png>
   [Accel Tutorial]: <http://learn.mikroe.com/>
   [Package Manager]: <http://www.mikroe.com/package-manager/>