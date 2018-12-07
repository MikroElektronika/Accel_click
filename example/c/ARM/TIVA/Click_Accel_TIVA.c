/*
Example for Accel Click

    Date          : Aug 2018.
    Author        : Nenad Filipovic

Test configuration TIVA :
    
    MCU              : TM4C129XNCZAD
    Dev. Board       : EasyMx PRO v7 for TIVA ARM
    ARM Compiler ver : v6.0.0.0

---

Description :

The application is composed of three sections :

- System Initialization - Initializes I2C.
- Application Initialization - Initialization driver enable's - I2C. Check sensor ID and initialize Accel click.
- Application Task - (code snippet) This is a example which demonstrates the use of Accel click board.
     Measured coordinates (X,Y,Z) are being sent to the UART where you can track their changes.

Additional Functions :

- UART
- Conversions

*/

#include "Click_Accel_types.h"
#include "Click_Accel_config.h"


void systemInit()
{
    mikrobus_i2cInit( _MIKROBUS1, &_ACCEL_I2C_CFG[0] );
    mikrobus_logInit( _MIKROBUS2, 9600 );
    Delay_100ms();
}

void applicationInit()
{
    accel_i2cDriverInit( (T_ACCEL_P)&_MIKROBUS1_GPIO, (T_ACCEL_P)&_MIKROBUS1_I2C, _ACCEL_I2C_ADDRESS_0 );
    Delay_100ms();

    if ( accel_checkId() == 0 )
    {
        accel_Init();
        mikrobus_logWrite(" Initialisation",_LOG_LINE);
        mikrobus_logWrite("----------------", _LOG_LINE);
    }
    else
    {
        mikrobus_logWrite("     ERROR",_LOG_LINE);
        mikrobus_logWrite("----------------", _LOG_LINE);
    }
    Delay_100ms();
}

void applicationTask()
{
    int16_t valueX;
    int16_t valueY;
    int16_t valueZ;
    uint8_t txtX[ 15 ];
    uint8_t txtY[ 15 ];
    uint8_t txtZ[ 15 ];

    valueX = accel_readXaxis();
    valueY = accel_readYaxis();
    valueZ = accel_readZaxis();

    IntToStr( valueX, txtX );
    IntToStr( valueY, txtY );
    IntToStr( valueZ, txtZ );

    mikrobus_logWrite( " Axis X :", _LOG_TEXT );
    mikrobus_logWrite( txtX, _LOG_LINE );

    mikrobus_logWrite( " Axis Y :", _LOG_TEXT );
    mikrobus_logWrite( txtY, _LOG_LINE );

    mikrobus_logWrite( " Axis Z :", _LOG_TEXT );
    mikrobus_logWrite( txtZ, _LOG_LINE );

    mikrobus_logWrite("----------------", _LOG_LINE);

    Delay_ms( 5000 );
}

void main()
{
    systemInit();
    applicationInit();

    while (1)
    {
            applicationTask();
    }
}