#include "FEHPropeller.h"
#include "FEHProteus.h"
#include "derivative.h"
#include "uart.h"
#include "mcg.h"
#include "FEHUtility.h"

FEHPropeller::FEHPropeller()
{
    _initialized = false;
}

bool FEHPropeller::Initialize()
{
    if( !_initialized )
    {
        // Init uart clock for propeller connection
        SIM_SCGC1 = SIM_SCGC1_UART5_MASK;

        // Propeller UART:
        // Signal PROP_TX - UART5_TX - PTE8 - Pin 11
        // Signal PROP_RX - UART5_RX - PTE9 - Pin 12
        // UART function on Alt3
        PORTE_PCR8 = PORT_PCR_MUX( 0x03 );
        PORTE_PCR9 = PORT_PCR_MUX( 0x03 );

        // Initialize propeller UART
        uart_init( UART5_BASE_PTR, CoreClockKHz / 2, 57600 );

        // wait for UART to init
        Sleep( 100 );

        // reset Propeller
        //this->Reset();

        _initialized = true;

        return true;
    }
    return false;
}

bool FEHPropeller::IsInitialized()
{
    return _initialized;
}

void FEHPropeller::Reset()
{
    uart_putchar( UART5_BASE_PTR, 0x7F );
    uart_putchar( UART5_BASE_PTR, 0x01 ); // soft reset
    uart_putchar( UART5_BASE_PTR, 0xFF );
}

void FEHPropeller::SetMotorRate( uint8 motor, uint8 speed, uint8 rate )
{
    uart_putchar( UART5_BASE_PTR, 0x7F );
    uart_putchar( UART5_BASE_PTR, 0x08 ); // motor set rate
    uart_putchar( UART5_BASE_PTR, motor ); // motor to set
    uart_putchar( UART5_BASE_PTR, speed ); // speed
    uart_putchar( UART5_BASE_PTR, rate ); // rate
    uart_putchar( UART5_BASE_PTR, 0xFF );
}
