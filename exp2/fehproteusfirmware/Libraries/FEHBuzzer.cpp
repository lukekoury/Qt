#include "derivative.h"
#include "FEHUtility.h"
#include "FEHBuzzer.h"
#include "uart.h"

#define default_frequency 1000
#define default_duration 500

FEHBuzzer Buzzer;

void FEHBuzzer::Beep()//default beep at default frequency and default duration
{
    Tone( default_frequency, default_duration );
}

void FEHBuzzer::Buzz()//buzz at default frequency and infinite duration
{
    Tone(default_frequency,0);
}

void FEHBuzzer::Buzz( double duration )//buzz at default freuency and user defined float duration (seconds)
{
    duration = duration * 1000;
    Tone( default_frequency, ( int ) duration );
}

void FEHBuzzer::Buzz( int duration )//buzz at default frequency and user defined integer duration (milliseconds)
{
    Tone( default_frequency, duration );
}

void FEHBuzzer::Tone( int frequency )//tone at user defined frequency and infinite duration
{
    Tone( frequency, 0 );
}

void FEHBuzzer::Tone( stdnote frequency )//tone at user defined frequency and infinite duration
{
    Tone( ( int ) frequency, 0 );
}

void FEHBuzzer::Tone( int frequency, int duration )//tone at user defined frequency and user defined integer duration (milliseconds)
{
    //to get low byte, typecase to unsigned char
    unsigned char frequency_low = ( unsigned char ) ( frequency & 0xFF );
    //to get high byte, right shift by eight and then cast
    unsigned char frequency_high = ( unsigned char ) ( ( frequency >> 8 ) & 0xFF );
    //to get low byte, typecase to unsigned char
    unsigned char duration_low = ( unsigned char ) ( duration & 0xFF );
    //to get high byte, right shift by eight and then cast
    unsigned char duration_high = ( unsigned char ) ( ( duration >> 8 ) & 0xFF );

    uart_putchar( UART5_BASE_PTR, 0x7F ); // start byte to propeller
    uart_putchar( UART5_BASE_PTR, 0x0A ); // command to propeller to signal the buzzer
    uart_putchar( UART5_BASE_PTR, frequency_high );
    uart_putchar( UART5_BASE_PTR, frequency_low );
    uart_putchar( UART5_BASE_PTR, duration_high );
    uart_putchar( UART5_BASE_PTR, duration_low );
    uart_putchar( UART5_BASE_PTR, 0xFF );

    Sleep( duration * 2);
}

void FEHBuzzer::Tone( int frequency, double duration)//tone at user defined frequency and user defined float duration (seconds)
{
    duration = duration * 1000;
    Tone( frequency, ( int ) duration );
}

void FEHBuzzer::Tone( stdnote frequency, int duration)//tone at user defined frequency and user defined integer duration (milliseconds)
{
    Tone( ( int ) frequency, duration );
}

void FEHBuzzer::Tone( stdnote frequency, double duration)//tone at user defined frequency and user defined float duration (seconds)
{
    duration = duration * 1000;
    Tone( ( int ) frequency, ( int ) duration );
}

void FEHBuzzer::Off()//turn off buzzer
{
    Tone( 0, 0 );
}

FEHBuzzer::FEHBuzzer(){}
