#include "FEHXBee.h"
#include "FEHProteus.h"
#include "FEHUtility.h"
#include "uart.h"
#include "FEHLCD.h"

void UART0_ISR();
void (*XBeeDataProcess)( unsigned char* data, unsigned char length );

typedef enum
{
	AwaitingStart, // awaiting reception of start byte
	FoundStart, // found start byte and receiving rest of packet
	PacketComplete
} XBeePacketState;

#define XBEEBUFFERSIZE 16
#define XBEEPACKETSIZE 13
#define XBEESTARTBYTE 0xFF
unsigned char _xbeebuffer[ XBEEBUFFERSIZE ];
unsigned char _xbeebufferindex = 0;
XBeePacketState _xbeestate = AwaitingStart;
bool _xbeebytereceived = false;
char _xbeebyte;

FEHXBee::FEHXBee()
{
	_initialized = false;
}

bool FEHXBee::Initialize()
{
	if( !_initialized )
	{
		_initialized = true;

		// Init uart clock for xbee connection
		SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;

		// XBEE UART:
		// Signal JTAG_TDI-XBEE_RX - UART0_RX - PTA1 - Pin 52
		// Signal JTAG_TDO-XBEE_TX - UART0_TX - PTA2 - Pin 53
		// UART function on Alt2, JTAG on Alt7
		PORTA_PCR1 = PORT_PCR_MUX( 0x2 );
		PORTA_PCR2 = PORT_PCR_MUX( 0x2 );

		// Initialize propeller UART
		uart_init( UART0_BASE_PTR, CoreClockKHz, 9600 );
//		uart_init( UART0_BASE_PTR, CoreClockKHz, 115200 );

		// Setup interrupt on UART0
		// disable receiver while changing settings
		UART_C2_REG( UART0_BASE_PTR ) &= ~( UART_C2_TE_MASK | UART_C2_RE_MASK );

		// enable interrupt
		UART_C2_REG( UART0_BASE_PTR ) |= UART_C2_RIE_MASK;

		// set to interrupt not dma
		UART_C5_REG( UART0_BASE_PTR ) &= ~( UART_C5_RDMAS_MASK );

		// enable interrupt on IRQ = 45
		NVICICPR1 |= 1 << ( 13 );
		NVICISER1 |= 1 << ( 13 );

		// enable receiver
		UART_C2_REG( UART0_BASE_PTR ) |= ( UART_C2_TE_MASK | UART_C2_RE_MASK );

		// wait for UART to init
		Sleep( 100 );

		_initialized = true;

		return true;
	}
	return false;
}

bool FEHXBee::IsInitialized()
{
	return _initialized;
}

unsigned int FEHXBee::ReceiveData( char* data, unsigned int maxlength )
{
	unsigned int i;
	unsigned int bufferindex;
	for( i = 0; i < maxlength; i++ )
	{
		while( !_xbeebytereceived );

//		data[ i ] = uart_getchar( UART0_BASE_PTR );
//		LCD.Write( (int)data[ i ] );
//		LCD.Write( " " );
		bufferindex = _xbeebufferindex - 1;
		if( bufferindex < 0 )
		{
			bufferindex = XBEEBUFFERSIZE - 1;
		}
		data[ i ] = _xbeebuffer[ bufferindex ];
		if( data[ i ] == 0x0D )
			break;
	}
	return i;
}

unsigned int FEHXBee::ReceiveDataSearch( char* data, unsigned int maxlength, unsigned char expectedfirstchar )
{
	unsigned int i;
	unsigned int bufferindex;
	bool foundfirst = false;
	for( i = 0; i < maxlength; i++ )
	{
		while( !_xbeebytereceived );

//		data[ i ] = uart_getchar( UART0_BASE_PTR );
//		LCD.Write( (int)data[ i ] );
//		LCD.Write( " " );
		data[ i ] = _xbeebyte;
		_xbeebytereceived = false;

		if( data[ i ] == 0x0D )
			break;

//		LCD.WriteLine( (int)data[ i ] );

		if( !foundfirst )
		{
			if( data[ i ] == expectedfirstchar )
			{
//				LCD.Write( 'F' );
				foundfirst = true;
			}
			else
			{
				i--;
			}
		}

	}
	return i;
}

unsigned int FEHXBee::SendData( char* data, unsigned int length )
{
	unsigned int i;
	for( i = 0; i < length; i++ )
	{
		uart_putchar( UART0_BASE_PTR, data[ i ] );
	}
	return i;
}

void FEHXBee::SetPacketCallBack( void (*packetcallbackfunction)( unsigned char* data, unsigned char length ) )
{
	XBeeDataProcess = packetcallbackfunction;
}

//void FEHXBee::EnableInterrupt()
//{
//	// enable interrupt on IRQ = 45
//	NVICICPR1 |= 1 << ( 13 );
//	NVICISER1 |= 1 << ( 13 );
//}

//void FEHXBee::DisableInterrupt()
//{
//	// disable interrupt on IRQ = 45
//	NVICICPR1 &= ~( 1 << ( 13 ) );
//	NVICISER1 &= ~( 1 << ( 13 ) );
//}

void UART0_ISR()
{
//  Echo test code
//	char c = uart_getchar( UART0_BASE_PTR );
//	uart_putchar( UART0_BASE_PTR, c );
//	LCD.Write( c );

	char c = uart_getchar( UART0_BASE_PTR );
	_xbeebyte = c;
	_xbeebytereceived = true;

	switch( _xbeestate )
	{
		case AwaitingStart:
		{
			if( c == XBEESTARTBYTE )
			{
				_xbeestate = FoundStart;
				_xbeebufferindex = 0;
				_xbeebuffer[ _xbeebufferindex++ ] = c;
			}
			break;
		}
		case FoundStart:
		{
			_xbeebuffer[ _xbeebufferindex++ ] = c;
            
			if( _xbeebufferindex == XBEEPACKETSIZE )
			{
				// call data processor
				XBeeDataProcess( _xbeebuffer, _xbeebufferindex );
				_xbeestate = PacketComplete;
			}
			break;
		}
		case PacketComplete:
		{
			if( c == XBEESTARTBYTE )
			{
				_xbeestate = FoundStart;
				_xbeebufferindex = 0;
				_xbeebuffer[ _xbeebufferindex++ ] = c;
			}
			else
			{
				_xbeebufferindex = 0;
				_xbeestate = AwaitingStart;
			}
			break;
		}
	}
}
