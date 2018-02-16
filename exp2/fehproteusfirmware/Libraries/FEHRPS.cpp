#include "FEHRPS.h"
#include "FEHIO.h"
#include "FEHLCD.h"
#include "FEHSD.h"
#include "FEHUtility.h"
#include "FEHServo.h"
#include "string.h"

FEHRPS RPS;

#define SATELLITEMASK    0x0000FF // bits 0,1,2,3,4,5,6,7 (1 byte)

#define STOPDATA 0xAA

void RPSDataProcess( unsigned char *data, unsigned char length );

bool _enabled;
int _region;
float _RPS_x;
float _RPS_y;
float _RPS_heading;
long _RPS_objective;
unsigned char _RPS_time;
bool _RPS_stop;
bool _RPS_foundpacket;


FEHRPS::FEHRPS()
{
    _xbee.SetPacketCallBack( &RPSDataProcess );

	//_enabled = true;
	_region = -1;

    _RPS_x = 0.0f;
    _RPS_y = 0.0f;
    _RPS_heading = 0.0f;
    _RPS_objective = 0x0;
    _RPS_time = 0;
    _RPS_stop = false;
}

// Manually pick and configure a region
// int region => { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 }
// char region => { a, b, c, d, e, f, g, h, i, j, k, l } || { A, B, C, D, E, F, G, H, I, J, K, L }
void FEHRPS::Initialize( int region )
{
    if( !_xbee.IsInitialized() )
        _xbee.Initialize();

    if( !_enabled )
    {
//		if( !_xbee.IsInitialized() )
        _xbee.Initialize();

    if( !_enabled )
    {
//		_enabled = true;
//		return;

        // check range of region
        if( region < 0 )
        {
            region = 0;
        }
        else if( region > 11 )
        {
            region = 11;
        }

        _region = region;

        // configure xbee

        //_xbee.DisableInterrupt();

        LCD.Clear();

        char txbuffer[ 10 ];
        unsigned int txlength = 0;

        char rxbuffer[ 10 ];
        unsigned int rxlength = 0;

        // enter AT mode
        bool entered_cmd_mode(false);
        while(!entered_cmd_mode)
        {
            LCD.Write( "Enter cmd mode..." );
            txbuffer[ 0 ] = '+';
            txbuffer[ 1 ] = '+';
            txbuffer[ 2 ] = '+';
            txlength = _xbee.SendData( txbuffer, 3 );
            //Sleep( 1000 );
            rxlength = _xbee.ReceiveDataSearch( rxbuffer, 10, 'O' );
            if( rxlength < 2 || rxbuffer[ 0 ] != 'O' ) // if error
            {
                LCD.WriteLine(" ");
                LCD.WriteLine("Waiting for response...");
                Sleep(2500);
                rxlength = _xbee.ReceiveDataSearch( rxbuffer, 10, '0');
                if( rxlength < 2 || rxbuffer[ 0 ] != 'O' ) // if error
                {
                    LCD.WriteLine( "Enter cmd mode FAILED" );
                    LCD.Write( "with Data: ");
                    for(int i=0 ; i<rxlength ; i++)
                    {
                        LCD.Write(rxbuffer[i]);
                    }
                    LCD.WriteLine( " " );
                    LCD.WriteLine("Trying again...");
                }
                else
                    entered_cmd_mode = true;
            }
            else
                entered_cmd_mode = true;
        }
        LCD.Write("Data: ");
        for( int i = 0; i < rxlength; i++ )
        {
            LCD.Write( rxbuffer[ i ] );
//			LCD.Write( " " );
        }
        LCD.WriteLine( " " );

        // set my address
        LCD.Write( "Set wireless id..." );
        txbuffer[ 0 ] = 'A';
        txbuffer[ 1 ] = 'T';
        txbuffer[ 2 ] = 'M';
        txbuffer[ 3 ] = 'Y';
        txbuffer[ 4 ] = '0' + ( _region / 4 + 1 );
        txbuffer[ 5 ] = '1';
        txbuffer[ 6 ] = '0';
        txbuffer[ 7 ] = '0' + ( _region % 4 );
        txbuffer[ 8 ] = '\r';
        txlength = _xbee.SendData( txbuffer, 9 );
    //	Sleep( 100 );
        rxlength = _xbee.ReceiveDataSearch( rxbuffer, 10, 'O' );
        if( rxlength < 2 || rxbuffer[ 0 ] != 'O' ) // if error
        {
            LCD.WriteLine( "Error with RPS configuration" );
            return;
        }
        for( int i = 0; i < rxlength; i++ )
        {
            LCD.Write( rxbuffer[ i ] );
//			LCD.Write( " " );
        }
        LCD.WriteLine( " " );

        // set my pan id
        LCD.Write( "Set network id..." );
        txbuffer[ 0 ] = 'A';
        txbuffer[ 1 ] = 'T';
        txbuffer[ 2 ] = 'I';
        txbuffer[ 3 ] = 'D';
        txbuffer[ 4 ] = '0' + ( _region / 4 + 1 );
        txbuffer[ 5 ] = '0';
        txbuffer[ 6 ] = '0';
        txbuffer[ 7 ] = '0';
        txbuffer[ 8 ] = '\r';
        txlength = _xbee.SendData( txbuffer, 9 );
    //	Sleep( 100 );
        rxlength = _xbee.ReceiveDataSearch( rxbuffer, 10, 'O' );
        if( rxlength < 2 || rxbuffer[ 0 ] != 'O' ) // if error
        {
            LCD.WriteLine( "Error with RPS configuration" );
            return;
        }
        for( int i = 0; i < rxlength; i++ )
        {
            LCD.Write( rxbuffer[ i ] );
        }
        LCD.WriteLine( " " );

        // set my destination (low)
        LCD.Write( "Set course id (low)..." );
        txbuffer[ 0 ] = 'A';
        txbuffer[ 1 ] = 'T';
        txbuffer[ 2 ] = 'D';
        txbuffer[ 3 ] = 'L';
        txbuffer[ 4 ] = '0' + ( _region / 4 + 1 );
        txbuffer[ 5 ] = '0';
        txbuffer[ 6 ] = '0';
        txbuffer[ 7 ] = '0' + ( _region % 4 );
        txbuffer[ 8 ] = '\r';
        txlength = _xbee.SendData( txbuffer, 9 );
    //	Sleep( 100 );
        rxlength = _xbee.ReceiveDataSearch( rxbuffer, 10, 'O' );
        if( rxlength < 2 || rxbuffer[ 0 ] != 'O' ) // if error
        {
            LCD.WriteLine( "Error with RPS configuration" );
            return;
        }
        for( int i = 0; i < rxlength; i++ )
        {
            LCD.Write( rxbuffer[ i ] );
        }
        LCD.WriteLine( " " );

        // set my destination (high)
        LCD.Write( "Set course id (high)..." );
        txbuffer[ 0 ] = 'A';
        txbuffer[ 1 ] = 'T';
        txbuffer[ 2 ] = 'D';
        txbuffer[ 3 ] = 'H';
        txbuffer[ 4 ] = '0';
        txbuffer[ 5 ] = '0';
        txbuffer[ 6 ] = '0';
        txbuffer[ 7 ] = '0';
        txbuffer[ 8 ] = '\r';
        txlength = _xbee.SendData( txbuffer, 9 );
    //	Sleep( 100 );
        rxlength = _xbee.ReceiveDataSearch( rxbuffer, 10, 'O' );
        if( rxlength < 2 || rxbuffer[ 0 ] != 'O' ) // if error
        {
            LCD.WriteLine( "Error with RPS configuration" );
            return;
        }
        for( int i = 0; i < rxlength; i++ )
        {
            LCD.Write( rxbuffer[ i ] );
        }
        LCD.WriteLine( " " );

        // write to flash
        LCD.Write( "Writing config..." );
        txbuffer[ 0 ] = 'A';
        txbuffer[ 1 ] = 'T';
        txbuffer[ 2 ] = 'W';
        txbuffer[ 3 ] = 'R';
        txbuffer[ 4 ] = '\r';
        txlength = _xbee.SendData( txbuffer, 5 );
    //	Sleep( 100 );
        rxlength = _xbee.ReceiveDataSearch( rxbuffer, 10, 'O' );
        if( rxlength < 2 || rxbuffer[ 0 ] != 'O' ) // if error
        {
            LCD.WriteLine( "Error with RPS configuration" );
            return;
        }
        for( int i = 0; i < rxlength; i++ )
        {
            LCD.Write( rxbuffer[ i ] );
        }
        LCD.WriteLine( " " );

        // exit command mode
        LCD.Write( "Exit command mode..." );
        txbuffer[ 0 ] = 'A';
        txbuffer[ 1 ] = 'T';
        txbuffer[ 2 ] = 'C';
        txbuffer[ 3 ] = 'N';
        txbuffer[ 4 ] = '\r';
        txlength = _xbee.SendData( txbuffer, 5 );
    //	Sleep( 100 );
        rxlength = _xbee.ReceiveDataSearch( rxbuffer, 10, 'O' );
        if( rxlength < 2 || rxbuffer[ 0 ] != 'O' ) // if error
        {
            LCD.WriteLine( "Error with RPS configuration" );
            return;
        }
        for( int i = 0; i < rxlength; i++ )
        {
            LCD.Write( rxbuffer[ i ] );
        }
        LCD.WriteLine( " " );

        // show success
        LCD.Write( "Successfully initialized  region: " );
        LCD.WriteLine( CurrentRegionLetter() );

//		_xbee.EnableInterrupt();

        //Wait to ensure that a connection is made
        _enabled = true;
        while( WaitForPacket() == 0x00);
        LCD.WriteLine("RPS Enabled Successfully");
        LCD.Clear();
    }
    }
}

void FEHRPS::Initialize( char region )
{
    if( region >= 'A' && region <= 'L' )
    {
        Initialize( (int)( region - 'A' ) );
    }
    else if( region >= 'a' && region <= 'l' )
    {
        Initialize( (int)( region - 'a' ) );
    }
}

void FEHRPS::InitializeMenu()
{
	ButtonBoard buttons( FEHIO::Bank3 );

	char region = 'A';

	LCD.Clear();
	LCD.WriteLine( "Use LEFT / RIGHT to change region" );
	LCD.WriteLine( "Use MIDDLE to select" );
	LCD.Write( "Region: " );
	LCD.WriteLine( region );

	// wait for user to press middle button
	while( !buttons.MiddlePressed() )
	{
		if( buttons.LeftPressed() )
		{
			region--;
			if( region < 'A' )
			{
				region = 'L';
			}
			LCD.Clear();
			LCD.WriteLine( "Use LEFT / RIGHT to change region" );
			LCD.WriteLine( "Use MIDDLE to select" );
			LCD.Write( "Region: " );
			LCD.WriteLine( region );

			while( buttons.LeftPressed() );
			Sleep( 100 );
		}

		if( buttons.RightPressed() )
		{
			region++;
			if( region > 'L' )
			{
				region = 'A';
			}
			LCD.Clear();
			LCD.WriteLine( "Use LEFT / RIGHT to change region" );
			LCD.WriteLine( "Use MIDDLE to select" );
			LCD.Write( "Region: " );
			LCD.WriteLine( region );

			while( buttons.RightPressed() );
			Sleep( 100 );
		}
	}

	Initialize( region );

	//while( buttons.MiddlePressed() );
	//Sleep( 1000 );
}

void FEHRPS::InitializeTouchMenu()
{
	int cancel = 1;
    int c=0, d=0, n;
    float x, y;
    char region;

    FEHIcon::Icon regions_title[1];
    char regions_title_label[1][20] = {"Select RPS Region"};

    FEHIcon::Icon regions[8];
    char regions_labels[8][20] = {"A", "B", "C", "D", "E", "F", "G", "H"};

    FEHIcon::Icon confirm_title[1];
    char confirm_title_label[1][20] = {""};

    FEHIcon::Icon confirm[2];
    char confirm_labels[2][20] = {"Ok", "Cancel"};

	while(cancel) {
    c=0;
    d=0;
    LCD.Clear(BLACK);

    FEHIcon::DrawIconArray(regions_title, 1, 1, 1, 201, 1, 1, regions_title_label, BLACK, WHITE);
    FEHIcon::DrawIconArray(regions, 2, 4, 40, 1, 1, 1, regions_labels, WHITE, WHITE);

	while (!c)
	{
        if (LCD.Touch(&x, &y))
        {
            for (n=0; n<=7; n++)
            {
                if (regions[n].Pressed(x, y, 0))
                {
                    regions[n].WhilePressed(x, y);
                    c = n+1;
                }
            }
        }
	}
    switch (c)
    {
    case 1:
        region = 'A';
        strcpy(confirm_title_label[0], "Choice: A");
        break;
    case 2:
        region = 'B';
        strcpy(confirm_title_label[0], "Choice: B");
        break;
    case 3:
        region = 'C';
        strcpy(confirm_title_label[0], "Choice: C");
        break;
    case 4:
        region = 'D';
        strcpy(confirm_title_label[0], "Choice: D");
        break;
    case 5:
        region = 'E';
        strcpy(confirm_title_label[0], "Choice: E");
        break;
    case 6:
        region = 'F';
        strcpy(confirm_title_label[0], "Choice: F");
        break;
    case 7:
        region = 'G';
        strcpy(confirm_title_label[0], "Choice: G");
        break;
    case 8:
        region = 'H';
        strcpy(confirm_title_label[0], "Choice: H");
        break;
    }

    LCD.Clear(BLACK);
    FEHIcon::DrawIconArray(confirm_title, 1, 1, 60, 201, 1, 1, confirm_title_label, BLACK, WHITE);
    FEHIcon::DrawIconArray(confirm, 1, 2, 60, 60, 1, 1, confirm_labels, WHITE, WHITE);

	while (!d)
	{
        if (LCD.Touch(&x, &y))
        {
            for (n=0; n<=1; n++)
            {
                if (confirm[n].Pressed(x, y, 0))
                {
                    confirm[n].WhilePressed(x, y);
                    d = n+1;
                }
            }
        }
	}
    switch (d)
    {
    case 1:
        cancel = 0;
        break;
    case 2:
        cancel = 1;
        break;
    }
    }

    Initialize( region );

}


// return the current course number { 1, 2, 3 }
unsigned char FEHRPS::CurrentCourse()
{
	if( _region >= 0 )
	{
		return (unsigned char)( _region / 4 + 1 );
	}

	return 0xFF;
}

// returns the letter of the current region { A, B, C, D, E, F, G, H, I, J, K, L }
char FEHRPS::CurrentRegionLetter()
{
	return ( 'A' + (char)_region );
}

// returns the number of the current course { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 }
int FEHRPS::CurrentRegion()
{
	return _region;
}

// Objective functions:

// returns 0 always due to visual scoring
int FEHRPS::SatellitePercent()
{
    return 0;
}

// returns the match time in seconds
int FEHRPS::Time()
{
    return (int)_RPS_time;
}

unsigned char FEHRPS::WaitForPacket()
{
	unsigned long starttime = TimeNowMSec();
    while( !_RPS_foundpacket && ( TimeNowMSec() - starttime ) < 1000 );
    if( _RPS_foundpacket )
	{
        _RPS_foundpacket = false;
        return 0xFF;
	}
	else
	{
		return 0x00;
	}
}

float FEHRPS::X()
{
	return _RPS_x;
}

float FEHRPS::Y()
{
	return _RPS_y;
}

float FEHRPS::Heading()
{
	return _RPS_heading;
}

void RPSDataProcess( unsigned char *data, unsigned char length )
{
	if( _enabled )
	{
        _RPS_x = (float)( (int)( ( ( (unsigned int)data[ 1 ] ) << 8 ) + (unsigned int)data[ 2 ] ) ) / 10.0f - 1600.0f;
        _RPS_y = (float)( (int)( ( ( (unsigned int)data[ 3 ] ) << 8 ) + (unsigned int)data[ 4 ] ) ) / 10.0f - 1600.0f;
        _RPS_heading = (float)( (int)( ( ( (unsigned int)data[ 5 ] ) << 8 ) + (unsigned int)data[ 6 ] ) ) / 10.0f - 1600.0f;
        _RPS_objective = ((data[ 7 ] << 24 ) + (data[ 8 ] << 16 ) + (data[ 9 ] << 8) + (data[ 10 ]));
        _RPS_time = data[ 11 ];
        _RPS_stop = (data[12] == STOPDATA);
        _RPS_foundpacket = true;
		
        if(_RPS_stop)
		{
			// set kill pin low for power shutdown
            SD.CloseLog();
			GPIOD_PDOR &= ~GPIO_PDOR_PDO( GPIO_PIN( 13 ) );
		}
	}
	
	
}
