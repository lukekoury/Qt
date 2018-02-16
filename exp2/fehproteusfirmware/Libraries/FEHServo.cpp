#include <FEHServo.h>
#include <FEHIO.h>
#include "FEHProteus.h"
#include "derivative.h"
#include "uart.h"
#include "mcg.h"
#include <FEHUtility.h>
#include <FEHLCD.h>



FEHServo::FEHServo( FEHServoPort _servo )
{
    servo_port = _servo;
    servo_min = 500;
    servo_max = 2500;

	// _position == -1 => servo is off
	_position = -1;
}

void FEHServo::SetMin( int _min )
{
    // default to min 500 if value given is outside range
    if( _min < 500 || _min > 2500 )
    {
        servo_min = 500;
    }
    else
    {
        servo_min = _min;
    }
}

void FEHServo::SetMax( int _max )
{
    // default to max 2500 if value given is outside range
    if( _max < 500 || _max > 2500 )
    {
        servo_max = 2500;
    }
    else
    {
        servo_max = _max;
    }
}

void FEHServo::SetDegree( float _degree )
{
	if( _degree < 0 ) _degree = 0;
	if( _degree > 180 ) _degree = 180;

	if( _degree != _position )
	{
		_position = _degree;

		//set rate based on min ,max and degree provided
		unsigned short rate;
		rate = servo_min + (unsigned short)((servo_max - servo_min) / 180.0 * _degree);

		//to get low byte, typecase to unsigned char
		unsigned char rate_low = (unsigned char)( rate & 0xFF );
		//to get high byte, right shift by eight and then cast
		unsigned char rate_high = (unsigned char)((rate >> 8));

		uart_putchar( UART5_BASE_PTR, 0x7F );
		uart_putchar( UART5_BASE_PTR, 0x05 ); // servo channel set time instruction
		uart_putchar( UART5_BASE_PTR, (unsigned char) servo_port ); // servo to set
		uart_putchar( UART5_BASE_PTR, rate_high ); // on time high byte
		uart_putchar( UART5_BASE_PTR, rate_low ); // on time low byte
		uart_putchar( UART5_BASE_PTR, 0xFF );
	}
}

void FEHServo::DigitalOn()
{
    uart_putchar( UART5_BASE_PTR, 0x7F );
    uart_putchar( UART5_BASE_PTR, 0x09 );
    uart_putchar( UART5_BASE_PTR, (unsigned char) servo_port );
    uart_putchar( UART5_BASE_PTR, 0x01 );
    uart_putchar( UART5_BASE_PTR, 0xFF );

}

void FEHServo::DigitalOff()
{
    uart_putchar( UART5_BASE_PTR, 0x7F );
    uart_putchar( UART5_BASE_PTR, 0x09 );
    uart_putchar( UART5_BASE_PTR, (unsigned char) servo_port );
    uart_putchar( UART5_BASE_PTR, 0x00 );
    uart_putchar( UART5_BASE_PTR, 0xFF );
}

void FEHServo::Calibrate()
{
	unsigned short temp_min;
	
    LCD.Clear( FEHLCD::Black );
    LCD.SetFontColor( FEHLCD::White );
    DigitalInputPin leftbutton( FEHIO::P3_0 );
    DigitalInputPin middlebutton( FEHIO::P3_1 );
    DigitalInputPin rightbutton( FEHIO::P3_2 );

    LCD.WriteLine( "Use left and right buttons" );
    LCD.WriteLine( "to select min." );
    LCD.WriteLine( "Press middle button" );
    LCD.WriteLine( "when complete." );
    // set servo to 0 degrees using default min
    while( middlebutton.Value() )
    {
		_position = -1;
        this->SetDegree( 0 );
        while( leftbutton.Value() && rightbutton.Value() && middlebutton.Value() );

        // set min using left and right buttons
        if ( !leftbutton.Value() )
        {
            servo_min = servo_min - 1;
            if( servo_min < 500 ) servo_min = 500;
        }
        if ( !rightbutton.Value() )
        {
            servo_min = servo_min + 1;
            if( servo_min > 2500 ) servo_min = 2500;
        }
        Sleep( 5 );
    }
    while(!middlebutton.Value());
    Sleep(500);
	
	temp_min = servo_min;
	
	//detect if the user accidentally set a max instead of a min
	if (servo_min < 1500)
		servo_min = 2500;
	else
		servo_min = 500;

    LCD.Clear( FEHLCD::Black );
    LCD.WriteLine( "Use left and right buttons" );
    LCD.WriteLine( "to select max." );
    LCD.WriteLine( "Press middle button" );
    LCD.WriteLine( "when complete." );
    // set servo to 0 using default max (min if user accidentally set max)
    while( middlebutton.Value() )
    {
		_position = -1;
        this->SetDegree( 0 );
        while( leftbutton.Value() && rightbutton.Value() && middlebutton.Value() );

        // set max using left and right buttons
        if ( !leftbutton.Value() )
        {
            servo_min = servo_min - 1;
            if( servo_min < 500 ) servo_min = 500;
        }
        if ( !rightbutton.Value() )
        {
            servo_min = servo_min + 1;
            if( servo_min > 2500 ) servo_min = 2500;
        }
        Sleep( 5 );
    }
	
	//set the smaller value to min and larger value to max
	if (servo_min < temp_min)
	{
		servo_max = temp_min;
	}
	else
	{
		servo_max = servo_min;
		servo_min = temp_min;
	}

    // Print out servo min and servo max
    LCD.Clear( FEHLCD::Black );
    LCD.Write( "Min = " );
    LCD.Write( servo_min );
    LCD.Write( "   Max = " );
    LCD.WriteLine( servo_max );

}

void FEHServo::TouchCalibrate()
{
	unsigned short temp_min;
    float x=0, y=0;
	
    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);

    FEHIcon::Icon VAL[2];
    char val_labels[2][20] = {"Current Minimum", ""};
    FEHIcon::DrawIconArray(VAL, 2, 1, 41, 160, 1, 1, val_labels, YELLOW, WHITE);

    FEHIcon::Icon MOVE[2];
    char move_labels[2][20] = {"Backward", "Forward"};
    FEHIcon::DrawIconArray(MOVE, 1, 2, 80, 40, 1, 1, move_labels, RED, WHITE);

    FEHIcon::Icon SET[1];
    char set_label[1][20] = {"SET MIN"};
    FEHIcon::DrawIconArray(SET, 1, 1, 201, 2, 1, 1, set_label, BLUE, WHITE);

    LCD.WriteLine( "Use icons to select min." );
    LCD.WriteLine( "Press ""SET MIN"" when ready.");

    _position = -1;
    this->SetDegree( 0 );

    while (!SET[0].Pressed(x, y, 0))
    {
        VAL[1].ChangeLabelInt(servo_min);
        if (LCD.Touch(&x, &y))
        {
            if (MOVE[0].Pressed(x, y, 0))
            {
                while (MOVE[0].Pressed(x, y, 1))
                {
                    servo_min = servo_min - 1;
                    if( servo_min < 500 ) servo_min = 500;
                    _position = -1;
                    this->SetDegree(0);
                    VAL[1].ChangeLabelInt(servo_min);
                }
                MOVE[0].Deselect();
            }
            if (MOVE[1].Pressed(x, y, 0))
            {
                while (MOVE[1].Pressed(x, y, 1))
                {
                    servo_min = servo_min + 1;
                    if( servo_min > 2500 ) servo_min = 2500;
                    _position = -1;
                    this->SetDegree(0);
                    VAL[1].ChangeLabelInt(servo_min);
                }
                MOVE[1].Deselect();
            }
        }
    }
    SET[0].WhilePressed(x, y);
    SET[0].Deselect();

	temp_min = servo_min;
	
	//detect if the user accidentally set a max instead of a min
	if (servo_min < 1500)
		servo_min = 2500;
	else
		servo_min = 500;

    LCD.Clear(BLACK);

    VAL[0].ChangeLabelString("Current Maximum");
    VAL[0].Draw();
    VAL[1].Draw();

    FEHIcon::DrawIconArray(MOVE, 1, 2, 80, 40, 1, 1, move_labels, RED, WHITE);

    SET[0].ChangeLabelString("SET MAX");
    SET[0].Draw();

    LCD.WriteLine( "Use icons to select max." );
    LCD.WriteLine( "Press ""SET MAX"" when ready.");

    _position = -1;
    this->SetDegree( 0 );

    while (!SET[0].Pressed(x, y, 0))
    {
        VAL[1].ChangeLabelInt(servo_min);
        if (LCD.Touch(&x, &y))
        {
            if (MOVE[0].Pressed(x, y, 0))
            {
                while (MOVE[0].Pressed(x, y, 1))
                {
                    servo_min = servo_min - 1;
                    if( servo_min < 500 ) servo_min = 500;
                    _position = -1;
                    this->SetDegree(0);
                    VAL[1].ChangeLabelInt(servo_min);
                }
                MOVE[0].Deselect();
            }
            if (MOVE[1].Pressed(x, y, 0))
            {
                while (MOVE[1].Pressed(x, y, 1))
                {
                    servo_min = servo_min + 1;
                    if( servo_min > 2500 ) servo_min = 2500;
                    _position = -1;
                    this->SetDegree(0);
                    VAL[1].ChangeLabelInt(servo_min);
                }
                MOVE[1].Deselect();
            }
        }
    }
    SET[0].WhilePressed(x, y);
    SET[0].Deselect();

	//set the smaller value to min and larger value to max
	if (servo_min < temp_min)
	{
		servo_max = temp_min;
	}
	else
	{
		servo_max = servo_min;
		servo_min = temp_min;
	}

    LCD.Clear(BLACK);

    FEHIcon::Icon OUT[4];
    char out_labels[4][20] = {"SERVO MIN", "SERVO MAX", "", ""};
    FEHIcon::DrawIconArray(OUT, 2, 2, 80, 120, 20, 20, out_labels, BLACK, WHITE);

    FEHIcon::Icon EXIT[1];
    char exit_label[1][20] = {"EXIT"};
    FEHIcon::DrawIconArray(EXIT, 1, 1, 121, 40, 20, 20, exit_label, RED, WHITE);

    OUT[2].ChangeLabelInt(servo_min);
    OUT[2].Draw();
    OUT[3].ChangeLabelInt(servo_max);
    OUT[3].Draw();

    while (!EXIT[0].Pressed(x, y, 0))
    {
        LCD.Touch(&x, &y);
    }
    EXIT[0].WhilePressed(x, y);
    LCD.Clear(BLACK);
}

void FEHServo::Off()
{
	if( _position >= 0 )
	{
		_position = -1;

		uart_putchar( UART5_BASE_PTR, 0x7F );
		uart_putchar( UART5_BASE_PTR, 0x06 ); // servo off type
		uart_putchar( UART5_BASE_PTR, (unsigned char) servo_port ); // servo to turn off
		uart_putchar( UART5_BASE_PTR, 0xFF );
	}
}
