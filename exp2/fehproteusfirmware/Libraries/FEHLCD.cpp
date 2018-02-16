#include "FEHLCD.h"
#include "FEHUtility.h"
#include "derivative.h"
#include "mcg.h"
#include "lptmr.h"
#include "stdio.h"
#include "image.h"
#include "spi.h"
#include <FEHBuzzer.h>
#include <string.h>
#include <LCDColors.h>

#define CLR_CS GPIOC_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 3 ) )
#define SET_CS GPIOC_PDOR |= GPIO_PDOR_PDO( ( 1 << 3 ) )

#define CLR_RS GPIOB_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 18 ) )
#define SET_RS GPIOB_PDOR |= GPIO_PDOR_PDO( ( 1 << 18 ) )

#define CLR_Rd GPIOB_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 16 ) )
#define SET_RD GPIOB_PDOR |= GPIO_PDOR_PDO( ( 1 << 16 ) )

#define CLR_WR GPIOB_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 17 ) )
#define SET_WR GPIOB_PDOR |= GPIO_PDOR_PDO( ( 1 << 17 ) )

#define CLR_RESET GPIOB_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 3 ) )
#define SET_RESET GPIOB_PDOR |= GPIO_PDOR_PDO( ( 1 << 3 ) )


#define CharHeight 17
#define CharWidth  12

unsigned char FEHLCD::fontData[] = {
    0x00, 0x00, 0x00, 0x00, 0x00,// (space)
    0x00, 0x00, 0x5F, 0x00, 0x00,// !
    0x00, 0x07, 0x00, 0x07, 0x00,// "
    0x14, 0x7F, 0x14, 0x7F, 0x14,// #
    0x24, 0x2A, 0x7F, 0x2A, 0x12,// $
    0x23, 0x13, 0x08, 0x64, 0x62,// %
    0x36, 0x49, 0x55, 0x22, 0x50,// &
    0x00, 0x05, 0x03, 0x00, 0x00,// '
    0x00, 0x1C, 0x22, 0x41, 0x00,// (
    0x00, 0x41, 0x22, 0x1C, 0x00,// )
    0x08, 0x2A, 0x1C, 0x2A, 0x08,// *
    0x08, 0x08, 0x3E, 0x08, 0x08,// +
    0x00, 0x50, 0x30, 0x00, 0x00,// ,
    0x08, 0x08, 0x08, 0x08, 0x08,// -
    0x00, 0x60, 0x60, 0x00, 0x00,// .
    0x20, 0x10, 0x08, 0x04, 0x02,// /
    0x3E, 0x51, 0x49, 0x45, 0x3E,// 0
    0x00, 0x42, 0x7F, 0x40, 0x00,// 1
    0x42, 0x61, 0x51, 0x49, 0x46,// 2
    0x21, 0x41, 0x45, 0x4B, 0x31,// 3
    0x18, 0x14, 0x12, 0x7F, 0x10,// 4
    0x27, 0x45, 0x45, 0x45, 0x39,// 5
    0x3C, 0x4A, 0x49, 0x49, 0x30,// 6
    0x01, 0x71, 0x09, 0x05, 0x03,// 7
    0x36, 0x49, 0x49, 0x49, 0x36,// 8
    0x06, 0x49, 0x49, 0x29, 0x1E,// 9
    0x00, 0x36, 0x36, 0x00, 0x00,// :
    0x00, 0x56, 0x36, 0x00, 0x00,// ;
    0x00, 0x08, 0x14, 0x22, 0x41,// <
    0x14, 0x14, 0x14, 0x14, 0x14,// =
    0x41, 0x22, 0x14, 0x08, 0x00,// >
    0x02, 0x01, 0x51, 0x09, 0x06,// ?
    0x32, 0x49, 0x79, 0x41, 0x3E,// @
    0x7E, 0x11, 0x11, 0x11, 0x7E,// A
    0x7F, 0x49, 0x49, 0x49, 0x36,// B
    0x3E, 0x41, 0x41, 0x41, 0x22,// C
    0x7F, 0x41, 0x41, 0x22, 0x1C,// D
    0x7F, 0x49, 0x49, 0x49, 0x41,// E
    0x7F, 0x09, 0x09, 0x01, 0x01,// F
    0x3E, 0x41, 0x41, 0x51, 0x32,// G
    0x7F, 0x08, 0x08, 0x08, 0x7F,// H
    0x00, 0x41, 0x7F, 0x41, 0x00,// I
    0x20, 0x40, 0x41, 0x3F, 0x01,// J
    0x7F, 0x08, 0x14, 0x22, 0x41,// K
    0x7F, 0x40, 0x40, 0x40, 0x40,// L
    0x7F, 0x02, 0x04, 0x02, 0x7F,// M
    0x7F, 0x04, 0x08, 0x10, 0x7F,// N
    0x3E, 0x41, 0x41, 0x41, 0x3E,// O
    0x7F, 0x09, 0x09, 0x09, 0x06,// P
    0x3E, 0x41, 0x51, 0x21, 0x5E,// Q
    0x7F, 0x09, 0x19, 0x29, 0x46,// R
    0x46, 0x49, 0x49, 0x49, 0x31,// S
    0x01, 0x01, 0x7F, 0x01, 0x01,// T
    0x3F, 0x40, 0x40, 0x40, 0x3F,// U
    0x1F, 0x20, 0x40, 0x20, 0x1F,// V
    0x7F, 0x20, 0x18, 0x20, 0x7F,// W
    0x63, 0x14, 0x08, 0x14, 0x63,// X
    0x03, 0x04, 0x78, 0x04, 0x03,// Y
    0x61, 0x51, 0x49, 0x45, 0x43,// Z
    0x00, 0x00, 0x7F, 0x41, 0x41,// [
    0x02, 0x04, 0x08, 0x10, 0x20,// "\"
    0x41, 0x41, 0x7F, 0x00, 0x00,// ]
    0x04, 0x02, 0x01, 0x02, 0x04,// ^
    0x40, 0x40, 0x40, 0x40, 0x40,// _
    0x00, 0x01, 0x02, 0x04, 0x00,// `
    0x20, 0x54, 0x54, 0x54, 0x78,// a
    0x7F, 0x48, 0x44, 0x44, 0x38,// b
    0x38, 0x44, 0x44, 0x44, 0x20,// c
    0x38, 0x44, 0x44, 0x48, 0x7F,// d
    0x38, 0x54, 0x54, 0x54, 0x18,// e
    0x08, 0x7E, 0x09, 0x01, 0x02,// f
    0x08, 0x14, 0x54, 0x54, 0x3C,// g
    0x7F, 0x08, 0x04, 0x04, 0x78,// h
    0x00, 0x44, 0x7D, 0x40, 0x00,// i
    0x20, 0x40, 0x44, 0x3D, 0x00,// j
    0x00, 0x7F, 0x10, 0x28, 0x44,// k
    0x00, 0x41, 0x7F, 0x40, 0x00,// l
    0x7C, 0x04, 0x18, 0x04, 0x78,// m
    0x7C, 0x08, 0x04, 0x04, 0x78,// n
    0x38, 0x44, 0x44, 0x44, 0x38,// o
    0x7C, 0x14, 0x14, 0x14, 0x08,// p
    0x08, 0x14, 0x14, 0x18, 0x7C,// q
    0x7C, 0x08, 0x04, 0x04, 0x08,// r
    0x48, 0x54, 0x54, 0x54, 0x20,// s
    0x04, 0x3F, 0x44, 0x40, 0x20,// t
    0x3C, 0x40, 0x40, 0x20, 0x7C,// u
    0x1C, 0x20, 0x40, 0x20, 0x1C,// v
    0x3C, 0x40, 0x30, 0x40, 0x3C,// w
    0x44, 0x28, 0x10, 0x28, 0x44,// x
    0x0C, 0x50, 0x50, 0x50, 0x3C,// y
    0x44, 0x64, 0x54, 0x4C, 0x44,// z
    0x00, 0x08, 0x36, 0x41, 0x00,// {
    0x00, 0x00, 0x7F, 0x00, 0x00,// |
    0x00, 0x41, 0x36, 0x08, 0x00,// }
    0x08, 0x08, 0x2A, 0x1C, 0x08,// ->
    0x08, 0x1C, 0x2A, 0x08, 0x08 // <-
};

FEHLCD LCD;

bool initialized = false;


#define LCD_WIDTH           320
#define LCD_HEIGHT          240

//Set DotClk Frequency
#define DotClk_H        0x01//01
#define DotClk_M        0xa0//50
#define DotClk_L        0xff

//Set the LCD panel mode
#define Panel_Data_Width_Bit   0 //0:18bit; 1:24bit;


/*--------------------Display pixel format[6:4] (POR = 000)
                        000 Reserved
                        001 3-bit/pixel
                        010 8-bit/pixel
                        011 12-bit/pixel
                        100 Reserved
                        101 16-bit/pixel
                        110 18-bit/pixel
                        111 24-bit/pixel-------------------*/
#define Display_Pixel_Format   6



/*--------------------Pixel Data Interface Format (POR = 101)
                        000 8-bit
                        001 12-bit
                        010 16-bit packed
                        011 16-bit (565 format)
                        100 18-bit
                        101 24-bit
                        110 9-bit
                        Others Reserved-------------------*/
#define Pixel_Data_Interface_Format  4//3

#define DotClk_Polarity        1 //0:Data latch in rising edge; 1:Data latch in falling edge
#define Hsync_Polarity         0 //0:Active low; 1:Active high
#define Vsync_Polarity         0 //0:Active low; 1:Active high

#define LCD_Panel_Mode         0 //0:Hsync+Vsync +DE mode; 1:TTL mode
#define TFT_Type               0 //0 or 1:TFT mode; 2:Serial RGB mode; 3:Serial RGB+dummy mode
#define LCD_Data_Pin_Strength  3 //0 to 3:weakest to strongest
#define LCD_Control_Pin_Strength  3 //0 to 3:weakest to strongest

#define HPS      (LCD_WIDTH-1)  //Set the horizontal panel size
#define VPS      (LCD_HEIGHT-1)  //Set the vertical panel size

  /*---------------G[5:3] : Even line RGB sequence (POR = 000)
                        000 RGB
                        001 RBG
                        010 GRB
                        011 GBR
                        100 BRG
                        101 BGR
                        11x Reserved ------------------*/
  /*---------------G[2:1] : Odd line RGB sequence (POR = 000)
                        000 RGB
                        001 RBG
                        010 GRB
                        011 GBR
                        100 BRG
                        101 BGR
                        11x Reserved ------------------*/

#define G  0

#define Panel_Mode1  (Panel_Data_Width_Bit<<5)|(1<<4)|(1<<3)|(DotClk_Polarity<<2)|(Hsync_Polarity<<1)|(Vsync_Polarity)
#define Panel_Mode2  (LCD_Panel_Mode<<7)|(TFT_Type<<5)|(LCD_Data_Pin_Strength<<2)|(LCD_Control_Pin_Strength<<0)
#define Panel_Mode3  G

//Horizontal Cycle = Horizontal Pulse Width + Horizontal Back Porch + Horizontal Display Period + Horizontal Front Porch
//Horizontal Start Position = Horizontal Pulse Width + Horizontal Back Porch
#define LCD_HORI_FRONT_PORCH        0x45//0x40//0x90//0x30//2
#define LCD_HORI_BACK_PORCH         0x12//0x10//0x05//0x0F//2
#define LCD_HORI_PULSE_WIDTH        0x02//1e //0x50//0x1e//0x60//41

//Vertical Cycle = Vertical Pulse Width + Vertical Back Porch + Vertical Display Period + Vertical Front Porch
//Vertical Start Position = Vertical Pulse Width + Vertical Back Porch
#define LCD_VERT_FRONT_PORCH        14//3//2
#define LCD_VERT_BACK_PORCH         9//2//0//2
#define LCD_VERT_PULSE_WIDTH        2//2//10

#define REFRESH_RATE                70  //Hz

#define LCD_HORI_CYCLE              (LCD_HORI_PULSE_WIDTH + LCD_HORI_BACK_PORCH + LCD_WIDTH + LCD_HORI_FRONT_PORCH)
#define LCD_VERT_CYCLE              (LCD_VERT_PULSE_WIDTH + LCD_VERT_BACK_PORCH + LCD_HEIGHT + LCD_VERT_FRONT_PORCH)

#define LCD_HORI_START              (LCD_HORI_PULSE_WIDTH + LCD_HORI_BACK_PORCH)
#define LCD_VERT_START              (LCD_VERT_PULSE_WIDTH + LCD_VERT_BACK_PORCH)


void FEHLCD::_Initialize()
{
    // Setup pins
    // CS - C3
    // RS - B18
    // RD - B16
    // WR - B17
    // RESET - B3
    PORTC_PCR3 = ( PORT_PCR_MUX( 1 ) );
    PORTB_PCR18 = ( PORT_PCR_MUX( 1 ) );
    PORTB_PCR16 = ( PORT_PCR_MUX( 1 ) );
    PORTB_PCR17 = ( PORT_PCR_MUX( 1 ) );
    PORTB_PCR3 = ( PORT_PCR_MUX( 1 ) );
    GPIOC_PDDR |= ( 1 << 3 );
    GPIOB_PDDR |= ( ( 1 << 18 ) );
    GPIOB_PDDR |= ( ( 1 << 16 ) );
    GPIOB_PDDR |= ( ( 1 << 17 ) );
    SET_RESET;
    GPIOB_PDDR |= ( ( 1 << 3 ) );
    // CS = 1;
    SET_CS;


    // D0 - B9
    // D1 - B8
    // D2 - C4
    // D3 - C5
    // D4 - C6
    // D5 - C7
    // D6 - C12
    // D7 - C13
    // D8 - C14
    // D9 - C15
    // D10 - C16
    // D11 - C17
    // D12 - C18
    // D13 - C19
    // D14 - D0
    // D15 - D2
    // D16 - D3
    // D17 - D4
    PORTB_PCR9 = ( PORT_PCR_MUX( 1 ) );
    PORTB_PCR8 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR4 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR5 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR6 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR7 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR12 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR13 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR14 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR15 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR16 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR17 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR18 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR19 = ( PORT_PCR_MUX( 1 ) );
    PORTD_PCR0 = ( PORT_PCR_MUX( 1 ) );
    PORTD_PCR2 = ( PORT_PCR_MUX( 1 ) );
    PORTD_PCR3 = ( PORT_PCR_MUX( 1 ) );
    PORTD_PCR4 = ( PORT_PCR_MUX( 1 ) );
    GPIOB_PDDR |= ( ( 1 << 9 ) | ( 1 << 8 ) );
    GPIOC_PDDR |= ( ( 1 << 4 ) | ( 1 << 5 ) | ( 1 << 6 ) | ( 1 << 7 ) | ( 1 << 12 ) | ( 1 << 13 ) | ( 1 << 14 ) | ( 1 << 15 ) | ( 1 << 16 ) | ( 1 << 17 ) | ( 1 << 18 ) | ( 1 << 19 ) );
    GPIOD_PDDR |= ( ( 1 << 0 ) | ( 1 << 2 ) | ( 1 << 3 ) | ( 1 << 4 ) );




    //PLL frequency=((0x2A+1)*8(crystal clock))/(0x02+1)
    WriteIndex(0xE2);
    WriteParameter(0x1D);    //0x2A
    WriteParameter(0x02);
    WriteParameter(0x04);

    //Start the PLL
    WriteIndex(0xE0);
    WriteParameter(0x01);

    Sleep(100);

    WriteIndex(0xE0);
    WriteParameter(0x03);

    //software reset
    WriteIndex(0x01);

    Sleep(1);

    //Set DotClk Frequency
    WriteIndex(0xE6);
    WriteParameter(DotClk_H);   //0x01
    WriteParameter(DotClk_M);
    WriteParameter(DotClk_L);


    ///
    //WriteIndex(0xE6);
    ////WriteParameter(LCDC_FPR&0x0F0000)>>16);
    //WriteParameter(LCDC_FPR&0x00FF00)>>8);
    //WriteParameter(LCDC_FPR&0x0000FF));
    ////


    //Set the LCD panel mode
    WriteIndex(0xB0);
    WriteParameter(Panel_Mode1);         //0x3C
    WriteParameter(Panel_Mode2);           //0x0F
    WriteParameter(HPS>>8);
    WriteParameter(HPS&0xFF);
    WriteParameter(VPS>>8);
    WriteParameter(VPS&0xFF);         /////////
    WriteParameter(Panel_Mode3);     //0


    WriteIndex(0xB4);
    WriteParameter(LCD_HORI_CYCLE>>8);       //0x02
    WriteParameter(LCD_HORI_CYCLE&0xFF);         //0x0D
    WriteParameter(LCD_HORI_FRONT_PORCH>>8);
    WriteParameter(LCD_HORI_FRONT_PORCH&0xFF);     //0x02
    WriteParameter(LCD_HORI_PULSE_WIDTH);   //////////0x29
    WriteParameter(0x00);
    WriteParameter(0x00);   //0xD1
    WriteParameter(0x00);


    WriteIndex(0xB6);
    WriteParameter(LCD_VERT_CYCLE>>8);      //0x01
    WriteParameter(LCD_VERT_CYCLE&0xFF);       //0x1e
    WriteParameter(LCD_VERT_FRONT_PORCH>>8);
    WriteParameter(LCD_VERT_FRONT_PORCH&0xFF);  //0x02
    WriteParameter(LCD_VERT_PULSE_WIDTH);  //0x0A       ///////
    WriteParameter(0x00);
    WriteParameter(0x00);

    WriteIndex(0xBA);
    WriteParameter(0x0F);

    Sleep(100);

    WriteIndex(0x3A);//set display pixel format
    WriteParameter(Display_Pixel_Format<<4);


    WriteIndex(0x2A);    //set coloum address and page address
    WriteParameter(0x00);
    WriteParameter(0x00);
    WriteParameter(HPS>>8);
    WriteParameter(HPS&0xFF);

    WriteIndex(0x2B);
    WriteParameter(0x00);
    WriteParameter(0x00);
    WriteParameter(VPS>>8);
    WriteParameter(VPS&0xFF);

    WriteIndex(0x36);
    WriteParameter(0x00);

    WriteIndex(0xF0);      //Set Pixel Data Interface
    WriteParameter(Pixel_Data_Interface_Format);       //0x04:18bit,0x03:16bit,0x05:24bit


    WriteIndex(0xB8);
    WriteParameter(0x0f);
    WriteParameter(0x01);




    //Buzzer.Beep();

    // Wait for the LCD to become responsive
    // I don't know why it takes so long.
    // The random pixels should mean its ready immediately
    //Sleep(2000);

    // RESET = 1;
    SET_RESET;



    // Wait for it to finish resetting
    Sleep(100 );

    // uchar i;

    // widx(0x36); //set adress mode
    WriteIndex( 0x36 );

    // wpat( 0x00 ); //set page,column,line,RGB adress order;display data latch data,fli horizontal,vertical
    WriteParameter( 0x00 );

    // widx( 0xf0 ); //set pixel data interface
    WriteIndex( 0xF0 );

    // wpat( 0x04 ); //select 18bit
    WriteParameter( 0x04 );

    // widx( 0x29 );  //display on
    //WriteIndex( 0x29 );

    LCD.Clear(FEHLCD::White);
    LCD.PrintImage(111,30);
    LCD.PrintLogo(86,175);
    LCD.SetFontColor(FEHLCD::Black);
    LCD.WriteAt("v02.08.17",1,1);

    WriteIndex(0x29);  //display on
    Sleep(100);

    WriteIndex(0xBE);
    WriteParameter(0x0E);    //(set PWM frequency) PWM signal frequency = PLL clock / (256 * PWMF[7:0]) / 256
    WriteParameter(0xF0);    // (dummy value if DBC is used)
    WriteParameter(0x01);   // (enable PWM controlled by DBC)
    WriteParameter(0x00);
    WriteParameter(0x00);
    WriteParameter(0x0F);

    TS_SPI_Init();


    LCD.SetFontColor(FEHLCD::White);
    LCD.SetBackgroundColor(FEHLCD::Black);

}

FEHLCD::FEHLCD()
{
    _forecolor = Black;
    _backcolor = White;



    _maxlines = 14;
    _maxcols = 26;
    
    _currentline = 0;
    _currentchar = 0;

    _width = 320;
    _height = 240;

    _orientation = North;

    SetRegisterColorValues();
}

void FEHLCD::Initialize()
{
    if( initialized == false )
    {
        initialized = true;

        _Initialize();
    }
}


void FEHLCD::PrintImage(int x, int y)
{
    SetDrawRegion(x,y,98,126);
    int image_length = sizeof(image) / sizeof(image[0]);
    for(int i = 0; i < image_length; i += 2){
        for(int j = 0; j < image[i]; j++){
            unsigned char r, g, b;
            if(image[i + 1] ==0) {
                r = 255; g=255; b=255;
            }
            else if(image[i + 1]==1) {
                r = 212; g=0; b=38;
            }
            else {
                r=181; g= 186; b=176;
            }
            //_forecolor = ConvertRGBColorTo16Bit(image[k][0],image[k][1],image[k][2]);
            _forecolor = ConvertRGBColorTo16Bit(r,g,b);

            SetRegisterColorValues();
            _ForePixel();
        }
    }
}

void FEHLCD::PrintLogo(int x, int y)
{
    SetDrawRegion(x,y,149,39);
    int logo_length = sizeof(logo) / sizeof(logo[0]);
    for(int i = 0; i < logo_length; i += 2){
        for(int j = 0; j < logo[i]; j++){
            unsigned char r, g, b;
            if(logo[i + 1] ==0) {
                r = 255; g=255; b=255;
            }
            else if(logo[i + 1]==1){
                r = 0; g=0; b=0;
            }
            else{
                r = 212; g=0; b=38;
            }
            //_forecolor = ConvertRGBColorTo16Bit(image[k][0],image[k][1],image[k][2]);
            _forecolor = ConvertRGBColorTo16Bit(r,g,b);

            SetRegisterColorValues();
            _ForePixel();
        }
    }

}

void FEHLCD::SetOrientation(FEHLCDOrientation orientation)
{
    _orientation = orientation;
    // widx(0x36); //set adress mode
    WriteIndex( 0x36 );

    // wpat( 0x00 ); //set page,column,line,RGB adress order;display data latch data,fli horizontal,vertical

    switch(_orientation)
    {
    case North: WriteParameter( 0x00); break;
    case South: WriteParameter(0x03); break;
    case East:  WriteParameter(0x05 << 5  | 0x03); break;
    case West:  WriteParameter((0x05 << 5)); break;
    }
    if(_orientation == North || _orientation == South)
    {
        _maxlines = 14;
        _maxcols = 26;

        _width = 320;
        _height = 240;

    }
    else
    {
        _maxlines = 18;
        _maxcols = 19;

        _width = 240;
        _height = 320;
    }
    _Clear();
}


unsigned int FEHLCD::ConvertFEHColorTo24Bit(FEHLCDColor color) {
    unsigned int htmlColor;
    switch(color)
    {
    case White:     htmlColor=0xFFFFFFu; break;
    case Black:     htmlColor=0x000000u; break;
    case Red:       htmlColor=0xFF0000u; break;
    case Green:     htmlColor=0x00FF00u; break;
    case Blue:      htmlColor=0x0000FFu; break;
    case Scarlet:   htmlColor=0x990000u; break;
    case Gray:      htmlColor=0x999999u; break;
    }
    return htmlColor;
}

unsigned int FEHLCD::Convert24BitColorTo16Bit(unsigned int color) {
    unsigned char r = (color & 0xFF0000u) >> 16;
    unsigned char g = (color & 0x00FF00u) >> 8;
    unsigned char b = (color & 0x0000FFu);
    return ConvertRGBColorTo16Bit(r,g,b);
}

unsigned int FEHLCD::ConvertRGBColorTo16Bit(unsigned char r, unsigned char g, unsigned char b) {
    unsigned int ru = r >>2;
    unsigned int gu = g >>2;
    unsigned int bu = b >>2;
    return (ru << 12) | (gu << 6) | bu;
}

void FEHLCD::Clear( FEHLCDColor color ) {
    Clear(ConvertFEHColorTo24Bit(color));
}

void FEHLCD::Clear( unsigned int color ) {
    _backcolor = Convert24BitColorTo16Bit(color);
    SetRegisterColorValues();

    _Clear();
}
void FEHLCD::Clear() {
    _Clear();
}

void FEHLCD::SetBackgroundColor( FEHLCDColor color ) {
    SetBackgroundColor(ConvertFEHColorTo24Bit(color));
}

void FEHLCD::SetBackgroundColor( unsigned int color ) {
    _backcolor = Convert24BitColorTo16Bit(color);
    SetRegisterColorValues();
}

void FEHLCD::SetFontColor( FEHLCDColor color ) {
    SetFontColor(ConvertFEHColorTo24Bit(color));
}

void FEHLCD::SetFontColor( unsigned int color ) {
    _forecolor = Convert24BitColorTo16Bit(color);
    SetRegisterColorValues();
}
void FEHLCD::WriteAt(const char * str, int x, int y)
{
    int i=0;
    while(str[i] != '\0') {
        WriteCharAt(x,y,str[i]);
        x+=CharWidth;
        i++;
    }
}
void FEHLCD::WriteAt(int i, int x, int y)
{
    char num[50];
    sprintf(num,"%d",i);
    WriteAt(num,x,y);
}
void FEHLCD::WriteAt(float f, int x, int y)
{
    char num[50];
    int d,r;
    d = (int) f;
    r = (int) ((f-d)*1000);
    if(f<0.)
        r=r*-1;
    if(f<0. && f>-1.)
        sprintf(num,"-%d.%03d",d,r);
    else
        sprintf(num,"%d.%03d",d,r);
    WriteAt(num,x,y);
}
void FEHLCD::WriteAt(double d, int x, int y)
{
    WriteAt((float)d,x,y);
}
void FEHLCD::WriteAt(bool b, int x, int y)
{
    if(b)
    {
        WriteAt("true",x,y);
    }
    else
    {
        WriteAt("false",x,y);
    }
}
void FEHLCD::WriteAt(char c, int x, int y)
{
    char str[1] = { c };
    WriteAt(str,x,y);
}

void FEHLCD::TS_SPI_Init()
{
    SPI_Init();
    SPI_WriteCommand(0x01,0x67);
    //Sleep(50);
    SPI_WriteCommand(0x02,0x00);
    //Sleep(50);
    SPI_WriteCommand(0x03,0xA0);
    //Sleep(50);
    SPI_WriteCommand(0x04,0x50);
    //Sleep(50);
    SPI_WriteCommand(0x05,0x20);
    //Sleep(50);
    SPI_WriteCommand(0x06,0x20);
    //Sleep(50);
    SPI_WriteCommand(0x07,0x77);
    //Sleep(50);
    SPI_WriteCommand(0x08,0x00);
    //Sleep(50);
    SPI_WriteCommand(0x09,0x00);
    //Sleep(50);
    SPI_WriteCommand(0x0A,0x00);
    //Sleep(50);
    SPI_WriteCommand(0x0B,0x04);
    //Sleep(50);
    lastx = -1;
    lasty = -1;
}

//New Read
bool FEHLCD::Touch(float *x_pos, float *y_pos)
{
    uint8 dat;                                   //Temporary data holder
    uint16 x, y;                                  //Read x and y values
              //For noise rejection

    dat = SPI_ReadCommand(0x00);                  //Read the status

    if(dat & 0x02)                              //Touch present?
    {
        SPI_CS_Assert();  //Command a conversion (x,y)
        SPI_SendChar(0xE0);
        Sleep(2);
        SPI_CS_Deassert();
        Sleep(1);
        SPI_CS_Assert();               //Read the positions
        SPI_SendChar(0xA5);
        Sleep(1);
        x = SPI_GetChar()<<8;
        x |= SPI_GetChar();
        y = SPI_GetChar()<<8;
        y |= SPI_GetChar();
        x >>= 4;
        y >>= 4;
        SPI_CS_Deassert();

        if(lastx==-1 && lasty==-1)
        {
            lastx=x;
            lasty=y;
            y=4095-y;
            *x_pos=x*(320/4095.);
            *y_pos=y*(240/4095.);
        }
        else if(abs(lastx-x)<100 && abs(lasty-y)<100)
        {
            lastx=x;
            lasty=y;
            y=4095-y;
            *x_pos=x*(320/4095.);
            *y_pos=y*(240/4095.);
        }
        else
        {
            *x_pos=-1;
            *y_pos=-1;
            lastx=-1;
            lasty=-1;
            return false;
        }
        return true;
    }
    else
    {
        *x_pos=-1;
        *y_pos=-1;
        lastx=-1;
        lasty=-1;
        return false;
    }



}

int FEHLCD::abs(int no)
{
    if(no<0)
    {
        no*=-1;
    }
    return no;
}

void FEHLCD::WriteRC(const char * str, int row, int col)
{
    int x,y;

    y = row * 17;
    x = col * 12;
    WriteAt( str, x, y );
}
void FEHLCD::WriteRC(int i, int row, int col)
{
    int x,y;

    y = row * 17;
    x = col * 12;
    WriteAt( i, x, y );
}
void FEHLCD::WriteRC(float f, int row, int col)
{
    int x,y;

    y = row * 17;
    x = col * 12;
    WriteAt( f, x, y );
}
void FEHLCD::WriteRC(double d, int row, int col)
{
    int x,y;

    y = row * 17;
    x = col * 12;
    WriteAt( d, x, y );
}
void FEHLCD::WriteRC(bool b, int row, int col)
{
    int x,y;

    y = row * 17;
    x = col * 12;
    WriteAt( b, x, y );
}
void FEHLCD::WriteRC(char c, int row, int col)
{
    int x,y;

    y = row * 17;
    x = col * 12;
    WriteAt( c, x, y );
}

void FEHLCD::Write( const char *str )
{
    int i=0;
    CheckLine();
    while(str[i] != '\0') {
        WriteChar(_currentline,_currentchar,str[i]);
        NextChar();
        i++;
    }
}
void FEHLCD::Write( int i )
{
    char num[50];
    sprintf(num,"%d",i);
    Write(num);
}
void FEHLCD::Write( float f )
{
    char num[50];
    int d,r;
    if( f >= 0 )
    {
        d = (int) f;
        r = (int) ((f-d)*1000);
        sprintf(num,"%d.%03d",d,r);
    }
    else
    {
        f *= -1;
        d = (int) f;
        r = (int) ((f-d)*1000);
        sprintf(num,"-%d.%03d",d,r);
    }
    Write(num);
}
void FEHLCD::Write( double d )
{
    Write((float) d);
}
void FEHLCD::Write( bool b )
{
    if( b )
    {
        Write( "true" );
    }
    else
    {
        Write( "false" );
    }
}
void FEHLCD::Write( char c )
{
    CheckLine();
    WriteChar( _currentline, _currentchar, c );
    NextChar();
}
void FEHLCD::WriteLine( const char* str )
{
    CheckLine();
    Write( str );
    NextLine();
}
void FEHLCD::WriteLine( int i )
{
    CheckLine();
    Write( i );
    NextLine();
}
void FEHLCD::WriteLine( float f )
{
    CheckLine();
    Write( f );
    NextLine();
}
void FEHLCD::WriteLine( double d )
{
    CheckLine();
    Write( d );
    NextLine();
}
void FEHLCD::WriteLine( bool b )
{
    CheckLine();
    Write( b );
    NextLine();
}
void FEHLCD::WriteLine( char c )
{
    CheckLine();
    Write( c );
    NextLine();
}

void FEHLCD::WriteIndex( unsigned char index )
{
    // CS = 0;
    CLR_CS;

    // RS = 0;
    CLR_RS;

    // RD = 1;
    SET_RD;

    // dataport0 = index;
    // clear all bits
    GPIOB_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 9 ) | ( 1 << 8 ) );
    GPIOC_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 4 ) | ( 1 << 5 ) | ( 1 << 6 ) | ( 1 << 7 ) | ( 1 << 12 ) | ( 1 << 13 ) );

    // Set index bits
    GPIOB_PDOR |= GPIO_PDOR_PDO( ( ( ( index & ( 1 << 0 ) ) ? ( 1 ) : ( 0 ) ) << 9 ) );
    GPIOB_PDOR |= GPIO_PDOR_PDO( ( ( ( index & ( 1 << 1 ) ) ? ( 1 ) : ( 0 ) ) << 8 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( index & ( 1 << 2 ) ) ? ( 1 ) : ( 0 ) ) << 4 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( index & ( 1 << 3 ) ) ? ( 1 ) : ( 0 ) ) << 5 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( index & ( 1 << 4 ) ) ? ( 1 ) : ( 0 ) ) << 6 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( index & ( 1 << 5 ) ) ? ( 1 ) : ( 0 ) ) << 7 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( index & ( 1 << 6 ) ) ? ( 1 ) : ( 0 ) ) << 12 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( index & ( 1 << 7 ) ) ? ( 1 ) : ( 0 ) ) << 13 ) );

    // WR = 0;
    CLR_WR;

    // // delay2( 1 )

    // WR = 1;
    SET_WR;

    // CS = 1;
    SET_CS;

    // // dataport0 = 0xff;
    GPIOB_PDOR |= GPIO_PDOR_PDO( ( 1 << 9 ) | ( 1 << 8 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( 1 << 4 ) | ( 1 << 5 ) | ( 1 << 6 ) | ( 1 << 7 ) | ( 1 << 12 ) | ( 1 << 13 ) );

    // RS = 1;
    SET_RS;

    // delay2( 1 );
}

void FEHLCD::WriteParameter( unsigned char param )
{
    // CS = 0;
    CLR_CS;

    // RD = 1;
    SET_RD;

    // RS = 1;
    SET_RS;

    // dataport0 = data1;
    // clear all bits
    GPIOB_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 9 ) | ( 1 << 8 ) );
    GPIOC_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 4 ) | ( 1 << 5 ) | ( 1 << 6 ) | ( 1 << 7 ) | ( 1 << 12 ) | ( 1 << 13 ) );

    // Set index bits
    GPIOB_PDOR |= GPIO_PDOR_PDO( ( ( ( param & ( 1 << 0 ) ) ? ( 1 ) : ( 0 ) ) << 9 ) );
    GPIOB_PDOR |= GPIO_PDOR_PDO( ( ( ( param & ( 1 << 1 ) ) ? ( 1 ) : ( 0 ) ) << 8 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( param & ( 1 << 2 ) ) ? ( 1 ) : ( 0 ) ) << 4 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( param & ( 1 << 3 ) ) ? ( 1 ) : ( 0 ) ) << 5 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( param & ( 1 << 4 ) ) ? ( 1 ) : ( 0 ) ) << 6 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( param & ( 1 << 5 ) ) ? ( 1 ) : ( 0 ) ) << 7 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( param & ( 1 << 6 ) ) ? ( 1 ) : ( 0 ) ) << 12 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( param & ( 1 << 7 ) ) ? ( 1 ) : ( 0 ) ) << 13 ) );

    // WR = 0;
    CLR_WR;

    // // delay2( 1 );

    // WR = 1;
    SET_WR;

    // CS = 1;
    SET_CS;

    // //dataport0 = 0xff;

    // RS = 0;
    CLR_RS;

    // delay2( 1 );
}

void FEHLCD::_Clear()
{
    _currentline = 0;
    _currentchar = 0;

    unsigned int color = _backcolor;

    unsigned char c1 = (unsigned char)( color & 0xFF );
    unsigned char c2 = (unsigned char)( ( color >> 8 ) & 0xFF );
    unsigned char c3 = (unsigned char)( ( color >> 16 ) & 0x03 );

    // widx(0x2a); //set column address
    WriteIndex( 0x2A );

    // wpat(0x00); //start column number high byte
    WriteParameter( 0x00 );

    //wpat(0x00); //start column number low byte
    WriteParameter( 0x00 );

    //wpat(0x01); //end column number high byte
    WriteParameter( 0x01 );

    //wpat(0x3f); //end column number low byte
    WriteParameter( 0x3F );

    //widx(0x2b); //set row adress
    WriteIndex( 0x2b );

    //wpat(0x00); //start row number high byte
    WriteParameter( 0x00 );

    //wpat(0x00); //start row number low byte
    WriteParameter( 0x00 );

    //wpat(0x00); //end row number high byte
    WriteParameter( 0x00 );

    //wpat(0xef); //end row number low byte
    WriteParameter( 0xEF );

    //widx(0x2c);       //enter write mode
    WriteIndex( 0x2C );

    // clear all bits
    GPIOB_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 9 ) | ( 1 << 8 ) );
    GPIOC_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 4 ) | ( 1 << 5 ) | ( 1 << 6 ) | ( 1 << 7 ) | ( 1 << 12 ) | ( 1 << 13 ) | ( 1 << 14 ) | ( 1 << 15 ) | ( 1 << 16 ) | ( 1 << 17 ) | ( 1 << 18 ) | ( 1 << 19 ) );
    GPIOD_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 0 ) | ( 1 << 2 ) | ( 1 << 3 ) | ( 1 << 4 ) );


    GPIOB_PDOR |= backRegisterValues.BVal;
    GPIOC_PDOR |= backRegisterValues.CVal;
    GPIOD_PDOR |= backRegisterValues.DVal;


    // CS = 0;
    CLR_CS;

    // RS = 1;
    SET_RS;

    for( unsigned int i = 0; i < 240; i++ )
    {
        for( unsigned int j = 0; j < 320; j++ )
        {
            // clr_cs, set_rs moved outside of loop

            //set color
            // moved outside of loop

            // WR = 0;
            CLR_WR;

            // WR = 1;
            SET_WR;

            // set_cs, clr_rs moved after loop
        }
    }

    // CS = 1;
    SET_CS;

    // RS = 0;
    CLR_RS;
}

void FEHLCD::NextLine() {
    if(_currentchar > 0)
    {
        _currentline ++;
        _currentchar = 0;
    }
}

void FEHLCD::CheckLine()
{
    if( _currentline >= _maxlines)
    {
        _currentline = 0;
        _Clear();
    }
}

void FEHLCD::NextChar() {
    _currentchar++;
    if(_currentchar == _maxcols) {
        NextLine();
        CheckLine();
    }
}

void FEHLCD::DrawPixel(int x, int y)
{
    SetDrawRegion(x,y,1,1);
    _ForePixel();
}
void Swap(int & a, int &b) {
    int c = a;
    a=b;
    b=c;
}
int abs(int z)
{
    if(z<0)
        return -z;
    else
        return z;
}


void FEHLCD::DrawHorizontalLine(int y, int x1, int x2)
{
    if(x2<x1)
        Swap(x1,x2);
    int length = x2-x1+1;
    SetDrawRegion(x1,y,length,1);

    _ForePixel();
    for(int i=1;i<_width;i++)
        RepeatColor();
}

void FEHLCD::DrawVerticalLine(int x,int y1, int y2)
{
    if(y2<y1)
        Swap(y1,y2);
    int length = y2-y1+1;
    SetDrawRegion(x,y1,1,length);

    _ForePixel();
    for(int i=1;i<length;i++)
        RepeatColor();
}




void FEHLCD::DrawRectangle(int x, int y, int width, int height)
{
    DrawLine(x,y,x+width,y);
    DrawLine(x+width,y,x+width,y+height);
    DrawLine(x+width,y+height,x,y+height);
    DrawLine(x,y+height,x,y);
}

void FEHLCD::SetDrawRegion(int x, int y, int width, int height)
{
    unsigned int pixStartCol, pixEndCol, pixStartRow, pixEndRow;

    if(_orientation == East || _orientation == West)
    {
         pixStartCol = y;
         pixEndCol = pixStartCol+height-1;

         pixEndRow = _width-x;
         pixStartRow  = pixEndRow-(width-1);
    }
    else
    {
        pixStartCol = x;
        pixEndCol   = pixStartCol+width-1;

        pixStartRow = y;
        pixEndRow = pixStartRow+height-1;
    }

    // widx( 0x2a ); //set column adress
    WriteIndex( 0x2A );

    // wpat( 0x00 ); //start column number high byte
    WriteParameter((pixStartCol & 0xf00) >> 8 );

    // wpat( 0x00 ); //start column number low byte
    WriteParameter( pixStartCol & 0xff);

    // wpat( 0x01 ); //end column number high byte
    WriteParameter((pixEndCol & 0x0f00) >> 8 );

    // wpat( 0x3f ); //end column number low byte
    WriteParameter( pixEndCol & 0xff);


    // widx( 0x2b ); //set row address
    WriteIndex( 0x2B );

    // wpat( 0x00 ); //start row number high byte
    WriteParameter((pixStartRow & 0x0f00) >> 8 );

    // wpat( 0x00 ); //start row number low byte
    WriteParameter( pixStartRow & 0xff);

    // wpat( 0x00 ); //end row number high byte
    WriteParameter( (pixEndRow & 0x0f00) >> 8 );

    // wpat( 0xef ); //end row number low byte
    WriteParameter( pixEndRow & 0xff);

    // widx( 0x2c );       //enter write mode
    WriteIndex( 0x2C );
}

void FEHLCD::FillRectangle(int x, int y, int width, int height)
{
    SetDrawRegion(x,y,width,height);
    _ForePixel();
    for(int i=1;i<width*height;i++)
        RepeatColor();

}

void FEHLCD::DrawLine(int x1, int y1, int x2, int y2)
{
    int dx = abs(x2-x1);
    int dy = abs(y2-y1);
    bool steep = (dy > dx);

    // If the line is steep, we'll iterate over y
    // instead of x. To keep code simple, I just
    // swap and always iterate over x
    if(steep)
    {
        Swap(x1,y1);
        Swap(x2,y2);
    }

    // Alwasy iterate from low x to high x
    if(x1>x2)
    {
        Swap(x1,x2);
        Swap(y1,y2);
    }

    // If x2==x1, then the line is just a pixel
    if(x2==x1)
    {
        DrawPixel(x1,y1);
        return;
    }

    float slope = (y2-y1)/((float)(x2-x1));
    float y = y1;

    // Iterate over x
    for(int x=x1;x<=x2;x++)
    {
        // Round the y coordinate, and draw
        // swap back x and y if we swapped them initially
        if(steep)
            DrawPixel((int)(y+.5),x);
        else
            DrawPixel(x,(int)(y+.5));

        y+=slope;
    }
}

void FEHLCD::DrawCircle(int x0, int y0, int radius)
{
    // This alogorithm is from wikipedia
    // It's called the "midpoint circle algorithm"
    // or the "Bresenham's circle algorithm"
    // http://en.wikipedia.org/wiki/Midpoint_circle_algorithm
    // See the page for further details
      int f = 1 - radius;
      int ddF_x = 1;
      int ddF_y = -2 * radius;
      int x = 0;
      int y = radius;

      DrawPixel(x0, y0 + radius);
      DrawPixel(x0, y0 - radius);
      DrawPixel(x0 + radius, y0);
      DrawPixel(x0 - radius, y0);

      while(x < y)
      {
        // ddF_x == 2 * x + 1;
        // ddF_y == -2 * y;
        // f == x*x + y*y - radius*radius + 2*x - y + 1;
        if(f >= 0)
        {
          y--;
          ddF_y += 2;
          f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;
        DrawPixel(x0 + x, y0 + y);
        DrawPixel(x0 - x, y0 + y);
        DrawPixel(x0 + x, y0 - y);
        DrawPixel(x0 - x, y0 - y);
        DrawPixel(x0 + y, y0 + x);
        DrawPixel(x0 - y, y0 + x);
        DrawPixel(x0 + y, y0 - x);
        DrawPixel(x0 - y, y0 - x);
      }
}
void FEHLCD::FillCircle(int x0, int y0, int radius)
{
    // This algorithm is a variant on DrawCircle.
    // Rather than draw the points around the circle,
    // We connect them with a series of lines
    // to fill in the circle.

      int f = 1 - radius;
      int ddF_x = 1;
      int ddF_y = -2 * radius;
      int x = 0;
      int y = radius;


      DrawVerticalLine(x0, y0 - radius, y0 + radius);
      DrawHorizontalLine(y0, x0 - radius, x0 + radius);

      while(x < y)
      {
        // ddF_x == 2 * x + 1;
        // ddF_y == -2 * y;
        // f == x*x + y*y - radius*radius + 2*x - y + 1;
        if(f >= 0)
        {
          y--;
          ddF_y += 2;
          f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;
        DrawHorizontalLine(y0 + x, x0 - y, x0 +y);
        DrawHorizontalLine(y0 - x, x0 - y, x0 +y);
        DrawVerticalLine(x0 + x, y0 - y, y0 +y);
        DrawVerticalLine(x0 - x, y0 - y, y0 +y);
      }
}

void FEHLCD::WriteChar(int charRow, int charCol, char charNum) {
    WriteCharAt(2+charCol*CharWidth,3+charRow*CharHeight, charNum);
}

void FEHLCD::WriteCharAt(int x, int y, char charNum)
{
    if( charNum == '\n' )
    {
        CheckLine();
        NextLine();
        return;
    }

    if(charNum > 125 || charNum <32)
    {
        charNum = 32;
    }
    charNum-=32;

    SetDrawRegion(x,y,CharWidth,CharHeight);

    // Look up the bitmap data out of the fontData table
    // Each entry in the fontData table corresponds
    // to a column of pixels in the 5x7 bitmapped character
    unsigned char charData[5];
    charData[0] = fontData[5*charNum];
    charData[1] = fontData[5*charNum+1];
    charData[2] = fontData[5*charNum+2];
    charData[3] = fontData[5*charNum+3];
    charData[4] = fontData[5*charNum+4];

    // The actual characters here are 12x17. They are double size
    // and have 2 pixels of border on the left, and 3 on the bottom.

    bool pix;
    bool prevPix;


    // Loop through the character data to draw the character
    // prevPix stores whether the previous pixel
    // was a forePixel. This allows the more efficient "Repeat pixel"
    // function when a pixel color is repeated.
    for (int row = 0 ; row <7 ; row++)
    {
        // 2 pixels of left border
        _BackPixel();
        RepeatColor();
        prevPix=false;

        // Loop through the columns of the character
        for(int col = 0; col < 5; col++)
        {
            // get the current pixel from the end of the
            // character data array
            bool pix = charData[col] & 0x01;

            // Draw it efficently using repeat pixel if possible
            if (pix)
            {
                if(!prevPix)
                    _ForePixel();
                else
                    RepeatColor();
                RepeatColor();
            }
            else
            {
                if(prevPix)
                    _BackPixel();
                else
                    RepeatColor();
                RepeatColor();
            }
            prevPix = pix;
            //charData[col] >>= 1;
            //WriteData1( n );
        }

        // Now we're at the next line of the display
        // so we need two more pixels of border
        if(prevPix)
            _BackPixel();
        else
            RepeatColor();

        RepeatColor();
        prevPix = false;

        // Loop through the columns again
        for(int col = 0; col < 5; col++)
        {
            bool pix = charData[col] & 0x01;
            if (pix)
            {
                if(!prevPix)
                    _ForePixel();
                else
                    RepeatColor();
                RepeatColor();
            }
            else
            {
                if(prevPix)
                    _BackPixel();
                else
                    RepeatColor();
                RepeatColor();
            }
            prevPix = pix;
            // Now we're done with this column of the character
            // so we bitshit to get the next pixel for this column
            // queued up for the next time through the loop.
            charData[col] >>= 1;
            //WriteData1( n );
        }
    }


    // Draw the three rows of padding at the bottom
    _BackPixel();
    prevPix = false;
    for(int i=1;i<CharWidth*3; i++)
    {
        RepeatColor();
    }
}

void FEHLCD::RepeatColor()
{
    // RD=1;
    SET_RD;

    // CS=0;
    CLR_CS;

    // RS=1;
    SET_RS;

    // WR=0;
    CLR_WR;

    // WR=1;
    SET_WR;

    // CS=1;
    SET_CS;

    // RS=0;
    CLR_RS;
}

void FEHLCD::_ForePixel(){
    unsigned char c;

    // RD=1;
    SET_RD;

    // CS=0;
    CLR_CS;

    // RS=1;
    SET_RS;

    // clear all bits
    GPIOB_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 9 ) | ( 1 << 8 ) );
    GPIOC_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 4 ) | ( 1 << 5 ) | ( 1 << 6 ) | ( 1 << 7 ) | ( 1 << 12 ) | ( 1 << 13 ) | ( 1 << 14 ) | ( 1 << 15 ) | ( 1 << 16 ) | ( 1 << 17 ) | ( 1 << 18 ) | ( 1 << 19 ) );
    GPIOD_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 0 ) | ( 1 << 2 ) | ( 1 << 3 ) | ( 1 << 4 ) );

    GPIOB_PDOR |= foreRegisterValues.BVal;
    GPIOC_PDOR |= foreRegisterValues.CVal;
    GPIOD_PDOR |= foreRegisterValues.DVal;

    // WR=0;
    CLR_WR;

    // WR=1;
    SET_WR;

    // CS=1;
    SET_CS;

    // RS=0;
    CLR_RS;
}

void FEHLCD::_BackPixel() {
    //_ForePixel();
    //return;
    // RD=1;
    SET_RD;

    // CS=0;
    CLR_CS;

    // RS=1;
    SET_RS;

    // clear all bits
    GPIOB_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 9 ) | ( 1 << 8 ) );
    GPIOC_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 4 ) | ( 1 << 5 ) | ( 1 << 6 ) | ( 1 << 7 ) | ( 1 << 12 ) | ( 1 << 13 ) | ( 1 << 14 ) | ( 1 << 15 ) | ( 1 << 16 ) | ( 1 << 17 ) | ( 1 << 18 ) | ( 1 << 19 ) );
    GPIOD_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 0 ) | ( 1 << 2 ) | ( 1 << 3 ) | ( 1 << 4 ) );


    GPIOB_PDOR |= backRegisterValues.BVal;
    GPIOC_PDOR |= backRegisterValues.CVal;
    GPIOD_PDOR |= backRegisterValues.DVal;


    // WR=0;
    CLR_WR;

    // WR=1;
    SET_WR;

    // CS=1;
    SET_CS;

    // RS=0;
    CLR_RS;
}

void FEHLCD::SetRegisterColorValues()
{
    unsigned int color = _backcolor;

    unsigned char c1 = (unsigned char)( color & 0xFF );
    unsigned char c2 = (unsigned char)( ( color >> 8 ) & 0xFF );
    unsigned char c3 = (unsigned char)( ( color >> 16 ) & 0x03 );




    foreRegisterValues.BVal = 0;
    foreRegisterValues.CVal = 0;
    foreRegisterValues.DVal = 0;

    backRegisterValues.BVal = 0;
    backRegisterValues.CVal = 0;
    backRegisterValues.DVal = 0;


    // dataport0 = sel_color[ i ];

    backRegisterValues.BVal |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 0 ) ) ? ( 1 ) : ( 0 ) ) << 9 ) );
    backRegisterValues.BVal |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 1 ) ) ? ( 1 ) : ( 0 ) ) << 8 ) );
    backRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 2 ) ) ? ( 1 ) : ( 0 ) ) << 4 ) );
    backRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 3 ) ) ? ( 1 ) : ( 0 ) ) << 5 ) );
    backRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 4 ) ) ? ( 1 ) : ( 0 ) ) << 6 ) );
    backRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 5 ) ) ? ( 1 ) : ( 0 ) ) << 7 ) );
    backRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 6 ) ) ? ( 1 ) : ( 0 ) ) << 12 ) );
    backRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 7 ) ) ? ( 1 ) : ( 0 ) ) << 13 ) );

    // dataport2 = sel_color[ i + 1 ];
//    c = sel_color[ i + 1 ];
    backRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 0 ) ) ? ( 1 ) : ( 0 ) ) << 14 ) );
    backRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 1 ) ) ? ( 1 ) : ( 0 ) ) << 15 ) );
    backRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 2 ) ) ? ( 1 ) : ( 0 ) ) << 16 ) );
    backRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 3 ) ) ? ( 1 ) : ( 0 ) ) << 17 ) );
    backRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 4 ) ) ? ( 1 ) : ( 0 ) ) << 18 ) );
    backRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 5 ) ) ? ( 1 ) : ( 0 ) ) << 19 ) );
    backRegisterValues.DVal |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 6 ) ) ? ( 1 ) : ( 0 ) ) << 0 ) );
    backRegisterValues.DVal |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 7 ) ) ? ( 1 ) : ( 0 ) ) << 2 ) );

    // dataport3 = sel_color[ i + 2 ];
//    c = sel_color[ i + 2 ];
    backRegisterValues.DVal |= GPIO_PDOR_PDO( ( ( ( c3 & ( 1 << 0 ) ) ? ( 1 ) : ( 0 ) ) << 3 ) );
    backRegisterValues.DVal |= GPIO_PDOR_PDO( ( ( ( c3 & ( 1 << 1 ) ) ? ( 1 ) : ( 0 ) ) << 4 ) );


    color = _forecolor;

    c1 = (unsigned char)( color & 0xFF );
    c2 = (unsigned char)( ( color >> 8 ) & 0xFF );
    c3 = (unsigned char)( ( color >> 16 ) & 0x03 );


    foreRegisterValues.BVal |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 0 ) ) ? ( 1 ) : ( 0 ) ) << 9 ) );
    foreRegisterValues.BVal |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 1 ) ) ? ( 1 ) : ( 0 ) ) << 8 ) );
    foreRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 2 ) ) ? ( 1 ) : ( 0 ) ) << 4 ) );
    foreRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 3 ) ) ? ( 1 ) : ( 0 ) ) << 5 ) );
    foreRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 4 ) ) ? ( 1 ) : ( 0 ) ) << 6 ) );
    foreRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 5 ) ) ? ( 1 ) : ( 0 ) ) << 7 ) );
    foreRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 6 ) ) ? ( 1 ) : ( 0 ) ) << 12 ) );
    foreRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 7 ) ) ? ( 1 ) : ( 0 ) ) << 13 ) );

    // dataport2 = sel_color[ i + 1 ];
//    c = sel_color[ i + 1 ];
    foreRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 0 ) ) ? ( 1 ) : ( 0 ) ) << 14 ) );
    foreRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 1 ) ) ? ( 1 ) : ( 0 ) ) << 15 ) );
    foreRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 2 ) ) ? ( 1 ) : ( 0 ) ) << 16 ) );
    foreRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 3 ) ) ? ( 1 ) : ( 0 ) ) << 17 ) );
    foreRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 4 ) ) ? ( 1 ) : ( 0 ) ) << 18 ) );
    foreRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 5 ) ) ? ( 1 ) : ( 0 ) ) << 19 ) );
    foreRegisterValues.DVal |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 6 ) ) ? ( 1 ) : ( 0 ) ) << 0 ) );
    foreRegisterValues.DVal |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 7 ) ) ? ( 1 ) : ( 0 ) ) << 2 ) );

    // dataport3 = sel_color[ i + 2 ];
//    c = sel_color[ i + 2 ];
    foreRegisterValues.DVal |= GPIO_PDOR_PDO( ( ( ( c3 & ( 1 << 0 ) ) ? ( 1 ) : ( 0 ) ) << 3 ) );
    foreRegisterValues.DVal |= GPIO_PDOR_PDO( ( ( ( c3 & ( 1 << 1 ) ) ? ( 1 ) : ( 0 ) ) << 4 ) );
}

/* Icon constructor function */
FEHIcon::Icon::Icon() {}

/* Icon function to set position, size, label, and color */
void FEHIcon::Icon::SetProperties(char name[20], int start_x, int start_y, int w, int h, unsigned int c, unsigned int tc)
{
    strcpy(label,name);
    x_start = start_x;
    y_start = start_y;
    width = w;
    height = h;
    x_end = x_start+width;
    y_end = y_start+height;
    color = c;
    textcolor = tc;
    set = 0;
}

/* Icon function to draw it and write label */
void FEHIcon::Icon::Draw()
{
    LCD.SetFontColor(color);
    LCD.DrawRectangle(x_start, y_start, width, height);
    LCD.SetFontColor(textcolor);
    LCD.WriteAt(label,x_start+((width-(strlen(label)*12))/2),y_start+((height-17)/2)); // equation to center text inside the icon
}

/* Icon function to make the icon selected and set */
void FEHIcon::Icon::Select()
{
    LCD.SetFontColor(color);
    LCD.DrawRectangle(x_start+1,y_start+1,width-2,height-2);
    LCD.DrawRectangle(x_start+2,y_start+2,width-4,height-4);
    LCD.DrawRectangle(x_start+3,y_start+3,width-6,height-6);
    set = 1;
}

/* Icon function to make the icon deselected and not set */
void FEHIcon::Icon::Deselect()
{
    LCD.SetFontColor(BLACK);
    LCD.DrawRectangle(x_start+3,y_start+3,width-6,height-6);
    LCD.DrawRectangle(x_start+2,y_start+2,width-4,height-4);
    LCD.DrawRectangle(x_start+1,y_start+1,width-2,height-2);
    set = 0;
}

/* Icon function to see if it has been pressed */
int FEHIcon::Icon::Pressed(float x, float y, int mode)
{
    if (x>=x_start && x<=x_end && y>=y_start && y<=y_end)
    {
        LCD.Touch(&x, &y);
        if (x>=x_start && x<=x_end && y>=y_start && y<=y_end) // check twice to avoid buggy touch screen issues
        {
            if (!mode) // if mode is 0, then alternate selecting and deselecting as it is pressed again and again; otherwise, the icon does not select and deselect
            {
                if (!set)
                {
                    Select();
                }
                else if (set)
                {
                    Deselect();
                }
            }
            return 1;
        }
    }
    else
    {
        return 0;
    }
}

/* Icon function to wait while it is pressed */
int FEHIcon::Icon::WhilePressed(float xi, float yi)
{
    float x = xi, y = yi;
    while(Pressed(x, y, 1))
    {
        LCD.Touch(&x, &y);
    }
}

/* Icon function to change the label of an icon with a string */
void FEHIcon::Icon::ChangeLabelString(const char new_label[])
{
    if (strcmp(label, new_label))
    {
        strcpy(label, new_label);
        LCD.SetFontColor(BLACK);
        LCD.FillRectangle(x_start+1, y_start+1, width-2, height-2);
        Draw();
    }
}

/* Icon function to change the label of an icon with a float */
void FEHIcon::Icon::ChangeLabelFloat(float val)
{
    int length_i = strlen(label);
    int d, r;
    /* Convert float to string so it can be auto-centered in icon */
    if(val>=0)
    {
        d = (int) val;
        r = (int) ((val-d)*1000);
        std::sprintf(label,"%d.%03d",d,r);
    }
    else
    {
        val *= -1;
        d = (int) val;
        r = (int) ((val-d)*1000);
        std::sprintf(label,"-%d.%03d",d,r);
    }
    /* If the new label is not the same length as the old one, then erase the old one so that it does not show up behind the new one */
    if (strlen(label)!=length_i)
    {
        LCD.SetFontColor(BLACK);
        LCD.FillRectangle(x_start+1, y_start+1, width-2, height-2);
    }
    Draw();
}

/* Icon function to change the label of an icon with a int */
void FEHIcon::Icon::ChangeLabelInt(int val)
{
    int length_i = strlen(label);

    /* Convert int to string so it can be auto-centered in icon */
    std::sprintf(label,"%d",val);

    /* If the new label is not the same length as the old one, then erase the old one so that it does not show up behind the new one */
    if (strlen(label)!=length_i)
    {
        LCD.SetFontColor(BLACK);
        LCD.FillRectangle(x_start+1, y_start+1, width-2, height-2);
    }
    Draw();
}

/* Function to draw an array of icons in a given space and size and label them */
void FEHIcon::DrawIconArray(Icon icon[], int rows, int cols, int top, int bot, int left, int right, char labels[][20], unsigned int col, unsigned int txtcol)
{
    int xs = left;
    int ys = top;
    float total_w = (320.-left-right);
    float total_h = (240.-top-bot);
    int w = total_w/cols;
    int h = total_h/rows;
    int nx, ny, N=0;
    for (ny=1; ny<=rows; ny++)
    {
        for (nx=1; nx<=cols; nx++)
        {
            icon[N].SetProperties(labels[N], xs, ys, w, h, col, txtcol);
            icon[N].Draw();
            N = N+1;
            xs = xs+w;
        }
        ys = ys+h;
        xs = left;
    }
}
