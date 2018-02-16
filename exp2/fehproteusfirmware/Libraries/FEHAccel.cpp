#include "FEHAccel.h"


#define MMA8451_I2C_ADDRESS = 0x1C
FEHAccel Accel;

FEHAccel::FEHAccel(){
	SIM_SCGC4 |= SIM_SCGC4_I2C0_MASK; //Turn on clock to I2C0 module

    /* configure GPIO for I2C0 function */
    PORTD_PCR9 = PORT_PCR_MUX(2);
    PORTD_PCR8 = PORT_PCR_MUX(2);

    I2C0_F  = 0x14;       /* set MULT and ICR */

    I2C0_C1 = I2C_C1_IICEN_MASK;       /* enable IIC */
    MMA8451QWriteRegister(0x2B,0x02);
    MMA8451QWriteRegister(0x2A,0x3D);
}

double FEHAccel::X(){
    unsigned char x1;
    unsigned char x2;
    short _x;

    //Read x-axis register
    x1 = u8MMA8451QReadRegister(0x01);
    x2 = u8MMA8451QReadRegister(0x02);

    _x = ((short) (x1<<8 | x2 ))>>2;

    return(_x/(8.*518.5)*(-1));
}

double FEHAccel::Y(){
    unsigned char y1;
    unsigned char y2;
    short _y;

    //Read x-axis register
    y1 = u8MMA8451QReadRegister(0x03);
    y2 = u8MMA8451QReadRegister(0x04);

    _y = ((short) (y1<<8 | y2 ))>>2;

    return(_y/(8.*518.5)*(-1));
}

double FEHAccel::Z(){
    unsigned char z1;
    unsigned char z2;
    short _z;

    //Read x-axis register
    z1 = u8MMA8451QReadRegister(0x05);
    z2 = u8MMA8451QReadRegister(0x06);

    _z = ((short) (z1<<8 | z2 ))>>2;

    return(_z/(8.*518.5)*(-1));
}


