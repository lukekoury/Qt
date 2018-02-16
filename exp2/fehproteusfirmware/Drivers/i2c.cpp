/*
 * File:        k60_i2c.c
 * Purpose:     Code for initializing and using I2C
 *
 * Notes:
 *
 */

//#include "common.h"
#include "i2c.h"



unsigned char MasterTransmission;
unsigned char SlaveID;
//*******************************************************************/
//*!
// * I2C Initialization
// * Set Baud Rate and turn on I2C0
// */
//void init_I2C(void)
//{
//    SIM_SCGC4 |= SIM_SCGC4_I2C0_MASK; //Turn on clock to I2C0 module

//    /* configure GPIO for I2C0 function */
//    PORTD_PCR9 = PORT_PCR_MUX(2);
//    PORTD_PCR8 = PORT_PCR_MUX(2);

//    I2C0_F  = 0x14;       /* set MULT and ICR */

//    I2C0_C1 = I2C_C1_IICEN_MASK;       /* enable IIC */
//}


/*******************************************************************/
/*!
 * Start I2C Transmision
 * @param SlaveID is the 7 bit Slave Address
 * @param Mode sets Read or Write Mode
 */
void IIC_StartTransmission (unsigned char SlaveID, unsigned char Mode)
{
  if(Mode == MWSR)
  {
    /* set transmission mode */
    MasterTransmission = MWSR;
  }
  else
  {
    /* set transmission mode */
    MasterTransmission = MRSW;
  }

  /* shift ID in right possition */
  SlaveID = (unsigned char) MMA8451Q_I2C_ADDRESS << 1;

  /* Set R/W bit at end of Slave Address */
  SlaveID |= (unsigned char)MasterTransmission;

  /* send start signal */
  i2c_Start();

  /* send ID with W/R bit */
  i2c_write_byte(SlaveID);
}

/*******************************************************************/
/*!
 * Pause Routine
 */
void Pause(void){
    int n;
    for(n=1;n<50;n++) {
      asm("nop");
    }
}

/*******************************************************************/
/*!
 * Read a register from the MPR084
 * @param u8RegisterAddress is Register Address
 * @return Data stored in Register
 */
unsigned char u8MMA8451QReadRegister(unsigned char u8RegisterAddress)
{
  unsigned char result;
  unsigned int j;

  /* Send Slave Address */
  IIC_StartTransmission(SlaveID,MWSR);
  i2c_Wait();

  /* Write Register Address */
  I2C0_D = u8RegisterAddress;
  i2c_Wait();

  /* Do a repeated start */
  I2C0_C1 |= I2C_C1_RSTA_MASK;

  /* Send Slave Address */
  I2C0_D = (MMA8451Q_I2C_ADDRESS << 1) | 0x01; //read address
  i2c_Wait();

  /* Put in Rx Mode */
  I2C0_C1 &= (~I2C_C1_TX_MASK);

  /* Turn off ACK */
  I2C0_C1 |= I2C_C1_TXAK_MASK;

  /* Dummy read */
  result = I2C0_D ;
  for (j=0; j<5000; j++){};
  i2c_Wait();

  /* Send stop */
  i2c_Stop();
  result = I2C0_D ;
  Pause();
  return result;
}

/*******************************************************************/
/*!
 * Write a byte of Data to specified register on MPR084
 * @param u8RegisterAddress is Register Address
 * @param u8Data is Data to write
 */
void MMA8451QWriteRegister(unsigned char u8RegisterAddress, unsigned char u8Data)
{
  /* send data to slave */
  IIC_StartTransmission(SlaveID,MWSR);
  i2c_Wait();

  I2C0_D = u8RegisterAddress;
  i2c_Wait();

  I2C0_D = u8Data;
  i2c_Wait();

  i2c_Stop();

  Pause();
}
