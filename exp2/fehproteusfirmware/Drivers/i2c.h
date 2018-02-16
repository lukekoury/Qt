/*
 * File:        k60_i2c.h
 * Purpose:     I2C header
 *
 * Notes:
 *
 */
#ifndef I2C_H
#define I2C_H

#include "derivative.h"

#define MMA8451Q_I2C_ADDRESS                         0x1C
#define i2c_DisableAck()       I2C0_C1 |= I2C_C1_TXAK_MASK

#define i2c_RepeatedStart()    I2C0_C1     |= 0x04;

#define i2c_Start()            I2C0_C1     |= 0x10;\
                               I2C0_C1     |= I2C_C1_MST_MASK

#define i2c_Stop()             I2C0_C1  &= ~I2C_C1_MST_MASK;\
                               I2C0_C1  &= ~I2C_C1_TX_MASK

#define i2c_EnterRxMode()      I2C0_C1   &= ~I2C_C1_TX_MASK;\
                               I2C0_C1   &= ~I2C_C1_TXAK_MASK

#define i2c_Wait()               while((I2C0_S & I2C_S_IICIF_MASK)==0) {} \
                                  I2C0_S |= I2C_S_IICIF_MASK;

#define i2c_write_byte(data)   I2C0_D = data

#define MWSR                   0x00  /* Master write  */
#define MRSW                   0x01  /* Master read */


void init_I2C(void);
void IIC_StartTransmission (unsigned char SlaveID, unsigned char Mode);
void MMA8451QWriteRegister(unsigned char u8RegisterAddress, unsigned char u8Data);
unsigned char u8MMA8451QReadRegister(unsigned char u8RegisterAddress);

#endif // FEHACCEL_H
