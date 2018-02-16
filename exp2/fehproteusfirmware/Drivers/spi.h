#ifndef SPI_H
#define SPI_H

#include "fehproteusfirmware/Startup/derivative.h"
#include "FEHUtility.h"

void SPI_Init();

void SPI_WriteCommand(uint8 cmd, uint8 dat);

void SPI_SendChar(uint8 send);

uint8 SPI_GetChar(void);

uint8 SPI_Transfer(uint8 send);


uint8 SPI_ReadCommand(uint8 cmd);

void SPI_CS_Assert(void);

void SPI_CS_Deassert(void);


#endif // SPI_H
