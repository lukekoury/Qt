#include "spi.h"


/** @brief Initializes the SPI port
*
*	@return void
*/


void SPI_SendChar(uint8 send)
{
    SPI_Transfer(send);
}

void SPI_WriteCommand(uint8 cmd, uint8 dat)
{
    SPI_CS_Assert();
    SPI_SendChar((cmd<<1)|0x00);
    SPI_SendChar(dat);
    SPI_CS_Deassert();
}

uint8 SPI_GetChar(void)
{
    return SPI_Transfer(0xFF);
}

uint8 SPI_ReadCommand(uint8 cmd)
{
    uint8 dat;                       //Read data

    SPI_CS_Assert();
    SPI_SendChar((cmd<<1)|0x01);     //Write the command (7-bits shifted)
    dat = SPI_GetChar();             //Read the data
                    //Reset the CS
    SPI_CS_Deassert();
    return dat;
}

void SPI_CS_Assert()
{
    GPIOB_PCOR |= (1<<20);
}

void SPI_CS_Deassert()
{
    GPIOB_PSOR |= (1<<20);
}

uint8 SPI_Transfer(uint8 send)
{
    uint8 dummy;
    uint8 buff = 0;

    while ((SPI2_SR & SPI_SR_TCF_MASK)==1) // wait until transmit buffer is empty
    {
        dummy++;
    }

    SPI2_SR |= SPI_SR_TCF_MASK; // clear status register action
    SPI2_PUSHR = send | SPI_PUSHR_CTAS(0); // write out to data register (0xff to read - i.e. keeps MOSI line high)

    while(!(SPI2_SR & SPI_SR_RFDF_MASK)); // wait until read buffer full
    buff=SPI2_POPR; // read data register
    return buff; // return data register value
}

void SPI_Init()
{
    //SIM_PINSEL1 |= SIM_PINSEL1_SPI2PS_MASK; // Select proper pins for SPI
    //SIM_SCGC |=  SIM_SCGC_SPI2_MASK;		// Select proper pins for SPI
    SIM_SCGC3 |= SIM_SCGC3_SPI2_MASK;

    PORTB_BASE_PTR->PCR[20]=PORT_PCR_MUX(1);
    GPIOB_PDDR |= (1<<20);
    SPI_CS_Deassert();

    PORTB_BASE_PTR->PCR[21]=PORT_PCR_MUX(2);
    PORTB_BASE_PTR->PCR[22]=PORT_PCR_MUX(2);
    PORTB_BASE_PTR->PCR[23]=PORT_PCR_MUX(2);

    SPI2_MCR = SPI_MCR_MSTR_MASK | SPI_MCR_CLR_RXF_MASK | SPI_MCR_CLR_TXF_MASK | SPI_MCR_PCSIS(0) ;				// Enable SPI
    SPI2_CTAR0 = SPI_CTAR_FMSZ(0x7) | SPI_CTAR_BR(0x5) |SPI_CTAR_CPHA_MASK|SPI_CTAR_CPOL_MASK;//8

    for(int i=0;i<10;i++)
        SPI_SendChar(0xff); //get the clock all warmed up


    Sleep(.1);
}
    



