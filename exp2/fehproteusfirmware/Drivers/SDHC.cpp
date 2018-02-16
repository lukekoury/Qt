/*
 * SDHC.c
 *
 *  Created on: 16/05/2011
 *      Author: gustavo
 */

#include "SDHC.h"
//#include "small_time.c"

#define NULL ((void *)0)
#define FALSE (DRESULT)0
#define TRUE 1

SDCARD_STRUCT 		SDHC_Card;
SDCARD_INIT_STRUCT  SDHC_Init;
ESDHC_INFO_STRUCT	SDHC_Info;
ESDHC_DEVICE_STRUCT SDHC_Device;

static volatile DSTATUS   Stat             = STA_NOINIT;	/* Disk status */
static volatile uint32	  Timer            = 0;           	/* Read/Write timer */


const ESDHC_INIT_STRUCT K60_SDHC0_init = {
    0,                          /* ESDHC device number */ 
    25000000,                   /* ESDHC baudrate      */
    BSP_SYSTEM_CLOCK            /* ESDHC clock source  */ 
};

//uint32 get_fattime (void)
//{
//  OS_RTC rtc;
//
//  GetCalendar(&rtc);
//
//  return    ((uint32)(rtc.Year - 1980) << 25)
//          | ((uint32)rtc.Month << 21)
//          | ((uint32)rtc.Day   << 16)
//          | ((uint32)rtc.Hour  << 11)
//          | ((uint32)rtc.Min << 5)
//          | ((uint32)rtc.Sec << 1);
//}


static const unsigned long ESDHC_COMMAND_XFERTYP[] = 
{   /* CMD0 */
    SDHC_XFERTYP_CMDINX(ESDHC_CMD0) | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_NO),
    SDHC_XFERTYP_CMDINX(ESDHC_CMD1) | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_NO),
    SDHC_XFERTYP_CMDINX(ESDHC_CMD2) | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_136),
    SDHC_XFERTYP_CMDINX(ESDHC_CMD3) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48),
    SDHC_XFERTYP_CMDINX(ESDHC_CMD4) | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_NO),
    /* CMD5 */
    SDHC_XFERTYP_CMDINX(ESDHC_CMD5) | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48),
    SDHC_XFERTYP_CMDINX(ESDHC_CMD6) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48),
    SDHC_XFERTYP_CMDINX(ESDHC_CMD7) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48BUSY),
    SDHC_XFERTYP_CMDINX(ESDHC_CMD8) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48),
    SDHC_XFERTYP_CMDINX(ESDHC_CMD9) | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_136),
    /* CMD10 */
    SDHC_XFERTYP_CMDINX(ESDHC_CMD10) | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_136),
    SDHC_XFERTYP_CMDINX(ESDHC_CMD11) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48),
    SDHC_XFERTYP_CMDINX(ESDHC_CMD12) | SDHC_XFERTYP_CMDTYP(ESDHC_XFERTYP_CMDTYP_ABORT) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48BUSY),
    SDHC_XFERTYP_CMDINX(ESDHC_CMD13) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48),
    0,
    /* CMD15 */
    SDHC_XFERTYP_CMDINX(ESDHC_CMD15) | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_NO),
    SDHC_XFERTYP_CMDINX(ESDHC_CMD16) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48),
    SDHC_XFERTYP_CMDINX(ESDHC_CMD17) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48),
    SDHC_XFERTYP_CMDINX(ESDHC_CMD18) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48),
    0,
    /* CMD20 */
    SDHC_XFERTYP_CMDINX(ESDHC_CMD20) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48),
    0,
    SDHC_XFERTYP_CMDINX(ESDHC_ACMD22) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48),
    SDHC_XFERTYP_CMDINX(ESDHC_ACMD23) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48),
    SDHC_XFERTYP_CMDINX(ESDHC_CMD24) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48),
    /* CMD25 */
    SDHC_XFERTYP_CMDINX(ESDHC_CMD25) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48),
    SDHC_XFERTYP_CMDINX(ESDHC_CMD26) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48),
    SDHC_XFERTYP_CMDINX(ESDHC_CMD27) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48),
    SDHC_XFERTYP_CMDINX(ESDHC_CMD28) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48BUSY),
    SDHC_XFERTYP_CMDINX(ESDHC_CMD29) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48BUSY),
    /* CMD30 */
    SDHC_XFERTYP_CMDINX(ESDHC_CMD30) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48),
    0,
    SDHC_XFERTYP_CMDINX(ESDHC_CMD32) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48),
    SDHC_XFERTYP_CMDINX(ESDHC_CMD33) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48),
    SDHC_XFERTYP_CMDINX(ESDHC_CMD34) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48),
    /* CMD35 */
    SDHC_XFERTYP_CMDINX(ESDHC_CMD35) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48),
    SDHC_XFERTYP_CMDINX(ESDHC_CMD36) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48),
    SDHC_XFERTYP_CMDINX(ESDHC_CMD37) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48),
    SDHC_XFERTYP_CMDINX(ESDHC_CMD38) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48BUSY),
    SDHC_XFERTYP_CMDINX(ESDHC_CMD39) | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48),
    /* CMD40 */
    SDHC_XFERTYP_CMDINX(ESDHC_CMD40) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48),
    SDHC_XFERTYP_CMDINX(ESDHC_ACMD41) | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48),
    SDHC_XFERTYP_CMDINX(ESDHC_CMD42) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48BUSY),
    0,
    0,
    /* CMD45 */
    0,
    0,
    0,
    0,
    0,
    /* CMD50 */
    0,
    SDHC_XFERTYP_CMDINX(ESDHC_ACMD51) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48),
    SDHC_XFERTYP_CMDINX(ESDHC_CMD52) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48),
    SDHC_XFERTYP_CMDINX(ESDHC_CMD53) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48),
    0,
    /* CMD55 */
    SDHC_XFERTYP_CMDINX(ESDHC_CMD55) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48),
    SDHC_XFERTYP_CMDINX(ESDHC_CMD56) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48BUSY),
    0,
    0,
    0,
    /* CMD60 */
    SDHC_XFERTYP_CMDINX(ESDHC_CMD60) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48BUSY),
    SDHC_XFERTYP_CMDINX(ESDHC_CMD61) | SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK | SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48BUSY),
    0,
    0
};


static void SDHC_set_baudrate 
(       
        /* [IN] Module input clock in Hz */
        uint32         clock,
        
        /* [IN] Desired baudrate in Hz */
        uint32         baudrate
) 
{
    uint32 pres, div, min, minpres = 0x80, mindiv = 0x0F;
    int32  val;

    /* Find closest setting */
    min = (uint32)-1;
    for (pres = 2; pres <= 256; pres <<= 1) 
    {
        for (div = 1; div <= 16; div++) 
        {
            val = pres * div * baudrate - clock;
            if (val >= 0)
            {
                if (min > val) 
                {
                    min = val;
                    minpres = pres;
                    mindiv = div;
                }
            }
        }
    }

    /* Disable ESDHC clocks */
    SDHC_BASE_PTR->SYSCTL &= (~ SDHC_SYSCTL_SDCLKEN_MASK);

    /* Change dividers */
    div = SDHC_BASE_PTR->SYSCTL & (~ (SDHC_SYSCTL_DTOCV_MASK | SDHC_SYSCTL_SDCLKFS_MASK | SDHC_SYSCTL_DVS_MASK));
    SDHC_BASE_PTR->SYSCTL = div | (SDHC_SYSCTL_DTOCV(0x0E) | SDHC_SYSCTL_SDCLKFS(minpres >> 1) | SDHC_SYSCTL_DVS(mindiv - 1));

    /* Wait for stable clock */
    while (0 == (SDHC_BASE_PTR->PRSSTAT & SDHC_PRSSTAT_SDSTB_MASK))
    {
        /* Workaround... */
        Sleep(10);
    };

    /* Enable ESDHC clocks */
    SDHC_BASE_PTR->SYSCTL |= SDHC_SYSCTL_SDCLKEN_MASK;
    SDHC_BASE_PTR->IRQSTAT |= SDHC_IRQSTAT_DTOE_MASK;
}


uint32 SDHC_init
			(
			/* [IN/OUT] Device runtime information */
			ESDHC_INFO_STRUCT_PTR  esdhc_info_ptr,

			/* [IN] Device initialization data */
			ESDHC_INIT_STRUCT_CPTR esdhc_init_ptr
			)					
{
	esdhc_info_ptr->CARD = ESDHC_CARD_NONE;
	
	/* Reset ESDHC */
    SDHC_BASE_PTR->SYSCTL = SDHC_SYSCTL_RSTA_MASK | SDHC_SYSCTL_SDCLKFS(0x80);
    while (SDHC_BASE_PTR->SYSCTL & SDHC_SYSCTL_RSTA_MASK){};
    
    /* Initial values */
    SDHC_BASE_PTR->VENDOR = 0;
    SDHC_BASE_PTR->BLKATTR = SDHC_BLKATTR_BLKCNT(1) | SDHC_BLKATTR_BLKSIZE(512);
    SDHC_BASE_PTR->PROCTL = SDHC_PROCTL_EMODE(ESDHC_PROCTL_EMODE_INVARIANT) | SDHC_PROCTL_D3CD_MASK;
    SDHC_BASE_PTR->WML = SDHC_WML_RDWML(1) | SDHC_WML_WRWML(1);
    
    /* Set the ESDHC initial baud rate divider and start */
    SDHC_set_baudrate (esdhc_init_ptr->CLOCK_SPEED,380000);
    
    /* Poll inhibit bits */
    while (SDHC_BASE_PTR->PRSSTAT & (SDHC_PRSSTAT_CIHB_MASK | SDHC_PRSSTAT_CDIHB_MASK)){};

    /* Init GPIO again */
    PORTE_BASE_PTR->PCR[0] = 0xFFFF & (PORT_PCR_MUX(4) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_DSE_MASK);    /* ESDHC.D1  */
    PORTE_BASE_PTR->PCR[1] = 0xFFFF & (PORT_PCR_MUX(4) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_DSE_MASK);    /* ESDHC.D0  */
    PORTE_BASE_PTR->PCR[2] = 0xFFFF & (PORT_PCR_MUX(4) | PORT_PCR_DSE_MASK);                                          /* ESDHC.CLK */
    PORTE_BASE_PTR->PCR[3] = 0xFFFF & (PORT_PCR_MUX(4) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_DSE_MASK);    /* ESDHC.CMD */
    PORTE_BASE_PTR->PCR[4] = 0xFFFF & (PORT_PCR_MUX(4) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_DSE_MASK);    /* ESDHC.D3  */
    PORTE_BASE_PTR->PCR[5] = 0xFFFF & (PORT_PCR_MUX(4) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_DSE_MASK);    /* ESDHC.D2  */

    /* Enable clock gate to SDHC module */
    SIM_BASE_PTR->SCGC3 |= SIM_SCGC3_SDHC_MASK;
    
    /* Enable requests */
    SDHC_BASE_PTR->IRQSTAT = 0xFFFF;
    SDHC_BASE_PTR->IRQSTATEN = 	  SDHC_IRQSTATEN_DEBESEN_MASK | SDHC_IRQSTATEN_DCESEN_MASK | SDHC_IRQSTATEN_DTOESEN_MASK
                         | SDHC_IRQSTATEN_CIESEN_MASK | SDHC_IRQSTATEN_CEBESEN_MASK | SDHC_IRQSTATEN_CCESEN_MASK | SDHC_IRQSTATEN_CTOESEN_MASK 
                         | SDHC_IRQSTATEN_BRRSEN_MASK | SDHC_IRQSTATEN_BWRSEN_MASK | SDHC_IRQSTATEN_CRMSEN_MASK
                         | SDHC_IRQSTATEN_TCSEN_MASK | SDHC_IRQSTATEN_CCSEN_MASK;
    
    /* 80 initial clocks */
    SDHC_BASE_PTR->SYSCTL |= SDHC_SYSCTL_INITA_MASK;
    while (SDHC_BASE_PTR->SYSCTL & SDHC_SYSCTL_INITA_MASK){};

    /* Check card */
    if (SDHC_BASE_PTR->PRSSTAT & SDHC_PRSSTAT_CINS_MASK)
    {
        esdhc_info_ptr->CARD = ESDHC_CARD_UNKNOWN;
    }
    SDHC_BASE_PTR->IRQSTAT |= SDHC_IRQSTAT_CRM_MASK;
    
    return ESDHC_OK;
}

/*FUNCTION****************************************************************
* 
* Function Name    : _esdhc_is_running
* Returned Value   : TRUE if running, FALSE otherwise
* Comments         :
*    Checks whether eSDHC module is currently in use.
*
*END*********************************************************************/
static uint8 SDHC_is_running(void)
{
    return (0 != (SDHC_BASE_PTR->PRSSTAT & (SDHC_PRSSTAT_RTA_MASK | SDHC_PRSSTAT_WTA_MASK | SDHC_PRSSTAT_DLA_MASK | SDHC_PRSSTAT_CDIHB_MASK | SDHC_PRSSTAT_CIHB_MASK)));
}   

/*FUNCTION****************************************************************
* 
* Function Name    : SDHC_status_wait
* Returned Value   : bits set for given mask
* Comments         :
*    Waits for ESDHC interrupt status register bits according to given mask.
*
*END*********************************************************************/
static uint32 SDHC_status_wait(uint32	mask)        /* [IN] Mask of IRQSTAT bits to wait for */
{
    uint32	result;
    do
    {
        result = SDHC_BASE_PTR->IRQSTAT & mask;
    } 
    while (0 == result);
    return result;
}   

/*FUNCTION****************************************************************
* 
* Function Name    : SDHC_send_command
* Returned Value   : 0 on success, 1 on error, -1 on timeout
* Comments         :
*    One ESDHC command transaction.
*
*END*********************************************************************/
static uint32 SDHC_send_command (ESDHC_COMMAND_STRUCT_PTR command) /* [IN/OUT] Command specification */
{
    uint32	xfertyp;
    
    /* Check command */
    xfertyp = ESDHC_COMMAND_XFERTYP[command->COMMAND & 0x3F];
    if ((0 == xfertyp) && (0 != command->COMMAND))
    {
        return 1;
    }

    /* Card removal check preparation */
    SDHC_BASE_PTR->IRQSTAT |= SDHC_IRQSTAT_CRM_MASK;

    /* Wait for cmd line idle */
    while (SDHC_BASE_PTR->PRSSTAT & SDHC_PRSSTAT_CIHB_MASK){};

    /* Setup command */
    SDHC_BASE_PTR->CMDARG = command->ARGUMENT;
    xfertyp &= (~ SDHC_XFERTYP_CMDTYP_MASK);
    xfertyp |= SDHC_XFERTYP_CMDTYP(command->TYPE);
    if (ESDHC_TYPE_RESUME == command->TYPE)
    {
        xfertyp |= SDHC_XFERTYP_DPSEL_MASK;
    }
    if (ESDHC_TYPE_SWITCH_BUSY == command->TYPE)
    {
        if ((xfertyp & SDHC_XFERTYP_RSPTYP_MASK) == SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48))
        {
            xfertyp &= (~ SDHC_XFERTYP_RSPTYP_MASK);
            xfertyp |= SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48BUSY);
        }
        else
        {
            xfertyp &= (~ SDHC_XFERTYP_RSPTYP_MASK);
            xfertyp |= SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48);
        }
    }
    SDHC_BASE_PTR->BLKATTR &= (~ SDHC_BLKATTR_BLKCNT_MASK);
    if (0 != command->BLOCKS)
    {
        if ((xfertyp & SDHC_XFERTYP_RSPTYP_MASK) != SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_48BUSY))
        {
            xfertyp |= SDHC_XFERTYP_DPSEL_MASK;
        }
        if (command->READ)
        {
            xfertyp |= SDHC_XFERTYP_DTDSEL_MASK;    
        }
        if (command->BLOCKS > 1)
        {
            xfertyp |= SDHC_XFERTYP_MSBSEL_MASK;    
        }
        if ((uint32)-1 != command->BLOCKS)
        {
            SDHC_BASE_PTR->BLKATTR |= SDHC_BLKATTR_BLKCNT(command->BLOCKS);
            xfertyp |= SDHC_XFERTYP_BCEN_MASK;
        }
    }

    /* Issue command */
    SDHC_BASE_PTR->DSADDR = 0;
    SDHC_BASE_PTR->XFERTYP = xfertyp;
    
    /* Wait for response */
    if (SDHC_status_wait (SDHC_IRQSTAT_CIE_MASK | SDHC_IRQSTAT_CEBE_MASK | SDHC_IRQSTAT_CCE_MASK | SDHC_IRQSTAT_CC_MASK) != SDHC_IRQSTAT_CC_MASK)
    {
        SDHC_BASE_PTR->IRQSTAT |= SDHC_IRQSTAT_CTOE_MASK | SDHC_IRQSTAT_CIE_MASK | SDHC_IRQSTAT_CEBE_MASK | SDHC_IRQSTAT_CCE_MASK | SDHC_IRQSTAT_CC_MASK;
        return 1;
    }

    /* Check card removal */
    if (SDHC_BASE_PTR->IRQSTAT & SDHC_IRQSTAT_CRM_MASK)
    {
        SDHC_BASE_PTR->IRQSTAT |= SDHC_IRQSTAT_CTOE_MASK | SDHC_IRQSTAT_CC_MASK;
        return 1;
    }

    /* Get response, if available */
    if (SDHC_BASE_PTR->IRQSTAT & SDHC_IRQSTAT_CTOE_MASK)
    {
        SDHC_BASE_PTR->IRQSTAT |= SDHC_IRQSTAT_CTOE_MASK | SDHC_IRQSTAT_CC_MASK;
        return -1;
    }
    if ((xfertyp & SDHC_XFERTYP_RSPTYP_MASK) != SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_NO))
    {
        command->RESPONSE[0] = SDHC_BASE_PTR->CMDRSP[0];
        if ((xfertyp & SDHC_XFERTYP_RSPTYP_MASK) == SDHC_XFERTYP_RSPTYP(ESDHC_XFERTYP_RSPTYP_136))
        {
            command->RESPONSE[1] = SDHC_BASE_PTR->CMDRSP[1];
            command->RESPONSE[2] = SDHC_BASE_PTR->CMDRSP[2];
            command->RESPONSE[3] = SDHC_BASE_PTR->CMDRSP[3];
        }
    }
    
    SDHC_BASE_PTR->IRQSTAT |= SDHC_IRQSTAT_CC_MASK;

    return 0;
}


/*FUNCTION****************************************************************
* 
* Function Name    : _esdhc_ioctl
* Returned Value   : MQX error code
* Comments         : 
*    This function performs miscellaneous services for the ESDHC I/O device.  
*
*END*********************************************************************/
int32 SDHC_ioctl
		(
        /* [IN] The command to perform */
        uint32              cmd,
        
        /* [IN/OUT] Parameters for the command */
        void                *param_ptr
        )
{
    ESDHC_INFO_STRUCT_PTR   esdhc_info_ptr;
    ESDHC_DEVICE_STRUCT_PTR esdhc_device_ptr;
    ESDHC_INIT_STRUCT_CPTR  esdhc_init_ptr;
    ESDHC_COMMAND_STRUCT    command;
    uint8                   mem, io, mmc, ceata, mp, hc;
    uint32                  val;
    uint32                  result = ESDHC_OK;
    uint32             	    *param32_ptr = (uint32*)param_ptr;

    /* Check parameters */
    esdhc_info_ptr = (ESDHC_INFO_STRUCT_PTR)&SDHC_Info;
    
    if (NULL == esdhc_info_ptr)
    {
        return IO_DEVICE_DOES_NOT_EXIST;
    }
    
    esdhc_device_ptr = &SDHC_Device;
    if (NULL == esdhc_device_ptr)
    {
        return IO_ERROR_DEVICE_INVALID;
    }
    
    esdhc_init_ptr = esdhc_device_ptr->INIT;
    if (NULL == esdhc_init_ptr)
    {
        return IO_ERROR_DEVICE_INVALID;
    }    
    
    switch (cmd) 
    {
        case IO_IOCTL_ESDHC_INIT:

            result = SDHC_init (esdhc_info_ptr, &K60_SDHC0_init);
            if (ESDHC_OK != result)
            {
                break;
            }
            
            mem = FALSE;
            io = FALSE;
            mmc = FALSE;
            ceata = FALSE;
            hc = FALSE;
            mp = FALSE;

            /* CMD0 - Go to idle - reset card */
            command.COMMAND = ESDHC_CMD0;
            command.TYPE = ESDHC_TYPE_NORMAL;
            command.ARGUMENT = 0;
            command.READ = FALSE;
            command.BLOCKS = 0;
            if (SDHC_send_command (&command))
            {
                result = ESDHC_ERROR_INIT_FAILED;
                break;
            }
            
            Sleep(1100);

            /* CMD8 - Send interface condition - check HC support */
            command.COMMAND = ESDHC_CMD8;
            command.TYPE = ESDHC_TYPE_NORMAL;
            command.ARGUMENT = 0x000001AA;
            command.READ = FALSE;
            command.BLOCKS = 0;
            val = SDHC_send_command (&command);
            
            if (val == 0)
            {
                // SDHC Card
            	if (command.RESPONSE[0] != command.ARGUMENT)
                {
                    result = ESDHC_ERROR_INIT_FAILED;
                    break;
                }
                hc = TRUE;
            }

            mp = TRUE;
            
            if (mp)
            {
                /* CMD55 - Application specific command - check MMC */
                command.COMMAND = ESDHC_CMD55;
                command.TYPE = ESDHC_TYPE_NORMAL;
                command.ARGUMENT = 0;
                command.READ = FALSE;
                command.BLOCKS = 0;
                val = SDHC_send_command (&command);
                if (val > 0)
                {
                    result = ESDHC_ERROR_INIT_FAILED;
                    break;
                }
                if (val < 0)
                {
                    /* MMC or CE-ATA */
                    io = FALSE;
                    mem = FALSE;
                    hc = FALSE;
                    
                    /* CMD1 - Send operating conditions - check HC */
                    command.COMMAND = ESDHC_CMD1;
                    command.TYPE = ESDHC_TYPE_NORMAL;
                    command.ARGUMENT = 0x40300000;
                    command.READ = FALSE;
                    command.BLOCKS = 0;
                    if (SDHC_send_command (&command))
                    {
                        result = ESDHC_ERROR_INIT_FAILED;
                        break;
                    }
                    if (0x20000000 == (command.RESPONSE[0] & 0x60000000))
                    {
                        hc = TRUE;
                    }
                    mmc = TRUE;

                    /* CMD39 - Fast IO - check CE-ATA signature CE */
                    command.COMMAND = ESDHC_CMD39;
                    command.TYPE = ESDHC_TYPE_NORMAL;
                    command.ARGUMENT = 0x0C00;
                    command.READ = FALSE;
                    command.BLOCKS = 0;
                    if (SDHC_send_command (&command))
                    {
                        result = ESDHC_ERROR_INIT_FAILED;
                        break;
                    }
                    if (0xCE == (command.RESPONSE[0] >> 8) & 0xFF)
                    {
                        /* CMD39 - Fast IO - check CE-ATA signature AA */
                        command.COMMAND = ESDHC_CMD39;
                        command.TYPE = ESDHC_TYPE_NORMAL;
                        command.ARGUMENT = 0x0D00;
                        command.READ = FALSE;
                        command.BLOCKS = 0;
                        if (SDHC_send_command (&command))
                        {
                            result = ESDHC_ERROR_INIT_FAILED;
                            break;
                        }
                        if (0xAA == (command.RESPONSE[0] >> 8) & 0xFF)
                        {
                            mmc = FALSE;
                            ceata = TRUE;
                        }
                    }
                }
                else
                {
                    /* SD */
                    /* ACMD41 - Send Operating Conditions */
                    command.COMMAND = ESDHC_ACMD41;
                    command.TYPE = ESDHC_TYPE_NORMAL;
                    command.ARGUMENT = 0;
                    command.READ = FALSE;
                    command.BLOCKS = 0;
                    if (SDHC_send_command (&command))
                    {
                        result = ESDHC_ERROR_INIT_FAILED;
                        break;
                    }
                    if (command.RESPONSE[0] & 0x300000)
                    {
                        val = 0;
                        do 
                        {
                            Sleep(BSP_ALARM_RESOLUTION);
                            val++;
                            
                            /* CMD55 + ACMD41 - Send OCR */
                            command.COMMAND = ESDHC_CMD55;
                            command.TYPE = ESDHC_TYPE_NORMAL;
                            command.ARGUMENT = 0;
                            command.READ = FALSE;
                            command.BLOCKS = 0;
                            if (SDHC_send_command (&command))
                            {
                                result = ESDHC_ERROR_INIT_FAILED;
                                break;
                            }

                            command.COMMAND = ESDHC_ACMD41;
                            command.TYPE = ESDHC_TYPE_NORMAL;
                            if (hc)
                            {
                                command.ARGUMENT = 0x40300000;
                            }
                            else
                            {
                                command.ARGUMENT = 0x00300000;
                            }
                            command.READ = FALSE;
                            command.BLOCKS = 0;
                            if (SDHC_send_command (&command))
                            {
                                result = ESDHC_ERROR_INIT_FAILED;
                                break;
                            }
                        } while ((0 == (command.RESPONSE[0] & 0x80000000)) && (val < BSP_ALARM_FREQUENCY));
                        if (ESDHC_OK != result)
                        {
                            break;
                        }
                        if (val >= BSP_ALARM_FREQUENCY)
                        {
                            hc = FALSE;
                        }
                        else
                        {
                            mem = TRUE;
                            if (hc)
                            {
                                hc = FALSE;
                                if (command.RESPONSE[0] & 0x40000000)
                                {
                                    hc = TRUE;
                                }
                            }
                        }
                    }
                }
            }
            
            
            if (mmc)
            {
                esdhc_info_ptr->CARD = ESDHC_CARD_MMC;
            }
            if (ceata)
            {
                esdhc_info_ptr->CARD = ESDHC_CARD_CEATA;
            }
            if (io)
            {
                esdhc_info_ptr->CARD = ESDHC_CARD_SDIO;
            }
            if (mem)
            {
                esdhc_info_ptr->CARD = ESDHC_CARD_SD;
                if (hc)
                {
                    esdhc_info_ptr->CARD = ESDHC_CARD_SDHC;
                }
            }
            if (io && mem)
            {
                esdhc_info_ptr->CARD = ESDHC_CARD_SDCOMBO;
                if (hc)
                {
                    esdhc_info_ptr->CARD = ESDHC_CARD_SDHCCOMBO;
                }
            }

            /* De-Init GPIO */
            PORTE_BASE_PTR->PCR[0] = 0 & (PORT_PCR_MUX(4) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_DSE_MASK);    /* ESDHC.D1  */
            PORTE_BASE_PTR->PCR[1] = 0 & (PORT_PCR_MUX(4) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_DSE_MASK);    /* ESDHC.D0  */
            PORTE_BASE_PTR->PCR[2] = 0 & (PORT_PCR_MUX(4) | PORT_PCR_DSE_MASK);                                          /* ESDHC.CLK */
            PORTE_BASE_PTR->PCR[3] = 0 & (PORT_PCR_MUX(4) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_DSE_MASK);    /* ESDHC.CMD */
            PORTE_BASE_PTR->PCR[4] = 0 & (PORT_PCR_MUX(4) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_DSE_MASK);    /* ESDHC.D3  */
            PORTE_BASE_PTR->PCR[5] = 0 & (PORT_PCR_MUX(4) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_DSE_MASK);    /* ESDHC.D2  */

            /* Set the ESDHC default baud rate */
            SDHC_set_baudrate (esdhc_init_ptr->CLOCK_SPEED, esdhc_init_ptr->BAUD_RATE);

            /* Init GPIO again */
            PORTE_BASE_PTR->PCR[0] = 0xFFFF & (PORT_PCR_MUX(4) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_DSE_MASK);    /* ESDHC.D1  */
            PORTE_BASE_PTR->PCR[1] = 0xFFFF & (PORT_PCR_MUX(4) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_DSE_MASK);    /* ESDHC.D0  */
            PORTE_BASE_PTR->PCR[2] = 0xFFFF & (PORT_PCR_MUX(4) | PORT_PCR_DSE_MASK);                                          /* ESDHC.CLK */
            PORTE_BASE_PTR->PCR[3] = 0xFFFF & (PORT_PCR_MUX(4) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_DSE_MASK);    /* ESDHC.CMD */
            PORTE_BASE_PTR->PCR[4] = 0xFFFF & (PORT_PCR_MUX(4) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_DSE_MASK);    /* ESDHC.D3  */
            PORTE_BASE_PTR->PCR[5] = 0xFFFF & (PORT_PCR_MUX(4) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_DSE_MASK);    /* ESDHC.D2  */

            /* Enable clock gate to SDHC module */
            SIM_BASE_PTR->SCGC3 |= SIM_SCGC3_SDHC_MASK;
            
            break;
        case IO_IOCTL_ESDHC_SEND_COMMAND:
            val = SDHC_send_command ((ESDHC_COMMAND_STRUCT_PTR)param32_ptr);
            if (val > 0)
            {
                result = ESDHC_ERROR_COMMAND_FAILED;
            }
            if (val < 0)
            {
                result = ESDHC_ERROR_COMMAND_TIMEOUT;
            }
            break;
        case IO_IOCTL_ESDHC_GET_BAUDRATE:
            if (NULL == param32_ptr) 
            {
                result = BRTOS_INVALID_PARAMETER;
            } 
            else 
            {
                /* Get actual baudrate */
                val = ((SDHC_BASE_PTR->SYSCTL & SDHC_SYSCTL_SDCLKFS_MASK) >> SDHC_SYSCTL_SDCLKFS_SHIFT) << 1;
                val *= ((SDHC_BASE_PTR->SYSCTL & SDHC_SYSCTL_DVS_MASK) >> SDHC_SYSCTL_DVS_SHIFT) + 1;
                *param32_ptr = (uint32)((esdhc_init_ptr->CLOCK_SPEED) / val);
            }
            break;
        case IO_IOCTL_ESDHC_SET_BAUDRATE:
            if (NULL == param32_ptr) 
            {
                result = BRTOS_INVALID_PARAMETER;
            } 
            else if (0 == (*param32_ptr)) 
            {
                result = BRTOS_INVALID_PARAMETER;
            } 
            else 
            {
                if (! SDHC_is_running())
                {
                    /* De-Init GPIO */
                    PORTE_BASE_PTR->PCR[0] = 0 & (PORT_PCR_MUX(4) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_DSE_MASK);    /* ESDHC.D1  */
                    PORTE_BASE_PTR->PCR[1] = 0 & (PORT_PCR_MUX(4) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_DSE_MASK);    /* ESDHC.D0  */
                    PORTE_BASE_PTR->PCR[2] = 0 & (PORT_PCR_MUX(4) | PORT_PCR_DSE_MASK);                                          /* ESDHC.CLK */
                    PORTE_BASE_PTR->PCR[3] = 0 & (PORT_PCR_MUX(4) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_DSE_MASK);    /* ESDHC.CMD */
                    PORTE_BASE_PTR->PCR[4] = 0 & (PORT_PCR_MUX(4) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_DSE_MASK);    /* ESDHC.D3  */
                    PORTE_BASE_PTR->PCR[5] = 0 & (PORT_PCR_MUX(4) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_DSE_MASK);    /* ESDHC.D2  */

                    /* Set closest baudrate */
                    SDHC_set_baudrate (esdhc_init_ptr->CLOCK_SPEED, *param32_ptr);

                    /* Init GPIO again */
                    PORTE_BASE_PTR->PCR[0] = 0xFFFF & (PORT_PCR_MUX(4) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_DSE_MASK);    /* ESDHC.D1  */
                    PORTE_BASE_PTR->PCR[1] = 0xFFFF & (PORT_PCR_MUX(4) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_DSE_MASK);    /* ESDHC.D0  */
                    PORTE_BASE_PTR->PCR[2] = 0xFFFF & (PORT_PCR_MUX(4) | PORT_PCR_DSE_MASK);                                          /* ESDHC.CLK */
                    PORTE_BASE_PTR->PCR[3] = 0xFFFF & (PORT_PCR_MUX(4) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_DSE_MASK);    /* ESDHC.CMD */
                    PORTE_BASE_PTR->PCR[4] = 0xFFFF & (PORT_PCR_MUX(4) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_DSE_MASK);    /* ESDHC.D3  */
                    PORTE_BASE_PTR->PCR[5] = 0xFFFF & (PORT_PCR_MUX(4) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_DSE_MASK);    /* ESDHC.D2  */

                    /* Enable clock gate to SDHC module */
                    SIM_BASE_PTR->SCGC3 |= SIM_SCGC3_SDHC_MASK;
                }
                else
                {
                    result = IO_ERROR_DEVICE_BUSY;
                }
            }
            break;
        case IO_IOCTL_ESDHC_GET_BLOCK_SIZE:
            if (NULL == param32_ptr) 
            {
                result = BRTOS_INVALID_PARAMETER;
            } 
            else 
            {
                /* Get actual ESDHC block size */
                *param32_ptr = (SDHC_BASE_PTR->BLKATTR & SDHC_BLKATTR_BLKSIZE_MASK) >> SDHC_BLKATTR_BLKSIZE_SHIFT;
            }       
            break;
        case IO_IOCTL_ESDHC_SET_BLOCK_SIZE:
            if (NULL == param32_ptr) 
            {
                result = BRTOS_INVALID_PARAMETER;
            } 
            else 
            {
                /* Set actual ESDHC block size */
                if (! SDHC_is_running())
                {
                    if (*param32_ptr > 0x0FFF)
                    {
                        result = BRTOS_INVALID_PARAMETER;
                    }
                    else
                    {
                        SDHC_BASE_PTR->BLKATTR &= (~ SDHC_BLKATTR_BLKSIZE_MASK);
                        SDHC_BASE_PTR->BLKATTR |= SDHC_BLKATTR_BLKSIZE(*param32_ptr);
                    }
                }
                else
                {
                    result = IO_ERROR_DEVICE_BUSY;
                }
            }       
            break;
        case IO_IOCTL_ESDHC_GET_BUS_WIDTH:
            if (NULL == param32_ptr) 
            {
                result = BRTOS_INVALID_PARAMETER;
            } 
            else 
            {
                /* Get actual ESDHC bus width */
                val = (SDHC_BASE_PTR->PROCTL & SDHC_PROCTL_DTW_MASK) >> SDHC_PROCTL_DTW_SHIFT;
                if (ESDHC_PROCTL_DTW_1BIT == val)
                {
                    *param32_ptr = ESDHC_BUS_WIDTH_1BIT;
                }
                else if (ESDHC_PROCTL_DTW_4BIT == val)
                {
                    *param32_ptr = ESDHC_BUS_WIDTH_4BIT;
                }
                else if (ESDHC_PROCTL_DTW_8BIT == val)
                {
                    *param32_ptr = ESDHC_BUS_WIDTH_8BIT;
                }
                else
                {
                    result = ESDHC_ERROR_INVALID_BUS_WIDTH; 
                }
            }       
            break;
        case IO_IOCTL_ESDHC_SET_BUS_WIDTH:
            if (NULL == param32_ptr) 
            {
                result = BRTOS_INVALID_PARAMETER;
            } 
            else 
            {
                /* Set actual ESDHC bus width */
                if (! SDHC_is_running())
                {
                    if (ESDHC_BUS_WIDTH_1BIT == *param32_ptr)
                    {
                        SDHC_BASE_PTR->PROCTL &= (~ SDHC_PROCTL_DTW_MASK);
                        SDHC_BASE_PTR->PROCTL |= SDHC_PROCTL_DTW(ESDHC_PROCTL_DTW_1BIT);
                    }
                    else if (ESDHC_BUS_WIDTH_4BIT == *param32_ptr)
                    {
                        SDHC_BASE_PTR->PROCTL &= (~ SDHC_PROCTL_DTW_MASK);
                        SDHC_BASE_PTR->PROCTL |= SDHC_PROCTL_DTW(ESDHC_PROCTL_DTW_4BIT);
                    }
                    else if (ESDHC_BUS_WIDTH_8BIT == *param32_ptr)
                    {
                        SDHC_BASE_PTR->PROCTL &= (~ SDHC_PROCTL_DTW_MASK);
                        SDHC_BASE_PTR->PROCTL |= SDHC_PROCTL_DTW(ESDHC_PROCTL_DTW_8BIT);
                    }
                    else
                    {
                        result = ESDHC_ERROR_INVALID_BUS_WIDTH; 
                    }
                }
                else
                {
                    result = IO_ERROR_DEVICE_BUSY;
                }
            }       
            break;
        case IO_IOCTL_ESDHC_GET_CARD:
            if (NULL == param32_ptr) 
            {
                result = BRTOS_INVALID_PARAMETER;
            } 
            else 
            {
                /* 80 clocks to update levels */
                SDHC_BASE_PTR->SYSCTL |= SDHC_SYSCTL_INITA_MASK;
                while (SDHC_BASE_PTR->SYSCTL & SDHC_SYSCTL_INITA_MASK)
                    { };
                    
                /* Update and return actual card status */
                if (SDHC_BASE_PTR->IRQSTAT & SDHC_IRQSTAT_CRM_MASK)
                {
                    SDHC_BASE_PTR->IRQSTAT |= SDHC_IRQSTAT_CRM_MASK;
                    esdhc_info_ptr->CARD = ESDHC_CARD_NONE;
                }
                if (SDHC_BASE_PTR->PRSSTAT & SDHC_PRSSTAT_CINS_MASK)
                {
                    if (ESDHC_CARD_NONE == esdhc_info_ptr->CARD)
                    {
                        esdhc_info_ptr->CARD = ESDHC_CARD_UNKNOWN;
                    }
                }
                else
                {
                    esdhc_info_ptr->CARD = ESDHC_CARD_NONE;
                }
                *param32_ptr = esdhc_info_ptr->CARD;
            }
            break;
        case IO_IOCTL_DEVICE_IDENTIFY:
            /* Get ESDHC device parameters */
            param32_ptr[IO_IOCTL_ID_PHY_ELEMENT]  = IO_DEV_TYPE_PHYS_ESDHC;
            param32_ptr[IO_IOCTL_ID_LOG_ELEMENT]  = IO_DEV_TYPE_LOGICAL_MFS;
            param32_ptr[IO_IOCTL_ID_ATTR_ELEMENT] = IO_ESDHC_ATTRIBS;
            /*
            if (esdhc_fd_ptr->FLAGS & IO_O_RDONLY)
            {
                param32_ptr[IO_IOCTL_ID_ATTR_ELEMENT] &= (~ IO_DEV_ATTR_WRITE);
            }
            */
            break;
        case IO_IOCTL_FLUSH_OUTPUT:
            /* Wait for transfer complete */
            SDHC_status_wait (SDHC_IRQSTAT_TC_MASK);
            if (SDHC_BASE_PTR->IRQSTAT & (SDHC_IRQSTAT_DEBE_MASK | SDHC_IRQSTAT_DCE_MASK | SDHC_IRQSTAT_DTOE_MASK))
            {
                SDHC_BASE_PTR->IRQSTAT |= SDHC_IRQSTAT_DEBE_MASK | SDHC_IRQSTAT_DCE_MASK | SDHC_IRQSTAT_DTOE_MASK;
                result = ESDHC_ERROR_DATA_TRANSFER;
            }
            SDHC_BASE_PTR->IRQSTAT |= SDHC_IRQSTAT_TC_MASK | SDHC_IRQSTAT_BRR_MASK | SDHC_IRQSTAT_BWR_MASK;
            break;
        default:
            result = IO_ERROR_INVALID_IOCTL_CMD;
            break;
    }
    return result;
}



uint8 disk_initialize (unsigned char drv)
{
    uint32              		param, c_size, c_size_mult, read_bl_len;
    ESDHC_COMMAND_STRUCT 		command;
    ESDHC_DEVICE_STRUCT_PTR     esdhc_device_ptr = &SDHC_Device;
    SDCARD_STRUCT_PTR    		sdcard_ptr = (SDCARD_STRUCT_PTR)&SDHC_Card;
    sdcard_ptr->INIT = 			&SDHC_Init;


    
	if (drv) return STA_NOINIT;			/* Supports only single drive */
	if (Stat & STA_NODISK) return Stat;	/* No card in the socket */
	if ((Stat & 0x03) == 0)	return 0;


    
    /* Check parameters */
    if ((NULL == sdcard_ptr) || (NULL == sdcard_ptr->INIT))
    {

        return FALSE;
    }
    
    esdhc_device_ptr->INIT = &K60_SDHC0_init;
    // Indicates one SDHC open
    esdhc_device_ptr->COUNT = 1;

    sdcard_ptr->SD_TIMEOUT = 0;
    sdcard_ptr->NUM_BLOCKS = 0;
    sdcard_ptr->ADDRESS = 0;
    sdcard_ptr->SDHC = FALSE;
    sdcard_ptr->VERSION2 = FALSE;
    
    /* Enable clock gate to SDHC module */
    SIM_BASE_PTR->SCGC3 |= SIM_SCGC3_SDHC_MASK;
    
    /* Initialize and detect card */
    if (ESDHC_OK != SDHC_ioctl (IO_IOCTL_ESDHC_INIT, NULL))
    {

        return FALSE;
    }

    /* SDHC check */
    param = 0;
    if (ESDHC_OK != SDHC_ioctl (IO_IOCTL_ESDHC_GET_CARD, &param))
    {

        return FALSE;
    }
    if ((ESDHC_CARD_SD == param) || (ESDHC_CARD_SDHC == param) || (ESDHC_CARD_SDCOMBO == param) || (ESDHC_CARD_SDHCCOMBO == param))
    {
        if ((ESDHC_CARD_SDHC == param) || (ESDHC_CARD_SDHCCOMBO == param))
        {
            sdcard_ptr->SDHC = TRUE;
        }
    }
    else
    {

        return FALSE;
    }

    /* Card identify */
    command.COMMAND = ESDHC_CMD2;
    command.TYPE = ESDHC_TYPE_NORMAL;
    command.ARGUMENT = 0;
    command.READ = FALSE;
    command.BLOCKS = 0;
    if (ESDHC_OK != SDHC_ioctl (IO_IOCTL_ESDHC_SEND_COMMAND, &command))
    {

        return FALSE;
    }

    /* Get card address */
    command.COMMAND = ESDHC_CMD3;
    command.TYPE = ESDHC_TYPE_NORMAL;
    command.ARGUMENT = 0;
    command.READ = FALSE;
    command.BLOCKS = 0;
    if (ESDHC_OK != SDHC_ioctl (IO_IOCTL_ESDHC_SEND_COMMAND, &command))
    {

        return FALSE;
    }
    sdcard_ptr->ADDRESS = command.RESPONSE[0] & 0xFFFF0000;
    
    /* Get card parameters */
    command.COMMAND = ESDHC_CMD9;
    command.TYPE = ESDHC_TYPE_NORMAL;
    command.ARGUMENT = sdcard_ptr->ADDRESS;
    command.READ = FALSE;
    command.BLOCKS = 0;
    if (ESDHC_OK != SDHC_ioctl (IO_IOCTL_ESDHC_SEND_COMMAND, &command))
    {

        return FALSE;
    }
    if (0 == (command.RESPONSE[3] & 0x00C00000))
    {
        read_bl_len = (command.RESPONSE[2] >> 8) & 0x0F;
        c_size = command.RESPONSE[2] & 0x03;
        c_size = (c_size << 10) | (command.RESPONSE[1] >> 22);
        c_size_mult = (command.RESPONSE[1] >> 7) & 0x07;
        sdcard_ptr->NUM_BLOCKS = (c_size + 1) * (1 << (c_size_mult + 2)) * (1 << (read_bl_len - 9));
    }
    else
    {
        sdcard_ptr->VERSION2 = TRUE;
        c_size = (command.RESPONSE[1] >> 8) & 0x003FFFFF;
        sdcard_ptr->NUM_BLOCKS = (c_size + 1) << 10;
    }

    /* Select card */
    command.COMMAND = ESDHC_CMD7;
    command.TYPE = ESDHC_TYPE_NORMAL;
    command.ARGUMENT = sdcard_ptr->ADDRESS;
    command.READ = FALSE;
    command.BLOCKS = 0;
    if (ESDHC_OK != SDHC_ioctl (IO_IOCTL_ESDHC_SEND_COMMAND, &command))
    {

        return FALSE;
    }

    /* Set block size */
    command.COMMAND = ESDHC_CMD16;
    command.TYPE = ESDHC_TYPE_NORMAL;
    command.ARGUMENT = IO_SDCARD_BLOCK_SIZE;
    command.READ = FALSE;
    command.BLOCKS = 0;
    if (ESDHC_OK != SDHC_ioctl (IO_IOCTL_ESDHC_SEND_COMMAND, &command))
    {

        return FALSE;
    }

    if (ESDHC_BUS_WIDTH_4BIT == sdcard_ptr->INIT->SIGNALS)
    {
        /* Application specific command */
        command.COMMAND = ESDHC_CMD55;
        command.TYPE = ESDHC_TYPE_NORMAL;
        command.ARGUMENT = sdcard_ptr->ADDRESS;
        command.READ = FALSE;
        command.BLOCKS = 0;
        if (ESDHC_OK != SDHC_ioctl (IO_IOCTL_ESDHC_SEND_COMMAND, &command))
        {

            return FALSE;
        }

        /* Set bus width == 4 */
        command.COMMAND = ESDHC_ACMD6;
        command.TYPE = ESDHC_TYPE_NORMAL;
        command.ARGUMENT = 2;
        command.READ = FALSE;
        command.BLOCKS = 0;
        if (ESDHC_OK != SDHC_ioctl (IO_IOCTL_ESDHC_SEND_COMMAND, &command))
        {

            return FALSE;
        }

        param = ESDHC_BUS_WIDTH_4BIT;
        if (ESDHC_OK != SDHC_ioctl (IO_IOCTL_ESDHC_SET_BUS_WIDTH, &param))
        {

            return FALSE;
        }
    }

    Stat &= ~STA_NOINIT;		/* Clear STA_NOINIT */
    

    return (Stat & 0x03);
}



/*-----------------------------------------------------------------------*/
/* Receive a data packet from MMC                                        */
/*-----------------------------------------------------------------------*/

static int rcvr_datablock (
    uint8   *buff,			/* Data buffer to store received data */
    uint32  btr				/* Byte count (must be multiple of 4) */
)
{
    uint32	bytes, i, j;
    uint32	*ptr = (uint32*)buff;
    ESDHC_INFO_STRUCT_PTR   esdhc_info_ptr;

    /* Check parameters */
    esdhc_info_ptr = (ESDHC_INFO_STRUCT_PTR)&SDHC_Info;
    
    /* Check parameters */    
    if (NULL == esdhc_info_ptr)
    {
        return 0;
    }    
    
    /* Workaround for random bit polling failures - not suitable for IO cards */
    if ((esdhc_info_ptr->CARD == ESDHC_CARD_SD) || (esdhc_info_ptr->CARD == ESDHC_CARD_SDHC) || (esdhc_info_ptr->CARD == ESDHC_CARD_MMC) || (esdhc_info_ptr->CARD == ESDHC_CARD_CEATA))
    {
        while (SDHC_BASE_PTR->PRSSTAT & SDHC_PRSSTAT_DLA_MASK){};
    }    
    
    /* Read data in 4 byte counts */
    bytes = btr;
    while (bytes)
    {
        i = bytes > 512 ? 512 : bytes;
        for (j = (i + 3) >> 2; j != 0; j--)
        {
            if (SDHC_BASE_PTR->IRQSTAT & (SDHC_IRQSTAT_DEBE_MASK | SDHC_IRQSTAT_DCE_MASK | SDHC_IRQSTAT_DTOE_MASK))
            {
                SDHC_BASE_PTR->IRQSTAT |= SDHC_IRQSTAT_DEBE_MASK | SDHC_IRQSTAT_DCE_MASK | SDHC_IRQSTAT_DTOE_MASK | SDHC_IRQSTAT_BRR_MASK;
                return 0;
            }
            
            while (0 == (SDHC_BASE_PTR->PRSSTAT & SDHC_PRSSTAT_BREN_MASK)){};
    
#if BRTOS_ENDIAN == BRTOS_LITTLE_ENDIAN
            *ptr++ = SDHC_BASE_PTR->DATPORT;
#else
            *ptr++ = _psp_swap4byte (SDHC_DATPORT);
#endif
        }
        bytes -= i;
    }
    
	return 1;						/* Return with success */    
}

/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
    uint8  drv,			/* Physical drive nmuber (0) */
    uint8  *buff,		/* Pointer to the data buffer to store read data */
    uint32 sector,		/* Start sector number (LBA) */
    uint8  count			/* Sector count (1..255) */
)
{
    ESDHC_COMMAND_STRUCT command;
    SDCARD_STRUCT_PTR    sdcard_ptr = (SDCARD_STRUCT_PTR)&SDHC_Card;
	
	if (drv || !count) return RES_PARERR;
	if (Stat & STA_NOINIT) return RES_NOTRDY;
	
    /* Check parameters */
    if ((NULL == buff))
    {
        return FALSE;
    }	

    if (!sdcard_ptr->SDHC)
    {
    	sector *= 512;	/* Convert to byte address if needed */
    }

	if (count == 1)	/* Single block read */
	{
		command.COMMAND = ESDHC_CMD17;
		command.TYPE = ESDHC_TYPE_NORMAL;
		command.ARGUMENT = sector;
		command.READ = TRUE;
		command.BLOCKS = count;	
		
	    if (ESDHC_OK == SDHC_ioctl (IO_IOCTL_ESDHC_SEND_COMMAND, &command))
	    {
	    	if (rcvr_datablock(buff, 512))
	    	{
	    		count = 0;
	    	}
	    }
	}
	else 
	{	/* Multiple block read */
		// Não sei se é 17 ou 18 no ESDHC
		command.COMMAND = ESDHC_CMD18;
		//command.COMMAND = ESDHC_CMD17;
		command.TYPE = ESDHC_TYPE_NORMAL;
		command.ARGUMENT = sector;
		command.READ = TRUE;
		command.BLOCKS = count;		
		
		if (ESDHC_OK == SDHC_ioctl (IO_IOCTL_ESDHC_SEND_COMMAND, &command))
		{
			if (rcvr_datablock(buff, 512*count))
			{
				count = 0;
			}
		}
	}

	return count ? RES_ERROR : RES_OK;
}

/*-----------------------------------------------------------------------*/
/* Send a data packet to MMC                                             */
/*-----------------------------------------------------------------------*/

static int xmit_datablock (
    const uint8 *buff,			/* 512 byte data block to be transmitted */
    uint32 btr				/* Byte count (must be multiple of 4) */
)
{
    uint32  bytes, i;
    uint32	*ptr = (uint32*)buff;

    /* Write data in 4 byte counts */
    bytes = btr;
    while (bytes)
    {
        i = bytes > 512 ? 512 : bytes;
        bytes -= i;
        for (i = (i + 3) >> 2; i != 0; i--)
        {
            if (SDHC_BASE_PTR->IRQSTAT & (SDHC_IRQSTAT_DEBE_MASK | SDHC_IRQSTAT_DCE_MASK | SDHC_IRQSTAT_DTOE_MASK))
            {
                SDHC_BASE_PTR->IRQSTAT |= SDHC_IRQSTAT_DEBE_MASK | SDHC_IRQSTAT_DCE_MASK | SDHC_IRQSTAT_DTOE_MASK | SDHC_IRQSTAT_BWR_MASK;
                return IO_ERROR;
            }
            while (0 == (SDHC_BASE_PTR->PRSSTAT & SDHC_PRSSTAT_BWEN_MASK)){};

#if PSP_ENDIAN == BRTOS_LITTLE_ENDIAN
            SDHC_BASE_PTR->DATPORT = *ptr++;
#else
            SDHC_DATPORT = _psp_swap4byte (*ptr++);
#endif

        }
    }	


	return 1;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_write (
    uint8  drv,			/* Physical drive nmuber (0) */
    const uint8  *buff,		/* Pointer to the data buffer to store read data */
    uint32 sector,		/* Start sector number (LBA) */
    uint8  count		/* Sector count (1..255) */
)
{
    ESDHC_COMMAND_STRUCT command;
    SDCARD_STRUCT_PTR    sdcard_ptr = (SDCARD_STRUCT_PTR)&SDHC_Card;
	
	if (drv || !count) return RES_PARERR;
	if (Stat & STA_NOINIT) return RES_NOTRDY;
	if (Stat & STA_PROTECT) return RES_WRPRT;
	
    /* Check parameters */
    if ((NULL == buff))
    {
        return FALSE;
    }	

    if (!sdcard_ptr->SDHC)
    {
    	sector *= 512;	/* Convert to byte address if needed */
    }

	if (count == 1)	/* Single block write */
	{
	    command.COMMAND = ESDHC_CMD24;
	    command.TYPE = ESDHC_TYPE_NORMAL;
	    command.ARGUMENT = sector;
	    command.READ = FALSE;
	    command.BLOCKS = count;
	
	    if (ESDHC_OK == SDHC_ioctl (IO_IOCTL_ESDHC_SEND_COMMAND, &command))
	    {
	    	if (xmit_datablock(buff,512))
	    	{
	    		count = 0;
	    	}
	    }
	}
	else 
	{
		//if (CardType & CT_SDC) send_cmd(ACMD23, count);
		//if (send_cmd(CMD25, sector) == 0) {	/* WRITE_MULTIPLE_BLOCK */
		command.COMMAND = ESDHC_CMD25;
	    command.TYPE = ESDHC_TYPE_NORMAL;
	    command.ARGUMENT = sector;
	    command.READ = FALSE;
	    command.BLOCKS = count;
	    		
	    if (ESDHC_OK == SDHC_ioctl (IO_IOCTL_ESDHC_SEND_COMMAND, &command))
	    {
	    	if (xmit_datablock(buff,512*count))
	    	{
	    		count = 0;
	    	}
	    }
	}
	
	/* Wait for card ready / transaction state */
    do
    {
        command.COMMAND = ESDHC_CMD13;
        command.TYPE = ESDHC_TYPE_NORMAL;
        command.ARGUMENT = sdcard_ptr->ADDRESS;
        command.READ = FALSE;
        command.BLOCKS = 0;
        if (ESDHC_OK != SDHC_ioctl (IO_IOCTL_ESDHC_SEND_COMMAND, &command))
        {
            return RES_ERROR;
        }

        /* Card status error check */
        if (command.RESPONSE[0] & 0xFFD98008)
        {
            return RES_ERROR;
        }
    } while (0x000000900 != (command.RESPONSE[0] & 0x00001F00));		
	    	
	return count ? RES_ERROR : RES_OK;
}


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
    uint8 drv,		/* Physical drive nmuber (0) */
    uint8 ctrl,		/* Control code */
	void  *buff		/* Buffer to send/receive control data */
)
{
	DRESULT 			 res;
	ESDHC_COMMAND_STRUCT command;
	SDCARD_STRUCT_PTR	 sdcard_ptr = (SDCARD_STRUCT_PTR)&SDHC_Card;	

	if (drv) return RES_PARERR;

	res = RES_ERROR;

	if (Stat & STA_NOINIT) return RES_NOTRDY;

	switch (ctrl) 
	{
		case CTRL_SYNC :		/* Make sure that no pending write process. Do not remove this or written sector might not left updated. */		
			res = RES_OK;
		break;

		case GET_SECTOR_COUNT :	/* Get number of sectors on the disk (DWORD) */
			*(unsigned long*)buff = sdcard_ptr->NUM_BLOCKS;
			res = RES_OK;
		break;		

	case GET_SECTOR_SIZE :	/* Get R/W sector size (WORD) */
		*(unsigned short*)buff = 512;
		res = RES_OK;
		break;

	case GET_BLOCK_SIZE :			/* Get erase block size in unit of sector (DWORD) */
		// Implementar
	    command.COMMAND = ESDHC_CMD9;
	    command.TYPE = ESDHC_TYPE_NORMAL;
	    command.ARGUMENT = sdcard_ptr->ADDRESS;
	    command.READ = FALSE;
	    command.BLOCKS = 0;
	    if (ESDHC_OK != SDHC_ioctl (IO_IOCTL_ESDHC_SEND_COMMAND, &command))
	    {
	        return RES_ERROR;
	    }
	    if (0 == (command.RESPONSE[3] & 0x00C00000))
	    {
	    	//SD V1
	    	*(unsigned long*)buff = ((((command.RESPONSE[2] >> 18) & 0x7F) + 1) << (((command.RESPONSE[3] >> 8) & 0x03) - 1));
	    }
	    else
	    {
	        //SD V2
	    	// Implementar
	    	//*(DWORD*)buff = (((command.RESPONSE[2] >> 18) & 0x7F) << (((command.RESPONSE[3] >> 8) & 0x03) - 1));
	    }				
		res = RES_OK;
		break;

	default:
		res = RES_PARERR;
	}

	return res;
}

/*-----------------------------------------------------------------------*/
/* Get Card Stat                                                         */
/*-----------------------------------------------------------------------*/

uint8 GetCardStat(void)
{
  return Stat;
}

/*-----------------------------------------------------------------------*/
/* Set Card Stat                                                         */
/*-----------------------------------------------------------------------*/

void SetCardStat(uint8 state)
{
  Stat = state;
}

/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
    uint8 drv		/* Physical drive nmuber (0) */
)
{
	if (drv) return STA_NOINIT;		/* Supports only single drive */
	return Stat;
}



/*-----------------------------------------------------------------------*/
/* Device Timer Interrupt Procedure                                      */
/*-----------------------------------------------------------------------*/
/* This function must be called in period of 10ms                        */

void disk_timerproc (void)
{
    uint8 s;
	
	Timer++;
	s = Stat;

	#if (SOCKWP == 1)
	if (SDCARD_GPIO_PROTECT == 0)				      
	{
		s &= ~STA_PROTECT;					/* Write enabled */
	}
	else					          
	{
		s |= STA_PROTECT;					/* Write protected */
	}
	#else
	  s &= ~STA_PROTECT;
	#endif

	if (SDCARD_GPIO_DETECT == 0)			/* Card inserted */
		s &= ~STA_NODISK;
	else					          		/* Socket empty */
		s |= (STA_NODISK | STA_NOINIT);

	Stat = s;				        		/* Update MMC status */
}


////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
/////      Return Calendar Function                    /////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

//void GetFatTimer(uint32 *time)
//{
//  UserEnterCritical();
//  *time = Timer;
//  UserExitCritical();
//}

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
/////      Set Calendar Function                       /////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

//void SetFatTimer(uint32 time)
//{
//  UserEnterCritical();
//  Timer = time;
//  UserExitCritical();
//}
