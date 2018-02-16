#include "FEHSD.h"
#include <FEHLCD.h>
#include <stdarg.h>

#define SD_CD_LOC (1<<6)


FEHSD SD;

FIL logfil;
static FATFS FATFS_Obj;
FRESULT f_res;

float x,y;

int counter = 0;

FEHSD::FEHSD(){
	SD.isOpen = 0;
}

void FEHSD::OpenLog(){
    int status;
    if(!SD.isOpen){
            SD.isOpen = 1;
            status = FEHSD::Initialize();
            if(status==0){
                char log_str[11] = "LOG000.TXT";
                uint16 log_no = 0;
                uint16 log_no_1;
                uint16 log_no_10;
                uint16 log_no_100;
                do
                {
                    if (log_no > 999){
                        LCD.WriteLine("Too many logfiles created!");
                        return;
                    } //overflow

                    log_no_100 = log_no / 100; //eliminate 1 and 10's place
                    log_no_10 = (log_no - (log_no_100 * 100)) / 10; //subtract out 100's place, then eliminate 1's
                    log_no_1 = log_no - (log_no_100 * 100) - (log_no_10 * 10);
                    log_str[5] = log_no_1 + '0';//1's
                    log_str[4] = log_no_10 + '0';//10's
                    log_str[3] = log_no_100 + '0';//100's

                    log_no++;

                } while (f_open(&logfil, log_str, FA_CREATE_NEW | FA_WRITE) == FR_EXIST); //test to see if file exists, if it does, make a new one
            } else if (status==-1){
                LCD.WriteLine("SD Card not detected!");
            } else {
                LCD.Write(status);
                LCD.WriteLine("SD Card Initialize failed!");
            }
    }
    else{
        LCD.WriteLine("Logfile already open");
    }
}

void FEHSD::CloseLog(){
	if(SD.isOpen){
    	f_close(&logfil);
    	SD.isOpen = 0;
	}
}

void FEHSD::Printf(
    const TCHAR* str,	/* Pointer to the format string */
    ...
)
{
    va_list args;
    va_start(args, str);
    f_printf(&logfil, str, args);
}

int FEHSD::Initialize(){

    PORTE_PCR6 = PORT_PCR_MUX(1);
    PTD_BASE_PTR->PDDR &= ~SD_CD_LOC;
    int opens = 0;
    if (SDCARD_GPIO_DETECT == 0)
    {
        Sleep(300);
        opens = disk_initialize(0);
        if (SDHC_Info.CARD == ESDHC_CARD_SD)
        {

            LCD.WriteLine("SD card initiallized");
        }
        if (SDHC_Info.CARD == ESDHC_CARD_SDHC)
        {

            LCD.WriteLine("SDHC card initialized !");
        }

        // Mount FAT File System
        f_res = f_mount((uint8)0, &FATFS_Obj);
        if (f_res == 0)
        {

            LCD.WriteLine("FAT filesystem mounted !");
            return opens;
        }else
        {

            LCD.WriteLine("Failed to mount SD card !");
            return opens;
        }
            return opens;
        opens = 1;

    } else{
        opens = -1;
        return opens;
    }
    Sleep(300);
    return opens;
}
