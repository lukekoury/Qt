#ifndef FEHSD_H
#define FEHSD_H
#include "SDHC.h"
#include "ff.h"
#include "ffconf.h"
#include "diskio.h"

class FEHSD
{
public:
    FEHSD();
    void OpenLog();
    void CloseLog();
    void Printf(const TCHAR* str,	/* Pointer to the format string */
    ...	);


private:
	int isOpen;
    int Initialize();
};

extern FEHSD SD;
#endif

