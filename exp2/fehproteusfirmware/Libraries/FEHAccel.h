#ifndef FEHACCEL_H
#define FEHACCEL_H
#include "i2c.h"

class FEHAccel
{
public:
    FEHAccel();
    double X();
    double Y();
    double Z();

};

extern FEHAccel Accel;
#endif // FEHACCEL_H
