#ifndef FEHBATTERY_H
#define FEHBATTERY_H

#include "FEHIO.h"

class FEHBattery : public AnalogInputPin
{
public:
    FEHBattery(FEHIO::FEHIOPin);
    float Voltage();
};

extern FEHBattery Battery;
#endif // FEHBUZZER_H
