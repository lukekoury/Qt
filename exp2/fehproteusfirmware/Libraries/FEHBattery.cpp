#include "FEHBattery.h"
#include "FEHIO.h"

FEHBattery Battery(FEHIO::BATTERY_VOLTAGE);

float FEHBattery::Voltage() {
    return Value()*(604+205)/205.;
}

FEHBattery::FEHBattery(FEHIO::FEHIOPin pin_) : AnalogInputPin(pin_){}
