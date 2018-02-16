#ifndef PROPELLER_H
#define PROPELLER_H
#include "derivative.h"

class FEHPropeller
{
public:
	FEHPropeller();

	bool Initialize();
	bool IsInitialized();

	void Reset();
	void SetMotorRate( uint8 motor, uint8 speed, uint8 rate );

private:
	bool _initialized;
};

#endif // PROPELLER_H
