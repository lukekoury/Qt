#ifndef FEHRPS_H
#define FEHRPS_H

#include <FEHXBee.h>

class FEHRPS
{
public:

    FEHRPS();

	// Creates a menu to allow you to pick the correct region
	// Assumes ButtonBoard is plugged into Bank3
	// Right button increments region
	// Left button decrements region
	// Middle button selects region
	void InitializeMenu();
	
	void InitializeTouchMenu();

	// Manually pick and configure a region
	// int region => { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 }
	// char region => { a, b, c, d, e, f, g, h, i, j, k, l } || { A, B, C, D, E, F, G, H, I, J, K, L }
	void Initialize( int region );
	void Initialize( char region );

	// return the current course number { 1, 2, 3 }
	unsigned char CurrentCourse();

	// returns the letter of the current region { A, B, C, D, E, F, G, H, I, J, K, L }
	char CurrentRegionLetter();

	// returns the number of the current course { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 }
	int CurrentRegion();

	
	// Objective functions:

	// returns the percentage (of 90 degrees) that satellite has been turned
	int SatellitePercent();
	
	// returns the match time in seconds
	int Time();

	unsigned char WaitForPacket();

	float X();
	float Y();
	float Heading();

private:
	FEHXBee _xbee;
	int _region;
};

extern FEHRPS RPS;

#endif // FEHRPS_H
