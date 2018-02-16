#ifndef FEHIO_H
#define FEHIO_H

#include "derivative.h"
#include "MK60DZ10.h"
#include "adc16.h"


class FEHIO
{
public:
    typedef enum
    {
        P0_0 = 0,
        P0_1,
        P0_2,
        P0_3,
        P0_4,
        P0_5,
        P0_6,
        P0_7,
        P1_0,
        P1_1,
        P1_2,
        P1_3,
        P1_4,
        P1_5,
        P1_6,
        P1_7,
        P2_0,
        P2_1,
        P2_2,
        P2_3,
        P2_4,
        P2_5,
        P2_6,
        P2_7,
        P3_0,
        P3_1,
        P3_2,
        P3_3,
        P3_4,
        P3_5,
        P3_6,
        P3_7,
        BATTERY_VOLTAGE
    }FEHIOPin;

    typedef enum
    {
        Bank0 = 0,
        Bank1,
        Bank2,
        Bank3
    }FEHIOPort;
	
	typedef enum
    {
        RisingEdge = 0x09,
        FallingEdge = 0x0A,
        EitherEdge = 0x0B
    }FEHIOInterruptTrigger;
};

// Begin Class Declarations for Pin Types
class DigitalInputPin
{
public:
    DigitalInputPin( FEHIO::FEHIOPin pin );
    bool Value();

    friend class ButtonBoard;

private:
    FEHIO::FEHIOPin _pin;

    DigitalInputPin();
    void Initialize( FEHIO::FEHIOPin pin );
};

class DigitalEncoder
{
public:
    DigitalEncoder( FEHIO::FEHIOPin pin, FEHIO::FEHIOInterruptTrigger trigger );
    DigitalEncoder( FEHIO::FEHIOPin pin);
    int Counts();
    void ResetCounts();

private:
    FEHIO::FEHIOPin _pin;

    DigitalEncoder();
    void Initialize( FEHIO::FEHIOPin pin, FEHIO::FEHIOInterruptTrigger trigger );
};

class AnalogInputPin
{
protected:
    FEHIO::FEHIOPin pin;
    static tADC_Config Master_Adc_Config;
    static tADC_Config Encoder_Adc_Config;

public:
    AnalogInputPin( FEHIO::FEHIOPin );
    float Value();


    static void InitADCs();
};

class DigitalOutputPin
{
private:
    FEHIO::FEHIOPin pin;
public:
    DigitalOutputPin( FEHIO::FEHIOPin );
    void Write( bool );
    int Status();
    void Toggle();
};

class ButtonBoard
{
public:
    ButtonBoard( FEHIO::FEHIOPort bank );

    bool LeftPressed();
    bool LeftReleased();

    bool MiddlePressed();
    bool MiddleReleased();

    bool RightPressed();
    bool RightReleased();

private:
    DigitalInputPin _left;
    DigitalInputPin _middle;
    DigitalInputPin _right;
};

class AnalogEncoder : public AnalogInputPin
{
private:
    typedef enum
    {
        LOW_STATE,
        HIGH_STATE
    } EncoderState;

    static void SetCounterInit(unsigned int);

    int counts;
    int lowThreshold;
    int highThreshold;

    // These hold the next and previous encoders tied to the same pin
    AnalogEncoder * pNext;
    AnalogEncoder * pPrev;

    bool ProcessIntSelf();
    int EncoderValue();

    struct PinInfo;
    typedef struct PinInfo PinInfo;

    struct PinInfo
    {
        FEHIO::FEHIOPin pin;
        EncoderState state;
        int numEncoders;
        AnalogEncoder * encoderList;
        PinInfo * pNext;
        PinInfo * pPrev;
    };


    static PinInfo * pinList;
    PinInfo * pPinInfo;

public:
    static void Init();
    static void ProcessInt();
    static void SetRate(unsigned int rateHz);
    AnalogEncoder(FEHIO::FEHIOPin);
    ~AnalogEncoder();
    int Counts();
    void ResetCounts();
    void SetThresholds(float low, float high);

};

#endif // FEHIO_H
