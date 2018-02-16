#include "FEHIO.h"
#include "FEHLCD.h"


tADC_Config AnalogInputPin::Master_Adc_Config;
tADC_Config AnalogInputPin::Encoder_Adc_Config;

typedef enum
{
    PortA,
    PortB,
    PortC,
    PortD,
    PortE
} GPIOPort;

typedef enum
{
    low,
    high
} GPIOValue;

typedef enum
{
    ADC0,
    ADC1
} ADCNumber;

const GPIOPort GPIOPorts[ 32 ] =
{
    PortB, PortB, PortB, PortB, PortB, PortB, PortB, PortB,
    PortC, PortC, PortC, PortC, PortC, PortC, PortA, PortA,
    PortA, PortA, PortA, PortA, PortA, PortA, PortA, PortA,
    PortA, PortE, PortE, PortE, PortE, PortE, PortD, PortD
};

const ADCNumber ADCNumbers[ 33 ] =
{
    ADC1, ADC1, ADC1, ADC1, ADC1, ADC1, ADC1, ADC1,
    ADC0, ADC0, ADC1, ADC1, ADC1, ADC1, ADC1, ADC0,
    ADC0, ADC1, ADC1, ADC0, ADC0, ADC1, ADC1, ADC0,
    ADC0, ADC0, ADC0, ADC0, ADC1, ADC0, ADC0, ADC0,
    ADC0
};

const int GPIOPinNumbers[ 32 ] =
{
    11, 10, 7, 6, 5, 4, 1, 0,
    0, 1, 8, 9, 10, 11, 17, 16,
    15, 14, 13, 12, 11, 10, 9, 8,
    7, 25, 24, 26, 27, 28, 1, 6
};

const int AnalogPinNumbers[ 33 ] =
{
    15, 14, 13, 12, 11, 10, 9, 8,
    14, 15, 4, 5, 6, 7, 17, 1,
    20, 1, 20, 0, 19, 0, 19, 11,
    10, 18, 17, 16, 16, 4, 5, 7,
    6
};

// Begin Functions for Digital Input Pin Type
DigitalInputPin::DigitalInputPin( FEHIO::FEHIOPin pin )
{
    Initialize( pin );
}

DigitalInputPin::DigitalInputPin()
{

}

void DigitalInputPin::Initialize( FEHIO::FEHIOPin pin )
{
    // store selected pin number in class
    _pin = pin;
    switch( GPIOPorts[ (int)_pin ] )
    {
        case PortA:
        {
            PORT_PCR_REG( PORTA_BASE_PTR, GPIOPinNumbers[ (int)_pin ] ) = ( 0 | PORT_PCR_MUX( 1 ) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK );
            GPIOA_PDDR &= ~GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)_pin ] ) );
            break;
        }
        case PortB:
        {
            PORT_PCR_REG( PORTB_BASE_PTR, GPIOPinNumbers[ (int)_pin ] ) = ( 0 | PORT_PCR_MUX( 1 ) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK );
            GPIOB_PDDR &= ~GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)_pin ] ) );
            break;
        }
        case PortC:
        {
            PORT_PCR_REG( PORTC_BASE_PTR, GPIOPinNumbers[ (int)_pin ] ) = ( 0 | PORT_PCR_MUX( 1 ) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK );
            GPIOC_PDDR &= ~GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)_pin ] ) );
            break;
        }
        case PortD:
        {
            PORT_PCR_REG( PORTD_BASE_PTR, GPIOPinNumbers[ (int)_pin ] ) = ( 0 | PORT_PCR_MUX( 1 ) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK );
            GPIOD_PDDR &= ~GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)_pin ] ) );
            break;
        }
        case PortE:
        {
            PORT_PCR_REG( PORTE_BASE_PTR, GPIOPinNumbers[ (int)_pin ] ) = ( 0 | PORT_PCR_MUX( 1 ) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK );
            GPIOE_PDDR &= ~GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)_pin ] ) );
            break;
        }
    }
}

bool DigitalInputPin::Value()
{
    int ret = 0;
    switch( GPIOPorts[ (int)_pin ] )
    {
        case PortA:
        {
            ret = GPIOA_PDIR & GPIO_PDIR_PDI( GPIO_PIN( GPIOPinNumbers[ (int)_pin ] ) );
            break;
        }
        case PortB:
        {
            ret = GPIOB_PDIR & GPIO_PDIR_PDI( GPIO_PIN( GPIOPinNumbers[ (int)_pin ] ) );
            break;
        }
        case PortC:
        {
            ret = GPIOC_PDIR & GPIO_PDIR_PDI( GPIO_PIN( GPIOPinNumbers[ (int)_pin ] ) );
            break;
        }
        case PortD:
        {
            ret = GPIOD_PDIR & GPIO_PDIR_PDI( GPIO_PIN( GPIOPinNumbers[ (int)_pin ] ) );
            break;
        }
        case PortE:
        {
            ret = GPIOE_PDIR & GPIO_PDIR_PDI( GPIO_PIN( GPIOPinNumbers[ (int)_pin ] ) );
            break;
        }
    }
    return ret;
}
// Begin Functions for Digital Encoder Pin Type
DigitalEncoder::DigitalEncoder( FEHIO::FEHIOPin pin, FEHIO::FEHIOInterruptTrigger trigger )
{
    Initialize( pin,trigger );
}
DigitalEncoder::DigitalEncoder( FEHIO::FEHIOPin pin)
{
    FEHIO::FEHIOInterruptTrigger trigger(FEHIO::EitherEdge);
    Initialize( pin,trigger );
}

DigitalEncoder::DigitalEncoder()
{

}

// Function used to enable interrupt register
void enable_irq(int irq)
{
    int div;
    div = (irq-16)/32;

    switch(div)
    {
        case 0x0:
            NVICICPR0 |= 1 << ((irq-16)%32);
            NVICISER0 |= 1 << ((irq-16)%32);
            break;
        case 0x1:
            NVICICPR1 |= 1 << ((irq-16)%32);
            NVICISER1 |= 1 << ((irq-16)%32);
            break;
        case 0x2:
            NVICICPR2 |= 1 << ((irq-16)%32);
            NVICISER2 |= 1 << ((irq-16)%32);
            break;
    }
}

void DigitalEncoder::Initialize( FEHIO::FEHIOPin pin, FEHIO::FEHIOInterruptTrigger trigger )
{
    // store selected pin number in class
    _pin = pin;
	unsigned char trig = (unsigned char)trigger;
    switch( GPIOPorts[ (int)_pin ] )
    {
        case PortA:
        {
            PORT_PCR_REG( PORTA_BASE_PTR, GPIOPinNumbers[ (int)_pin ] ) = ( 0 | PORT_PCR_MUX( 1 ) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(trig) | PORT_PCR_PFE_MASK );
            GPIOA_PDDR &= ~GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)_pin ] ) );
            enable_irq(INT_PORTA);
            break;
        }
        case PortB:
        {
            PORT_PCR_REG( PORTB_BASE_PTR, GPIOPinNumbers[ (int)_pin ] ) = ( 0 | PORT_PCR_MUX( 1 ) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(trig) | PORT_PCR_PFE_MASK );
            GPIOB_PDDR &= ~GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)_pin ] ) );
            enable_irq(INT_PORTB);
            break;
        }
        case PortC:
        {
            PORT_PCR_REG( PORTC_BASE_PTR, GPIOPinNumbers[ (int)_pin ] ) = ( 0 | PORT_PCR_MUX( 1 ) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(trig) | PORT_PCR_PFE_MASK );
            GPIOC_PDDR &= ~GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)_pin ] ) );
            enable_irq(INT_PORTC);
            break;
        }
        case PortD:
        {
            //Port D is already in use for power reset pin. Therefore Digital Encoders cannot be used on P3_6 and P3_7
            //PORT_PCR_REG( PORTD_BASE_PTR, GPIOPinNumbers[ (int)_pin ] ) = ( 0 | PORT_PCR_MUX( 1 ) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(0xA) | PORT_PCR_PFE_MASK );
            //GPIOD_PDDR &= ~GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)_pin ] ) );
            //enable_irq(INT_PORTD);
            break;
        }
        case PortE:
        {
            PORT_PCR_REG( PORTE_BASE_PTR, GPIOPinNumbers[ (int)_pin ] ) = ( 0 | PORT_PCR_MUX( 1 ) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(trig) | PORT_PCR_PFE_MASK );
            GPIOE_PDDR &= ~GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)_pin ] ) );
            enable_irq(INT_PORTE);
            break;
        }
    }
}

//Interrupt port functions
unsigned long interrupt_counts[32];
void portB_isr()
{
    int pins[8] = { 11, 10, 7, 6, 5, 4, 1, 0 };

    for( int i=0 ; i<8 ; i++)
    {

        if( (PORTB_ISFR & (1<<pins[i])) != 0)
        {
            interrupt_counts[i]++;
            PORTB_ISFR &= (1<<pins[i]);
        }
    }
}
void portC_isr()
{
    int pins[6] = { 0, 1, 8, 9, 10, 11 };

    for( int i=0 ; i<6 ; i++)
    {
        if( (PORTC_ISFR & (1<<pins[i])) != 0)
        {
            interrupt_counts[i+8]++;
            PORTC_ISFR &= (1<<pins[i]);
        }
    }
}
void portA_isr()
{
    int pins[11] = { 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7 };

    for( int i=0 ; i<11 ; i++)
    {
        if( (PORTA_ISFR & (1<<pins[i])) != 0)
        {
            interrupt_counts[i+14]++;
            PORTA_ISFR &= (1<<pins[i]);
        }
    }
}
void portE_isr()
{
    int pins[5] = { 25, 24, 26, 27, 28 };

    for( int i=0 ; i<5 ; i++)
    {
        if( (PORTE_ISFR & (1<<pins[i])) != 0)
        {
            interrupt_counts[i+25]++;
            PORTE_ISFR &= (1<<pins[i]);
        }
    }
}

int DigitalEncoder::Counts()
{
    return interrupt_counts[_pin];
}

void DigitalEncoder::ResetCounts()
{
    interrupt_counts[_pin] = 0;
}

// Begin Functions for Analog Input Pin Type
AnalogInputPin::AnalogInputPin( FEHIO::FEHIOPin _pin )
{
    pin = _pin;
}


//Analog read function causes digital output to behave strangely????????
float AnalogInputPin::Value()
{
    int analogPin = AnalogPinNumbers[pin];
        ADCNumber adcNum = ADCNumbers[pin];

         Master_Adc_Config.STATUS1A = AIEN_OFF | DIFF_SINGLE | ADC_SC1_ADCH(analogPin);
         Master_Adc_Config.STATUS1B = AIEN_OFF | DIFF_SINGLE | ADC_SC1_ADCH(analogPin);

        unsigned int result;

        // Disable Encoder Interrupts Temporarily
        NVICICER2 = (1 << (4));

        if (adcNum == ADC0)
        {
             ADC_Config_Alt(ADC0_BASE_PTR, &Master_Adc_Config);  // config ADC0

             // Check the status control register to see is the COnversion is COmplete
             while (( ADC0_SC1A & ADC_SC1_COCO_MASK ) != ADC_SC1_COCO_MASK){}
             result = ADC0_RA;
        }
        else
        {
             ADC_Config_Alt(ADC1_BASE_PTR, &Master_Adc_Config);  // config ADC0

             // Check the status control register to see is the COnversion is COmplete
             while (( ADC1_SC1A & ADC_SC1_COCO_MASK ) != ADC_SC1_COCO_MASK){}
             result = ADC1_RA;

        }

        // Re-enable Encoder Interrupt
        NVICICPR2 |= (1 << (4));
        NVICISER2 |= (1 << (4));

        // // Re-enable button Interrupt
        // NVICICPR2 |= (1 << ( 26 ));
        // NVICISER2 |= (1 << ( 26 ));


        float v = (result & 0xFFFFu) *3.33 / (0xFFFFu);
        return v;
}


int AnalogEncoder::EncoderValue()
{
    int analogPin = AnalogPinNumbers[pin];
    ADCNumber adcNum = ADCNumbers[pin];

     Encoder_Adc_Config.STATUS1A = AIEN_OFF | DIFF_SINGLE | ADC_SC1_ADCH(analogPin);
     Encoder_Adc_Config.STATUS1B = AIEN_OFF | DIFF_SINGLE | ADC_SC1_ADCH(analogPin);

    int result;

    if (adcNum == ADC0)
    {
         ADC_Config_Alt(ADC0_BASE_PTR, &Encoder_Adc_Config);  // config ADC0

         // Check the status control register to see is the COnversion is COmplete
         while (( ADC0_SC1A & ADC_SC1_COCO_MASK ) != ADC_SC1_COCO_MASK){}
         result = ADC0_RA;
    }
    else
    {
         ADC_Config_Alt(ADC1_BASE_PTR, &Encoder_Adc_Config);  // config ADC0

         // Check the status control register to see is the COnversion is COmplete
         while (( ADC1_SC1A & ADC_SC1_COCO_MASK ) != ADC_SC1_COCO_MASK){}
         result = ADC1_RA;
    }
    return result;
}


// Begin Functions for Digital Output Pin Type
DigitalOutputPin::DigitalOutputPin( FEHIO::FEHIOPin _pin )
{
    // store selected pin number in class
    pin = _pin;
    switch( GPIOPorts[ (int)pin ] )
                {
                    case PortA:
                    {
                        PORT_PCR_REG( PORTA_BASE_PTR, GPIOPinNumbers[ (int)pin ] ) = ( 0 | PORT_PCR_MUX( 1 ) );
                        GPIOA_PDDR |= GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                        break;
                    }
                    case PortB:
                    {
                        PORT_PCR_REG( PORTB_BASE_PTR, GPIOPinNumbers[ (int)pin ] ) = ( 0 | PORT_PCR_MUX( 1 ) );
                        GPIOB_PDDR |= GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                        break;
                    }
                    case PortC:
                    {
                        PORT_PCR_REG( PORTC_BASE_PTR, GPIOPinNumbers[ (int)pin ] ) = ( 0 | PORT_PCR_MUX( 1 ) );
                        GPIOC_PDDR |= GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                        break;
                    }
                    case PortD:
                    {
                        PORT_PCR_REG( PORTD_BASE_PTR, GPIOPinNumbers[ (int)pin ] ) = ( 0 | PORT_PCR_MUX( 1 ) );
                        GPIOD_PDDR |= GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                        break;
                    }
                    case PortE:
                    {
                        PORT_PCR_REG( PORTE_BASE_PTR, GPIOPinNumbers[ (int)pin ] ) = ( 0 | PORT_PCR_MUX( 1 ) );
                        GPIOE_PDDR |= GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                        break;
                    }
                }
}

void DigitalOutputPin::Write( bool value )
{
    switch( value )
    {
        case true:
        {
            switch( GPIOPorts[ (int)pin ] )
            {
                case PortA:
                {
                    GPIOA_PDOR |= GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                    break;
                 }
                case PortB:
                {
                    GPIOB_PDOR |= GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                    break;
                }
                case PortC:
                {
                    GPIOC_PDOR |= GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                    break;
                }
                case PortD:
                {
                    GPIOD_PDOR |= GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                    break;
                }
                case PortE:
                {
                    GPIOE_PDOR |= GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                    break;
                }
            }
            break;
        }
        case false:
        {
            switch( GPIOPorts[ (int)pin ] )
            {
                case PortA:
                {
                    GPIOA_PDOR &= ~GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                    break;
                }
                case PortB:
                {
                    GPIOB_PDOR &= ~GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                    break;
                }
                case PortC:
                {
                    GPIOC_PDOR &= ~GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                    break;
                }
                case PortD:
                {
                    GPIOD_PDOR &= ~GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                    break;
                }
                case PortE:
                {
                    GPIOE_PDOR &= ~GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                    break;
                }
            }
        }
    }
}

int DigitalOutputPin::Status()
{
    int ret = 0;
        switch( GPIOPorts[ (int)pin ] )
        {
            case PortA:
            {
                ret = GPIOA_PDOR & GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                break;
            }
            case PortB:
            {
                ret = GPIOB_PDOR & GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                break;
            }
            case PortC:
            {
                ret = GPIOC_PDOR & GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                break;
            }
            case PortD:
            {
                ret = GPIOD_PDOR & GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                break;
            }
            case PortE:
            {
                ret = GPIOE_PDOR & GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                break;
            }
        }
        return ret;
}

void DigitalOutputPin::Toggle()
{
    switch( GPIOPorts[ (int)pin ] )
        {
            case PortA:
            {
                GPIOA_PTOR |= GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                break;
            }
            case PortB:
            {
                GPIOB_PTOR |= GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                break;
            }
            case PortC:
            {
                GPIOC_PTOR |= GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                break;
            }
            case PortD:
            {
                GPIOD_PTOR |= GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                break;
            }
            case PortE:
            {
                GPIOE_PTOR |= GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                break;
            }
        }

}

////Initialize ADC Function. Needs to be placed somewhere??
void AnalogInputPin::InitADCs()
{
    // setup the initial ADC default configuration
         Master_Adc_Config.CONFIG1  = ADLPC_NORMAL                   // Normal power, (not low power)
                                    | ADC_CFG1_ADIV(ADIV_4)          // Clock divider
                                    | ADLSMP_LONG                    // Take a long time to sample
                                    | ADC_CFG1_MODE(MODE_16)         // 16 bit mode
                                    | ADC_CFG1_ADICLK(ADICLK_BUS);   // use the bus clock
         Master_Adc_Config.CONFIG2  = MUXSEL_ADCB                    // use channel A
                                    | ADACKEN_DISABLED               // Asynch clock disabled?
                                    | ADHSC_NORMAL                   // Asynch clock setting
                                    | ADC_CFG2_ADLSTS(ADLSTS_20) ;
         Master_Adc_Config.COMPARE1 = 0x1234u ;                 // can be anything
         Master_Adc_Config.COMPARE2 = 0x5678u ;                 // can be anything
                                                                // since not using
                                                                // compare feature
         Master_Adc_Config.STATUS2  = ADTRG_SW                  // Software triggered conversion
                                    | ACFE_DISABLED             // Disable comparator (if enabled only registers as an anlog reading if it is greater than a certain value)
                                    | ACFGT_GREATER             // comparator setting
                                    | ACREN_DISABLED            // Compare Function Range disabled
                                    | DMAEN_DISABLED               // Disable DMA
                                    | ADC_SC2_REFSEL(REFSEL_EXT); // external voltage reference

         Master_Adc_Config.STATUS3  = CAL_OFF                     // Calibration begins off
                                    | ADCO_SINGLE                 // Take a single reading
                                    | AVGE_ENABLED                // Enable averaging
                                    | ADC_SC3_AVGS(AVGS_32);      // Average 32 samples

         Master_Adc_Config.PGA      = PGAEN_DISABLED             // PGA disabled
                                    | PGACHP_NOCHOP              // no chopping for PGA?
                                    | PGALP_NORMAL               // Normal (not low power mode)
                                    | ADC_PGA_PGAG(PGAG_64);     // PGA gain of 64

         // Set up channel as all ones for configuration
         Master_Adc_Config.STATUS1A = AIEN_OFF | DIFF_SINGLE | ADC_SC1_ADCH(31);

         Master_Adc_Config.STATUS1B = AIEN_OFF | DIFF_SINGLE | ADC_SC1_ADCH(31);


        // Configure ADC as it will be used, but becuase ADC_SC1_ADCH is 31,
        // the ADC will be inactive.  Channel 31 is just disable function.
        // There really is no channel 31.

         ADC_Config_Alt(ADC0_BASE_PTR, &Master_Adc_Config);  // config ADC
         ADC_Config_Alt(ADC1_BASE_PTR, &Master_Adc_Config);  // config ADC

        // Calibrate the ADC in the configuration in which it will be used:
         ADC_Cal(ADC1_BASE_PTR);                    // do the calibration
         ADC_Cal(ADC0_BASE_PTR);                    // do the calibration

        // The structure still has the desired configuration.  So restore it.
        // Why restore it?  The calibration makes some adjustments to the
        // configuration of the ADC.  The are now undone:

        // config the ADC again to desired conditions
         ADC_Config_Alt(ADC1_BASE_PTR, &Master_Adc_Config);
         ADC_Config_Alt(ADC0_BASE_PTR, &Master_Adc_Config);

         //Load Encoder ADC Config (A bit different from Master)
         Encoder_Adc_Config = Master_Adc_Config;

         Encoder_Adc_Config.CONFIG1  = ADLPC_NORMAL                   // Normal power, (not low power)
                                    | ADC_CFG1_ADIV(ADIV_4)          // Clock divider
                                    | ADLSMP_LONG                    // Take a long time to sample
                                    | ADC_CFG1_MODE(MODE_16)         // 16 bit mode
                                    | ADC_CFG1_ADICLK(ADICLK_BUS);   // use the bus clock

         Encoder_Adc_Config.STATUS3  = CAL_OFF                     // Calibration begins off
                                    | ADCO_SINGLE                 // Take a single reading
                                    | AVGE_ENABLED                // Enable averaging
                                    | ADC_SC3_AVGS(AVGS_4);      // Average 4 samples
}

#define NULL 0
AnalogEncoder::PinInfo * AnalogEncoder::pinList = NULL;


volatile long __interrupt_counter = 0;

void pit0_isr(void) {
	__interrupt_counter++;
    AnalogEncoder::ProcessInt();
    PIT_TFLG0 = PIT_TFLG_TIF_MASK;
    return;
}

void AnalogEncoder::SetRate(unsigned int rateHz)
{
	unsigned long countDown = 44000000UL / rateHz;
	if(countDown < 0x100000000UL)
	{
		PIT_LDVAL0 = countDown;
	}
}

void AnalogEncoder::Init() {
    // Freeze on Debug
    PIT_MCR = PIT_MCR_FRZ_MASK;

    // Load wait period (100,000 clock divider = 440 Hz interrupts)
    SetCounterInit(0x186A0);

    //#define PIT_CVAL0                                PIT_CVAL_REG(PIT_BASE_PTR,0)

    // Enable PINT0 and interrupt
    PIT_TCTRL0 = PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;

    // Other Vector Interrupt Register
    NVICICPR2 |= 1 << (4);
    NVICISER2 |= 1 << (4);
}

void AnalogEncoder::SetCounterInit(unsigned int val) {
    PIT_LDVAL0  =  val;
}

AnalogEncoder::AnalogEncoder(FEHIO::FEHIOPin pin_) : AnalogInputPin(pin_)
{
    if(pinList == NULL) {
        // Initialize linked list of encoders within the pPinInfo Object
        this->pNext = NULL;
        this->pPrev = NULL;

        // Add encoder linked list to pinInfo
        pPinInfo = new PinInfo;
        pPinInfo->encoderList = this;
        pPinInfo->numEncoders = 1;
        pPinInfo->pin = pin_;
        pPinInfo->pNext = NULL;
        pPinInfo->pPrev = NULL;
        pPinInfo->state = LOW_STATE;

        // Since the pinList is empty, set the pinInfo to the head
        pinList = pPinInfo;
    }
    else // There is already a list in place (see if this pin is already in it)
    {
        PinInfo * pCurPin = pinList;
        // Loop through list trying to find our pin
        while(pCurPin->pin!= pin_ && pCurPin->pNext!=NULL) {
            pCurPin = pCurPin->pNext;
        }

        // If the pin is in the list
        if(pCurPin->pin == pin_) {
            pPinInfo = pCurPin;
            AnalogEncoder * pCurEnc = pPinInfo->encoderList;
            // Loop to end of encoder list
            while(pCurEnc->pNext != NULL) {
                pCurEnc = pCurEnc->pNext;
            }
            // Link the encoder info the end of the list
            pCurEnc->pNext = this;
            this->pPrev = pCurEnc;
            this->pNext = NULL;
            pPinInfo->numEncoders++;
        }
        else { // Need to add a new pin to the list
            this->pNext = NULL;
            this->pPrev = NULL;

            // Add encoder linked list to pinInfo
            pPinInfo = new PinInfo;
            pPinInfo->encoderList = this;
            pPinInfo->numEncoders = 1;
            pPinInfo->pin = pin_;
            pPinInfo->pNext = NULL;
            pPinInfo->state = LOW_STATE;

            // Link the end of the list to the new pin
            pPinInfo->pPrev = pCurPin;
            pCurPin->pNext = pPinInfo;
        }
    }

    ResetCounts();
    lowThreshold = 70*256;
    highThreshold = 210*256;
}

void AnalogEncoder::SetThresholds(float low, float high) {
    lowThreshold = low/3.3*0x10000;
    highThreshold = high/3.3*0x10000;
}

AnalogEncoder::~AnalogEncoder()
{
    // Remove this encoder from the encoder list in the pin info
    AnalogEncoder * pPrevEnc = this->pPrev;
    AnalogEncoder * pNextEnc = this->pNext;

    if(pPrevEnc==NULL && pNextEnc==NULL) {
        //Then this was a lone encoder, must kill the pinInfo entry too
        PinInfo * pPrevPin = pPinInfo->pPrev;
        PinInfo * pNextPin = pPinInfo->pNext;
        if(pPrevPin==NULL && pNextPin==NULL) {
            // The pin list is now empty
            pinList = NULL;
        }
        else {
            if(pPrevPin!=NULL) {
                pPrevPin->pNext = pNextPin;
            }
            else { // Reset the head
                pinList = pNextPin;
            }

            if(pNextPin!=NULL) {
                pNextPin->pPrev = pPrevPin;
            }
        }
        // Delete the pinInfo
        delete pPinInfo;
    }
    else {
        // If we aren't at the start of the list
        if(pPrevEnc!=NULL) {
            pPrevEnc->pNext = pNextEnc;
        }
        else { // Reset the head
            pPinInfo->encoderList = pNextEnc;
        }

        if(pNextEnc!=NULL) {
            pNextEnc->pPrev = pPrevEnc;
        }
        pPinInfo->numEncoders--;
    }
}

void AnalogEncoder::ProcessInt() {

    PinInfo * pCurPin = pinList;
    while(pCurPin != NULL)
    {
        EncoderState oldState = pCurPin->state;
        // Process interrupt for the first encoder on each pin
        bool countReceived = pCurPin->encoderList->ProcessIntSelf();

        // If the state changed and there are more encoders tied to this pin
        if(countReceived && pCurPin->numEncoders > 1) {
            // Get the pointer to the next object
            AnalogEncoder * pCurEnc = pCurPin->encoderList->pNext;
            while(pCurEnc != NULL) {
                // Update other encoders
                pCurEnc->counts++;

                // Move to the next in the list
                pCurEnc = pCurEnc->pNext;
            }
        }
        pCurPin = pCurPin->pNext;
    }
}

bool AnalogEncoder::ProcessIntSelf() {
    int value = EncoderValue();
    EncoderState state = pPinInfo->state;
    if(state == LOW_STATE && value>highThreshold) {
        pPinInfo->state = HIGH_STATE;
        counts++;
        return true;
    }
    if(state== HIGH_STATE && value<lowThreshold) {
        pPinInfo->state = LOW_STATE;
        counts++;
        return true;
    }
    return false;
}

int AnalogEncoder::Counts()
{
    return (int) counts;
}

void AnalogEncoder::ResetCounts()
{
    counts=0;
}


ButtonBoard::ButtonBoard( FEHIO::FEHIOPort bank )
{
    switch( bank )
    {
        case FEHIO::Bank0:
        {
            _left.Initialize( FEHIO::P0_0 );
            _middle.Initialize( FEHIO::P0_1 );
            _right.Initialize( FEHIO::P0_2 );
            break;
        }
        case FEHIO::Bank1:
        {
            _left.Initialize( FEHIO::P1_0 );
            _middle.Initialize( FEHIO::P1_1 );
            _right.Initialize( FEHIO::P1_2 );
            break;
        }
        case FEHIO::Bank2:
        {
            _left.Initialize( FEHIO::P2_0 );
            _middle.Initialize( FEHIO::P2_1 );
            _right.Initialize( FEHIO::P2_2 );
            break;
        }
        case FEHIO::Bank3:
        {
            _left.Initialize( FEHIO::P3_0 );
            _middle.Initialize( FEHIO::P3_1 );
            _right.Initialize( FEHIO::P3_2 );
            break;
        }
    }
}

bool ButtonBoard::LeftPressed()
{
    return ( _left.Value() == 0 );
}

bool ButtonBoard::LeftReleased()
{
    return ( _left.Value() == 1 );
}

bool ButtonBoard::MiddlePressed()
{
    return ( _middle.Value() == 0 );
}

bool ButtonBoard::MiddleReleased()
{
    return ( _middle.Value() == 1 );
}

bool ButtonBoard::RightPressed()
{
    return ( _right.Value() == 0 );
}

bool ButtonBoard::RightReleased()
{
    return ( _right.Value() == 1 );
}


