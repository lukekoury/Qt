#include "FEHUtility.h"
#include "lptmr.h"

#define __USE_NEW_SLEEP__

// My new delay scheme caputes the timer registers immediately, and then calls this delay function
// note that the system time is RTC_TSR + RTC_TPR/(0x8000)
// That is, RTC_TSR contains seconds, while RTC_TPR contains the decimal information
static void Delay(unsigned long ms, const unsigned int tsr0, const unsigned int tpr0)
{
    unsigned int tsrf = tsr0 + ms/1000;
    ms %= 1000;
    unsigned int tprf = tpr0 + (ms << 15)/1000;
    tsrf += tprf / 0x8000u;
    tprf %= 0x8000u;
    while( RTC_TSR < tsrf || (RTC_TSR == tsrf && RTC_TPR <tprf)) {}
}

void Sleep( int msec )
{
#ifdef __USE_NEW_SLEEP__
    const unsigned int tsr0 = RTC_TSR;
    const unsigned int tpr0 = RTC_TPR;
    Delay( (unsigned int)msec, tsr0,tpr0 );
#else
    time_delay_ms( (unsigned int)msec );
#endif
}

void Sleep( float sec )
{
#ifdef __USE_NEW_SLEEP__
    const unsigned int tsr0 = RTC_TSR;
    const unsigned int tpr0 = RTC_TPR;
    Delay( (unsigned int)( sec * 1000 ), tsr0,tpr0 );
#else
    time_delay_ms( (unsigned int)( sec * 1000 ) );
#endif
}

void Sleep( double sec )
{
#ifdef __USE_NEW_SLEEP__
    const unsigned int tsr0 = RTC_TSR;
    const unsigned int tpr0 = RTC_TPR;
    Delay( (unsigned int)( sec * 1000 ), tsr0,tpr0 );
#else
    time_delay_ms( (unsigned int)( sec * 1000 ) );
#endif
}

double TimeNow()
{
    unsigned int tsr = RTC_TSR;
    unsigned int tpr = RTC_TPR;
    double t = tsr;
    t+=(tpr) / ((double) 0x8000u);
    return t;
}

unsigned int TimeNowSec() {
    return (unsigned int) RTC_TSR;
}

unsigned long TimeNowMSec() {
    unsigned long m;
    m = ((unsigned int) RTC_TSR)*1000;
    m+= (((unsigned int) RTC_TPR)*1000) >> 15;
    return m;
}

void ResetTime()
{
    RTC_SR &=  ~RTC_SR_TCE_MASK;
    RTC_TSR = 0x0u;
    RTC_TPR = 0x0u;
    RTC_SR =  RTC_SR_TCE_MASK;
}
