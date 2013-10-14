#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "hardware_profile.h"
#include "freq_measure_ic.h"

static volatile bool channel1_is_enabled = false;
static volatile bool channel2_is_enabled = false;
static volatile bool channel7_is_enabled = false;
static volatile bool channel8_is_enabled = false;
static volatile uint16_t channel1_freq_measure = 0U;
static volatile uint16_t channel2_freq_measure = 0U;
static volatile uint16_t channel7_freq_measure = 0U;
static volatile uint16_t channel8_freq_measure = 0U;

void
freq_measure_ic_init(unsigned int channels_to_enable, unsigned int freq_range)
{
    T3CONbits.TON = 0U;     // Disable Timer
    T3CONbits.TCS = 0U;     // Select internal instruction cycle clock
    T3CONbits.TGATE = 0U;   // Disable Gated Timer mode
 
    switch(freq_range)
    {
        case RANGE_HIGHER_FREQ:
            T3CONbits.TCKPS = 0x0;  // Select 1:1 Prescaler
            break;
        case RANGE_HIGH_FREQ:
            T3CONbits.TCKPS = 0x1;  // Select 1:8 Prescaler
            break;
        case RANGE_MIDDLE_FREQ:
            T3CONbits.TCKPS = 0x2;  // Select 1:64 Prescaler
            break;
        case RANGE_LOW_FREQ:
            T3CONbits.TCKPS = 0x3;  // Select 1:256 Prescaler
            break;
        default:
            T3CONbits.TCKPS = 0x2;  // Select 1:64 Prescaler
            break;
    }
    
    TMR3 = 0U;              // Clear timer register
    IPC2bits.T3IP = 0x2;    // Set Timer 3 Interrupt Priority Level
    IFS0bits.T3IF = 0U;     // Clear Timer 3 Interrupt Flag
    IEC0bits.T3IE = 0U;     // Disable Timer 3 interrupt

    if((~channels_to_enable) & (~CHANNEL_IC1))
    {
        channel1_is_enabled = true;

        IC1CONbits.ICM = 0x0;       // Disable Input Capture 1 module
        IC1CONbits.ICTMR = 0U;      // Select Timer3 as the IC1 Time base
        IC1CONbits.ICI = 0x1;       // Interrupt on every second capture event
        IC1CONbits.ICM = 0x3;       // Generate capture event on every Rising edge

        IPC0bits.IC1IP = 0x2;       // Setup IC1 interrupt priority level
        IFS0bits.IC1IF = 0U;        // Clear IC1 Interrupt Status Flag
        IEC0bits.IC1IE = 1U;        // Enable IC1 interrupt
    }
#if 0    
    if((~channels_to_enable) & (~CHANNEL_IC2))
    {
        channel2_is_enabled = true;

        IC2CONbits.ICM = 0x0;       // Disable Input Capture 2 module
        IC2CONbits.ICTMR = 0U;      // Select Timer3 as the IC2 Time base
        IC2CONbits.ICI = 0x1;       // Interrupt on every second capture event
        IC2CONbits.ICM = 0x3;       // Generate capture event on every Rising edge

        IPC1bits.IC2IP = 0x2;       // Setup IC2 interrupt priority level
        IFS0bits.IC2IF = 0U;        // Clear IC2 Interrupt Status Flag
        IEC0bits.IC2IE = 1U;        // Enable IC2 interrupt
    }

    if((~channels_to_enable) & (~CHANNEL_IC7))
    {
        channel7_is_enabled = true;

        IC7CONbits.ICM = 0x0;       // Disable Input Capture 7 module
        IC7CONbits.ICTMR = 0U;      // Select Timer3 as the IC7 Time base
        IC7CONbits.ICI = 0x1;       // Interrupt on every second capture event
        IC7CONbits.ICM = 0x3;       // Generate capture event on every Rising edge

        IPC5bits.IC7IP = 0x2;       // Setup IC7 interrupt priority level
        IFS1bits.IC7IF = 0U;        // Clear IC7 Interrupt Status Flag
        IEC1bits.IC7IE = 1U;        // Enable IC7 interrupt
    }

    if((~channels_to_enable) & (~CHANNEL_IC8))
    {
        channel8_is_enabled = true;

        IC8CONbits.ICM = 0x0;       // Disable Input Capture 8 module
        IC8CONbits.ICTMR = 0U;      // Select Timer3 as the IC8 Time base
        IC8CONbits.ICI = 0x1;       // Interrupt on every second capture event
        IC8CONbits.ICM = 0x3;       // Generate capture event on every Rising edge

        IPC5bits.IC8IP = 0x2;       // Setup IC8 interrupt priority level
        IFS1bits.IC8IF = 0U;        // Clear IC8 Interrupt Status Flag
        IEC1bits.IC8IE = 1U;        // Enable IC8 interrupt
    }
#endif
    
    T3CONbits.TON = 1U;     // Start Timer

}

uint16_t
freq_measure_ic_get_freq(unsigned int channel)
{
    uint16_t freq_measure = 0U;

    switch(channel)
    {
        case CHANNEL_IC1:
            freq_measure = channel1_freq_measure;
            break;
        case CHANNEL_IC2:
            freq_measure = channel2_freq_measure;
            break;
        case CHANNEL_IC7:
            freq_measure = channel7_freq_measure;
            break;
        case CHANNEL_IC8:
            freq_measure = channel8_freq_measure;
            break;
        default:
            freq_measure = 0U;
            break;
    }

    return freq_measure;
}

void __attribute__((__interrupt__, auto_psv)) _IC1Interrupt(void)
{
    uint16_t time_pointer1;
    uint16_t time_pointer2;

    time_pointer1 = IC1BUF;
    time_pointer2 = IC1BUF;

    IFS0bits.IC1IF = 0;

    if(time_pointer2 > time_pointer1)
    {
        channel1_freq_measure = time_pointer2 - time_pointer1;
    }
    else
    {
        channel1_freq_measure = (PR3 - time_pointer1) + time_pointer2;
    }
}

#if 0
void __attribute__((__interrupt__, auto_psv)) _IC2Interrupt(void)
{
    uint16_t time_pointer1;
    uint16_t time_pointer2;

    time_pointer1 = IC2BUF;
    time_pointer2 = IC2BUF;

    IFS0bits.IC2IF = 0;

    if(time_pointer2 > time_pointer1)
    {
        channel2_freq_measure = time_pointer2 - time_pointer1;
    }
    else
    {
        channel2_freq_measure = (PR3 - time_pointer1) + time_pointer2;
    }
}

void __attribute__((__interrupt__, auto_psv)) _IC7Interrupt(void)
{
    uint16_t time_pointer1;
    uint16_t time_pointer2;

    time_pointer1 = IC7BUF;
    time_pointer2 = IC7BUF;

    IFS1bits.IC7IF = 0;

    if(time_pointer2 > time_pointer1)
    {
        channel7_freq_measure = time_pointer2 - time_pointer1;
    }
    else
    {
        channel7_freq_measure = (PR3 - time_pointer1) + time_pointer2;
    }
}

void __attribute__((__interrupt__, auto_psv)) _IC8Interrupt(void)
{
    uint16_t time_pointer1;
    uint16_t time_pointer2;

    time_pointer1 = IC8BUF;
    time_pointer2 = IC8BUF;

    IFS1bits.IC8IF = 0;

    if(time_pointer2 > time_pointer1)
    {
        channel8_freq_measure = time_pointer2 - time_pointer1;
    }
    else
    {
        channel8_freq_measure = (PR3 - time_pointer1) + time_pointer2;
    }
}
#endif