/*
 * File:   servo_ocpwm.c
 * Author: Miguel Zea
 *
 * Created on 15 de agosto de 2013, 10:07 AM
 */

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "hardware_profile.h"
#include "ds65hb_oc.h"

static volatile bool servo1_is_enabled = false;
static volatile bool servo2_is_enabled = false;
static volatile bool servo3_is_enabled = false;
static volatile bool servo4_is_enabled = false;
static volatile uint16_t servo1_pulse_width = DS65HB_NEUTRAL;
static volatile uint16_t servo2_pulse_width = DS65HB_NEUTRAL;
static volatile uint16_t servo3_pulse_width = DS65HB_NEUTRAL;
static volatile uint16_t servo4_pulse_width = DS65HB_NEUTRAL;

void
ds65hb_oc_init(unsigned int servos_to_enable)
{
    if((~servos_to_enable) & (~DS65HB_OC1))
    {
        servo1_is_enabled = true;

        OC1CONbits.OCM = 0U;        // Disable Output Compare Module
        OC1R = DS65HB_NEUTRAL;      // Write the duty cycle for the first PWM pulse
        OC1RS = DS65HB_NEUTRAL;     // Write the duty cycle for the second PWM pulse
        OC1CONbits.OCTSEL = 0U;     // Select Timer 2 as output compare time base
        OC1R = DS65HB_NEUTRAL;      // Load the Compare Register Value
        OC1CONbits.OCM = 0x6;       // Select the Output Compare mode
    }

    if((~servos_to_enable) & (~DS65HB_OC2))
    {
        servo2_is_enabled = true;

        OC2CONbits.OCM = 0U;        // Disable Output Compare Module
        OC2R = DS65HB_NEUTRAL;      // Write the duty cycle for the first PWM pulse
        OC2RS = DS65HB_NEUTRAL;     // Write the duty cycle for the second PWM pulse
        OC2CONbits.OCTSEL = 0U;     // Select Timer 2 as output compare time base
        OC2R = DS65HB_NEUTRAL;      // Load the Compare Register Value
        OC2CONbits.OCM = 0x6;       // Select the Output Compare mode
    }

    if((~servos_to_enable) & (~DS65HB_OC3))
    {
        servo3_is_enabled = true;

        OC3CONbits.OCM = 0U;        // Disable Output Compare Module
        OC3R = DS65HB_NEUTRAL;      // Write the duty cycle for the first PWM pulse
        OC3RS = DS65HB_NEUTRAL;     // Write the duty cycle for the second PWM pulse
        OC3CONbits.OCTSEL = 0U;     // Select Timer 2 as output compare time base
        OC3R = DS65HB_NEUTRAL;      // Load the Compare Register Value
        OC3CONbits.OCM = 0x6;       // Select the Output Compare mode
    }

    if((~servos_to_enable) & (~DS65HB_OC4))
    {
        servo4_is_enabled = true;

        OC4CONbits.OCM = 0U;        // Disable Output Compare Module
        OC4R = DS65HB_NEUTRAL;      // Write the duty cycle for the first PWM pulse
        OC4RS = DS65HB_NEUTRAL;     // Write the duty cycle for the second PWM pulse
        OC4CONbits.OCTSEL = 0U;     // Select Timer 2 as output compare time base
        OC4R = DS65HB_NEUTRAL;      // Load the Compare Register Value
        OC4CONbits.OCM = 0x6;       // Select the Output Compare mode
    }

    T2CONbits.TON = 0U;     // Disable Timer
    T2CONbits.TCS = 0U;     // Select internal instruction cycle clock
    T2CONbits.TGATE = 0U;   // Disable Gated Timer mode
    T2CONbits.T32 = 0U;     // Disable 32-bit timer
    T2CONbits.TCKPS = 0x1;  // Select 1:8 Prescaler
    TMR2 = 0U;              // Clear timer register
                            // Load the period value (300 Hz) (16505U)
    PR2 = 14880U;           // Load the period value (~333 Hz) (14880U)

    IPC1bits.T2IP = 0x2;    // Set Timer 2 Interrupt Priority Level
    IFS0bits.T2IF = 0U;     // Clear Timer 2 Interrupt Flag
    IEC0bits.T2IE = 1U;     // Enable Timer 2 interrupt
    T2CONbits.TON = 1U;     // Start Timer
}

unsigned int
ds65hb_oc_set_pulse_width(unsigned int servo_id, uint16_t pulse_width)
{
    unsigned int operation_error = 0U;

    if((pulse_width < DS65HB_MIN_PULSE_WIDTH) ||
            (pulse_width > DS65HB_MAX_PULSE_WIDTH))
    {
        operation_error = 1U;
    }

    if(0U == operation_error)
    {
        switch(servo_id)
        {
            case DS65HB_OC1:
                servo1_pulse_width = pulse_width;
                break;
            case DS65HB_OC2:
                servo2_pulse_width = pulse_width;
                break;
            case DS65HB_OC3:
                servo3_pulse_width = pulse_width;
                break;
            case DS65HB_OC4:
                servo4_pulse_width = pulse_width;
                break;
            default:
                operation_error = 2U;
                break;
        }
    }

    return operation_error;
}

uint16_t
ds65hb_oc_get_pulse_width(unsigned int servo_id)
{
    uint16_t pulse_width = 0U;

    switch(servo_id)
    {
        case DS65HB_OC1:
            pulse_width = servo1_pulse_width;
            break;
        case DS65HB_OC2:
            pulse_width = servo2_pulse_width;
            break;
        case DS65HB_OC3:
            pulse_width = servo3_pulse_width;
            break;
        case DS65HB_OC4:
            pulse_width = servo4_pulse_width;
            break;
        default:
            pulse_width = 0U;
            break;
    }

    return pulse_width;
}


void __attribute__((interrupt, auto_psv)) _T2Interrupt(void)
{
    IFS0bits.T2IF = 0u;             // Clear Timer 2 interrupt flag

    if(servo1_is_enabled)
    {
        OC1RS = servo1_pulse_width; // Write Duty Cycle value for next PWM cycle
    }

    if(servo2_is_enabled)
    {
        OC2RS = servo2_pulse_width; // Write Duty Cycle value for next PWM cycle
    }

    if(servo3_is_enabled)
    {
        OC3RS = servo3_pulse_width; // Write Duty Cycle value for next PWM cycle
    }

    if(servo4_is_enabled)
    {
        OC4RS = servo4_pulse_width; // Write Duty Cycle value for next PWM cycle
    }
}
