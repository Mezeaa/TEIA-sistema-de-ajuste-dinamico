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
#include "servo_ocpwm.h"

static bool servo1_is_enabled = false;
static bool servo2_is_enabled = false;
//static bool servo3_is_enabled = false;
//static bool servo4_is_enabled = false;
static uint16_t servo1_duty_cycle = OC_SERVO_MIN_DUTY_CYCLE;
static uint16_t servo2_duty_cycle = OC_SERVO_MIN_DUTY_CYCLE;
//static uint16_t servo3_duty_cycle = OC_SERVO_MIN_DUTY_CYCLE;
//static uint16_t servo4_duty_cycle = OC_SERVO_MIN_DUTY_CYCLE;

void
servo_ocpwm_init(unsigned int servos_to_enable)
{
    if((~servos_to_enable) & (~OC1_SERVO))
    {
        servo1_is_enabled = true;

        OC1CONbits.OCM = 0u;                // Disable Output Compare Module
        OC1R = OC_SERVO_MIN_DUTY_CYCLE;     // Write the duty cycle for the first PWM pulse
        OC1RS = OC_SERVO_MIN_DUTY_CYCLE;    // Write the duty cycle for the second PWM pulse
        OC1CONbits.OCTSEL = 0u;             // Select Timer 2 as output compare time base
        OC1R = OC_SERVO_MIN_DUTY_CYCLE;     // Load the Compare Register Value
        OC1CONbits.OCM = 0x6;               // Select the Output Compare mode
    }
    
    if((~servos_to_enable) & (~OC2_SERVO))
    {
        servo2_is_enabled = true;

        OC2CONbits.OCM = 0u;                // Disable Output Compare Module
        OC2R = OC_SERVO_MIN_DUTY_CYCLE;     // Write the duty cycle for the first PWM pulse
        OC2RS = OC_SERVO_MIN_DUTY_CYCLE;    // Write the duty cycle for the second PWM pulse
        OC2CONbits.OCTSEL = 0u;             // Select Timer 2 as output compare time base
        OC2R = OC_SERVO_MIN_DUTY_CYCLE;     // Load the Compare Register Value
        OC2CONbits.OCM = 0x6;               // Select the Output Compare mode
    }

#if 0
    if((~servos_to_enable) & (~OC3_SERVO))
    {
        servo3_is_enabled = true;

        OC3CONbits.OCM = 0u;                // Disable Output Compare Module
        OC3R = OC_SERVO_MIN_DUTY_CYCLE;     // Write the duty cycle for the first PWM pulse
        OC3RS = OC_SERVO_MIN_DUTY_CYCLE;    // Write the duty cycle for the second PWM pulse
        OC3CONbits.OCTSEL = 0u;             // Select Timer 2 as output compare time base
        OC3R = OC_SERVO_MIN_DUTY_CYCLE;     // Load the Compare Register Value
        OC3CONbits.OCM = 0x6;               // Select the Output Compare mode
    }
    
    if((~servos_to_enable) & (~OC4_SERVO))
    {
        servo4_is_enabled = true;

        OC4CONbits.OCM = 0u;                // Disable Output Compare Module
        OC4R = OC_SERVO_MIN_DUTY_CYCLE;     // Write the duty cycle for the first PWM pulse
        OC4RS = OC_SERVO_MIN_DUTY_CYCLE;    // Write the duty cycle for the second PWM pulse
        OC4CONbits.OCTSEL = 0u;             // Select Timer 2 as output compare time base
        OC4R = OC_SERVO_MIN_DUTY_CYCLE;     // Load the Compare Register Value
        OC4CONbits.OCM = 0x6;               // Select the Output Compare mode
    }
#endif

    T2CONbits.TON = 0u;     // Disable Timer
    T2CONbits.TCS = 0u;     // Select internal instruction cycle clock
    T2CONbits.TGATE = 0u;   // Disable Gated Timer mode
    T2CONbits.T32 = 0u;     // Disable 32-bit timer
    T2CONbits.TCKPS = 0x1;  // Select 1:8 Prescaler
    TMR2 = 0u;              // Clear timer register
                            // Load the period value (300 Hz) (16505U)
    PR2 = ((FCY / TIMER2_PRESCALER) / 333U) + 1U;

    IPC1bits.T2IP = 0x2;    // Set Timer 2 Interrupt Priority Level
    IFS0bits.T2IF = 0u;     // Clear Timer 2 Interrupt Flag
    IEC0bits.T2IE = 1u;     // Enable Timer 2 interrupt
    T2CONbits.TON = 1u;     // Start Timer
}

bool
servo_ocpwm_set_duty_cycle(unsigned int servo_id, uint16_t duty_cycle)
{
    bool operation_is_valid = true;
    
    if((duty_cycle < OC_SERVO_MIN_DUTY_CYCLE) ||
            (duty_cycle > OC_SERVO_MAX_DUTY_CYCLE))
    {
        operation_is_valid = false;
    }

    if(operation_is_valid)
    {
        switch(servo_id)
        {
            case OC1_SERVO:
                servo1_duty_cycle = duty_cycle;
                break;
            case OC2_SERVO:
                servo2_duty_cycle = duty_cycle;
                break;
#if 0
            case OC3_SERVO:
                servo3_duty_cycle = duty_cycle;
                break;
            case OC4_SERVO:
                servo4_duty_cycle = duty_cycle;
                break;
#endif
            default:
                operation_is_valid = false;
                break;
        }
    }

    return operation_is_valid;
}

uint16_t
servo_ocpwm_get_duty_cycle(unsigned int servo_id)
{
    uint16_t duty_cycle = 0u;

    switch(servo_id)
    {
        case OC1_SERVO:
            duty_cycle = servo1_duty_cycle;
            break;
        case OC2_SERVO:
            duty_cycle = servo2_duty_cycle;
            break;
#if 0
        case OC3_SERVO:
            duty_cycle = servo3_duty_cycle;
            break;
        case OC4_SERVO:
            duty_cycle = servo4_duty_cycle;
            break;
#endif
        default:
            duty_cycle = 0u;
            break;
    }

    return duty_cycle;
}

bool
servo_ocpwm_is_servo_enabled(unsigned int servo_id)
{
    bool servo_is_enabled = false;

    switch(servo_id)
    {
        case OC1_SERVO:
            servo_is_enabled = servo1_is_enabled;
            break;
        case OC2_SERVO:
            servo_is_enabled = servo2_is_enabled;
            break;
#if 0
        case OC3_SERVO:
            servo_is_enabled = servo3_is_enabled;
            break;
        case OC4_SERVO:
            servo_is_enabled = servo4_is_enabled;
            break;
#endif
        default:
            servo_is_enabled = false;
            break;
    }

    return servo_is_enabled;
}

void __attribute__((interrupt, auto_psv)) _T2Interrupt(void)
{
    IFS0bits.T2IF = 0u;             // Clear Timer 2 interrupt flag

    if(servo1_is_enabled)
    {
        OC1RS = servo1_duty_cycle;  // Write Duty Cycle value for next PWM cycle
    }

    if(servo2_is_enabled)
    {
        OC2RS = servo2_duty_cycle;  // Write Duty Cycle value for next PWM cycle
    }

#if 0
    if(servo3_is_enabled)
    {
        OC3RS = servo3_duty_cycle;  // Write Duty Cycle value for next PWM cycle
    }

    if(servo4_is_enabled)
    {
        OC4RS = servo4_duty_cycle;  // Write Duty Cycle value for next PWM cycle
    }
#endif
}