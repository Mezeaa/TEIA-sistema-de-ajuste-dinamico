/* 
 * File:   servo_ocpwm.h
 * Author: Miguel Zea
 *
 * Created on 15 de agosto de 2013, 09:48 AM
 */

#ifndef SERVO_OCPWM_H
#define	SERVO_OCPWM_H

#ifdef	__cplusplus
extern "C" {
#endif

#define TIMER2_PRESCALER (8U)
#define OC1_SERVO (~0x0001)
#define OC2_SERVO (~0x0002)
//#define OC3_SERVO (~0x0004)
//#define OC4_SERVO (~0x0008)
#define OC_SERVO_MIN_DUTY_CYCLE (((FCY / TIMER2_PRESCALER) / 1250U) + 1U) //(3962u)
#define OC_SERVO_MAX_DUTY_CYCLE ((FCY / TIMER2_PRESCALER) / 455U) //(10893u)
#define OC_SERVO_RESOLUTION (OC_SERVO_MAX_DUTY_CYCLE - OC_SERVO_MIN_DUTY_CYCLE)
#define OC_SERVO_NEUTRAL (7428U)
#define OC_SERVO_DEADBAND (10U)

void
servo_ocpwm_init(unsigned int servos_to_enable);

bool
servo_ocpwm_set_duty_cycle(unsigned int servo_id, uint16_t duty_cycle);

uint16_t
servo_ocpwm_get_duty_cycle(unsigned int servo_id);

bool
servo_ocpwm_is_servo_enabled(unsigned int servo_id);

#ifdef	__cplusplus
}
#endif

#endif	/* SERVO_OCPWM_H */

