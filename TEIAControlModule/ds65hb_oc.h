/* 
 * File:   ds65hb_oc.h
 * Author: Miguel Zea
 *
 * Created on 18 de agosto de 2013, 07:57 PM
 */

#ifndef DS65HB_OC_H
#define	DS65HB_OC_H

#ifdef	__cplusplus
extern "C" {
#endif

#define DS65HB_OC1 (~0x0001)
#define DS65HB_OC2 (~0x0002)
#define DS65HB_OC3 (~0x0004)
#define DS65HB_OC4 (~0x0008)
#define DS65HB_NEUTRAL (7430U)
#define DS65HB_LEFT (4950U)
#define DS65HB_RIGHT (9910U)
#define DS65HB_DEADBAND (10U)
#define DS65HB_MIN_PULSE_WIDTH (3970U)
#define DS65HB_MAX_PULSE_WIDTH (10890U)
#define DS65HB_RESOLUTION (6920U)

void
ds65hb_oc_init(unsigned int servos_to_enable);

unsigned int
ds65hb_oc_set_pulse_width(unsigned int servo_id, uint16_t pulse_width);

uint16_t
ds65hb_oc_get_pulse_width(unsigned int servo_id);

#ifdef	__cplusplus
}
#endif

#endif	/* DS65HB_OC_H */

