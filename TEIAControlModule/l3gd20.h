/* 
 * File:   l3gd20.h
 * Author: Miguel Zea
 *
 * Created on 22 de julio de 2013, 03:03 PM
 */

#ifndef L3GD20_H
#define	L3GD20_H

#ifdef	__cplusplus
extern "C" {
#endif

#ifdef L3GD20_SA0_HIGH
#define L3GD20_ADDRESS (0x6B)
#elif defined L3GD20_SA0_LOW
#define L3GD20_ADDRESS (0x6A)
#else
#define L3GD20_ADDRESS (0x6B)
#endif

#if (!defined X_AXIS) && (!defined Y_AXIS) && (!defined Z_AXIS)
#define X_AXIS  (0u)
#define Y_AXIS  (1u)
#define Z_AXIS  (2u)
#endif

#ifdef L3GD20_CALIBRATE
void
l3gd20_calibrate(void);
#endif

unsigned int
l3gd20_init_default(void);

int16_t
l3gd20_get_sample(unsigned int axis);

#ifdef	__cplusplus
}
#endif

#endif	/* L3GD20_H */

