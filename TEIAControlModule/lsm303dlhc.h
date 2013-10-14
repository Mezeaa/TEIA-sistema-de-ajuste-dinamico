/* 
 * File:   lsm303dlhc.h
 * Author: Miguel Zea
 *
 * Created on 16 de julio de 2013, 10:01 AM
 */

#ifndef LSM303DLHC_H
#define	LSM303DLHC_H

#ifdef	__cplusplus
extern "C" {
#endif

#define LSM303_ACC_ADDRESS  (0x19)
#define LSM303_MAG_ADDRESS  (0x1E)

#if (!defined X_AXIS) && (!defined Y_AXIS) && (!defined Z_AXIS)
#define X_AXIS  (0u)
#define Y_AXIS  (1u)
#define Z_AXIS  (2u)
#endif

#ifdef LSM303_CALIBRATE
void
lsm303_acc_calibrate(void);

void
lsm303_mag_calibrate(void);
#endif

unsigned int
lsm303_acc_init_default(void);

unsigned int
lsm303_mag_init_default(void);

int16_t
lsm303_acc_get_sample(unsigned int axis);

int16_t
lsm303_mag_get_sample(unsigned int axis);

#if 0
void
lsm303_get_acc_xyz(uint16_t * p_sample_buf);

void
lsm303_get_mag_xyz(uint16_t * p_buffer);
#endif

#ifdef	__cplusplus
}
#endif

#endif	/* LSM303DLHC_H */

