/* 
 * File:   hardware_profile.h
 * Author: Miguel Zea
 *
 * Created on 20 de junio de 2013, 04:29 PM
 */

#ifndef HARDWARE_PROFILE_H
#define	HARDWARE_PROFILE_H

#define FOSC (79227500UL)   // Clock frequency 79.22MHz
#define FCY (FOSC / 2)      // Instruction clock frequency 39.61MHz

#define L3GD20_SA0_HIGH
//#define LSM303_CALIBRATE
//#define L3GD20_CALIBRATE

//#define UART1_FAST_MODE
#define I2C1_FAST_MODE

#endif	/* HARDWARE_PROFILE_H */

