/**
 * @file l3gd20_registers.h
 * @author Miguel Zea
 * @date 17 de julio de 2013, 05:28 PM
 * @brief Address especification for device's registers.
 * @see
 */
#ifndef L3GD20_REGISTERS_H
#define	L3GD20_REGISTERS_H

#ifdef	__cplusplus
extern "C" {
#endif

#define WHO_AM_I        (0x0F)
#define CTRL_REG1       (0x20)
#define CTRL_REG2       (0x21)
#define CTRL_REG3       (0x22)
#define CTRL_REG4       (0x23)
#define CTRL_REG5       (0x24)
#define REFERENCE       (0x25)
#define OUT_TEMP        (0x26)
#define STATUS_REG      (0x27)
#define OUT_X_L         (0x28)
#define OUT_X_H         (0x29)
#define OUT_Y_L         (0x2A)
#define OUT_Y_H         (0x2B)
#define OUT_Z_L         (0x2C)
#define OUT_Z_H         (0x2D)
#define FIFO_CTRL_REG   (0x2E)
#define FIFO_SRC_REG    (0x2F)
#define INT1_CFG        (0x30)
#define INT1_SRC        (0x31)
#define INT1_TSH_XH     (0x32)
#define INT1_TSH_XL     (0x33)
#define INT1_TSH_YH     (0x34)
#define INT1_TSH_YL     (0x35)
#define INT1_TSH_ZH     (0x36)
#define INT1_TSH_ZL     (0x37)
#define INT1_DURATION   (0x38)

#ifdef	__cplusplus
}
#endif

#endif	/* L3GD20_REGISTERS_H */

