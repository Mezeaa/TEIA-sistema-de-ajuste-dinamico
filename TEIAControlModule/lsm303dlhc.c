#include <xc.h>
#include <stdint.h>
#include "hardware_profile.h"
#include "i2c1_master_simple.h"
#include "lsm303dlhc_registers.h"
#include "lsm303dlhc.h"

#ifdef LSM303_CALIBRATE
#include <stdio.h>
#include "uart1_simple.h"

void
lsm303_acc_calibrate(void)
{
    int16_t acc_min_x = INT16_MAX;
    int16_t acc_max_x = INT16_MIN;
    int16_t acc_min_y = INT16_MAX;
    int16_t acc_max_y = INT16_MIN;
    int16_t acc_min_z = INT16_MAX;
    int16_t acc_max_z = INT16_MIN;
    int16_t acc_sample = 0;

    do
    {
        acc_sample = lsm303_acc_get_sample(X_AXIS);
        if (acc_sample < acc_min_x) acc_min_x = acc_sample;
        if (acc_sample > acc_max_x) acc_max_x = acc_sample;

        acc_sample = lsm303_acc_get_sample(Y_AXIS);
        if (acc_sample < acc_min_y) acc_min_y = acc_sample;
        if (acc_sample > acc_max_y) acc_max_y = acc_sample;

        acc_sample = lsm303_acc_get_sample(Z_AXIS);
        if (acc_sample < acc_min_z) acc_min_z = acc_sample;
        if (acc_sample > acc_max_z) acc_max_z = acc_sample;
    } while(uart1_read() != 'a');

    printf("Calibration results\n");
    printf("X axis:\n");
    printf("\tMin %d\n", acc_min_x);
    printf("\tMax %d\n", acc_max_x);
    printf("Y axis:\n");
    printf("\tMin %d\n", acc_min_y);
    printf("\tMax %d\n", acc_max_y);
    printf("Z axis:\n");
    printf("\tMin %d\n", acc_min_z);
    printf("\tMax %d\n", acc_max_z);
}

void
lsm303_mag_calibrate(void)
{
    int16_t mag_min_x = INT16_MAX;
    int16_t mag_max_x = INT16_MIN;
    int16_t mag_min_y = INT16_MAX;
    int16_t mag_max_y = INT16_MIN;
    int16_t mag_min_z = INT16_MAX;
    int16_t mag_max_z = INT16_MIN;
    int16_t mag_sample = 0;

    do
    {
        mag_sample = lsm303_mag_get_sample(X_AXIS);
        if (mag_sample < mag_min_x) mag_min_x = mag_sample;
        if (mag_sample > mag_max_x) mag_max_x = mag_sample;

        mag_sample = lsm303_mag_get_sample(Y_AXIS);
        if (mag_sample < mag_min_y) mag_min_y = mag_sample;
        if (mag_sample > mag_max_y) mag_max_y = mag_sample;

        mag_sample = lsm303_mag_get_sample(Z_AXIS);
        if (mag_sample < mag_min_z) mag_min_z = mag_sample;
        if (mag_sample > mag_max_z) mag_max_z = mag_sample;
    } while(uart1_read() != 'm');

    printf("Calibration results\n");
    printf("X axis:\n");
    printf("\tMin %d\n", mag_min_x);
    printf("\tMax %d\n", mag_max_x);
    printf("Y axis:\n");
    printf("\tMin %d\n", mag_min_y);
    printf("\tMax %d\n", mag_max_y);
    printf("Z axis:\n");
    printf("\tMin %d\n", mag_min_z);
    printf("\tMax %d\n", mag_max_z);
}
#endif

/**
 * @copydoc lsm303_acc_init_default()
 * @details
 */
unsigned int
lsm303_acc_init_default(void)
{
    unsigned int b_acc_is_connected;

    if(i2c1_write_slave_register(LSM303_ACC_ADDRESS, CTRL_REG1_A, 0x47))
    {
        b_acc_is_connected = 0;
    } 
    else
    {
        b_acc_is_connected = 1;
        i2c1_write_slave_register(LSM303_ACC_ADDRESS, CTRL_REG4_A, 0x08);
    }

    return b_acc_is_connected;
}

/**
 * @copydoc lsm303_mag_init_default()
 * @details
 */
unsigned int
lsm303_mag_init_default(void)
{
    unsigned int b_mag_is_connected;

    if(i2c1_write_slave_register(LSM303_MAG_ADDRESS, CRA_REG_M, 0x14))
    {
        b_mag_is_connected = 0;
    }
    else
    {
        b_mag_is_connected = 1;
        i2c1_write_slave_register(LSM303_MAG_ADDRESS, CRB_REG_M, 0x20);
        i2c1_write_slave_register(LSM303_MAG_ADDRESS, MR_REG_M, 0x00);
    }

    return b_mag_is_connected;
}

/**
 * @copydoc lsm303_get_acc_sample()
 * @details
 */
int16_t
lsm303_acc_get_sample(unsigned int axis)
{
    int16_t acc_sample = 0;
    uint8_t sample_high = 0;
    uint8_t sample_low = 0;
    uint8_t address_high = 0;
    uint8_t address_low = 0;

    switch(axis)
    {
        case X_AXIS:
            address_high = OUT_X_H_A;
            address_low = OUT_X_L_A;
        break;

        case Y_AXIS:
            address_high = OUT_Y_H_A;
            address_low = OUT_Y_L_A;
        break;

        case Z_AXIS:
            address_high = OUT_Z_H_A;
            address_low = OUT_Z_L_A;
        break;

        default:
            address_high = OUT_X_H_A;
            address_low = OUT_X_L_A;
        break;
    }

    i2c1_read_slave_register(LSM303_ACC_ADDRESS, address_high, &sample_high);
    i2c1_read_slave_register(LSM303_ACC_ADDRESS, address_low, &sample_low);

    //acc_sample = ((int16_t)(sample_high << 8 | sample_low)) >> 4;

    acc_sample = ((int16_t)(sample_high << 8 | sample_low)) / 16;

    return acc_sample;
}

/**
 * @copydoc lsm303_get_mag_sample()
 * @details
 */
int16_t
lsm303_mag_get_sample(unsigned int axis)
{
    int16_t mag_sample = 0;
    uint8_t sample_high = 0;
    uint8_t sample_low = 0;
    uint8_t address_high = 0;
    uint8_t address_low = 0;

    switch(axis)
    {
        case X_AXIS:
            address_high = OUT_X_H_M;
            address_low = OUT_X_L_M;
        break;

        case Y_AXIS:
            address_high = OUT_Y_H_M;
            address_low = OUT_Y_L_M;
        break;

        case Z_AXIS:
            address_high = OUT_Z_H_M;
            address_low = OUT_Z_L_M;
        break;

        default:
            address_high = OUT_X_H_M;
            address_low = OUT_X_L_M;
        break;
    }

    i2c1_read_slave_register(LSM303_MAG_ADDRESS, address_high, &sample_high);
    i2c1_read_slave_register(LSM303_MAG_ADDRESS, address_low, &sample_low);

    //mag_sample = ((int16_t)(sample_high << 8 | sample_low)) >> 4;
    mag_sample = ((int16_t)(sample_high << 8 | sample_low));

    return mag_sample;
}
