#include <xc.h>
#include <stdint.h>
#include "hardware_profile.h"
#include "i2c1_master_simple.h"
#include "l3gd20_registers.h"
#include "l3gd20.h"

#ifdef L3GD20_CALIBRATE
#include <stdio.h>
#include <libpic30.h>
#include "uart1_simple.h"

void
l3gd20_calibrate(void)
{
    int32_t gyro_zoff_x = 0;
    int32_t gyro_zoff_y = 0;
    int32_t gyro_zoff_z = 0;
    uint16_t average_counter = 0;
    const uint16_t average_samples = 128;

    while(uart1_read() != 'g')
    {

    }

    for(average_counter = 0; average_counter < average_samples;
            average_counter++)
    {
        gyro_zoff_x += l3gd20_get_sample(X_AXIS);
        __delay_ms(11);
    }

    for(average_counter = 0; average_counter < average_samples;
            average_counter++)
    {
        gyro_zoff_y += l3gd20_get_sample(Y_AXIS);
        __delay_ms(11);
    }

    for(average_counter = 0; average_counter < average_samples;
            average_counter++)
    {
        gyro_zoff_z += l3gd20_get_sample(Z_AXIS);
        __delay_ms(11);
    }

    gyro_zoff_x /= average_samples;
    gyro_zoff_y /= average_samples;
    gyro_zoff_z /= average_samples;

    printf("Calibration results\n");
    printf("X axis:\n");
    printf("\tZero offset %ld\n", gyro_zoff_x);
    printf("Y axis:\n");
    printf("\tZero offset %ld\n", gyro_zoff_y);
    printf("Z axis:\n");
    printf("\tZero offset %ld\n", gyro_zoff_z);
}
#endif

/**
 * @copydoc l3gd20_init_default()
 * @details
 */
unsigned int
l3gd20_init_default(void)
{
    unsigned int b_gyro_is_connected;

    if(i2c1_write_slave_register(L3GD20_ADDRESS, CTRL_REG1, 0x0F))
    {
        b_gyro_is_connected = 0;
    }
    else
    {
        b_gyro_is_connected = 1;
        i2c1_write_slave_register(L3GD20_ADDRESS, CTRL_REG4, 0x00);
    }

    return b_gyro_is_connected;
}

/**
 * @copydoc lsm303_get_acc_sample()
 * @details
 */
int16_t
l3gd20_get_sample(unsigned int axis)
{
    int16_t gyro_sample = 0;
    uint8_t sample_high = 0;
    uint8_t sample_low = 0;
    uint8_t address_high = 0;
    uint8_t address_low = 0;

    switch(axis)
    {
        case X_AXIS:
            address_high = OUT_X_H;
            address_low = OUT_X_L;
        break;

        case Y_AXIS:
            address_high = OUT_Y_H;
            address_low = OUT_Y_L;
        break;

        case Z_AXIS:
            address_high = OUT_Z_H;
            address_low = OUT_Z_L;
        break;

        default:
            address_high = OUT_X_H;
            address_low = OUT_X_L;
        break;
    }

    i2c1_read_slave_register(L3GD20_ADDRESS, address_high, &sample_high);
    i2c1_read_slave_register(L3GD20_ADDRESS, address_low, &sample_low);

    //gyro_sample = ((int16_t)(sample_high << 8 | sample_low)) >> 4;
    gyro_sample = ((int16_t)(sample_high << 8 | sample_low));

    return gyro_sample;
}