/**
 * @file i2c1_master_simple.c
 * @author Miguel Zea
 * @date 30 de junio de 2013, 10:08 PM
 * @brief Implementation of a non interrupt driven module for I2C master-slave
 * communication.
 */

#include <xc.h>
#include <stdint.h>
#include "hardware_profile.h"   // Needed for operation frequency definition.
#include "i2c1_master_simple.h"

/**
 * Private function like macro that waits for the I2C module 1 bus to be idle,
 * and consequently clears the module interrupt flag.
 */
#define i2c1_wait_for_idle() do {                           \
                                while(!IFS1bits.MI2C1IF);   \
                                IFS1bits.MI2C1IF = 0;       \
                            } while(0) 

#define PGD (130UL)     ///< Pulse gobbler delay in ns.

/**
 * @copydoc i2c1_open()
 * @details
 */
void
i2c1_open(uint32_t required_fscl)
{
    if(!i2c1_is_enabled())
    {
        I2C1CONbits.I2CSIDL = 0;    // Continue module operation in idle mode.
        I2C1CONbits.SCLREL = 1;     // Release SCL1 clock (as I2C slave).
        I2C1CONbits.IPMIEN = 0;     // IPMI support mode disabled.
        I2C1CONbits.A10M = 0;       // I2C1ADD register is a 7-bit slave
                                    // address.
        I2C1CONbits.DISSLW = 0;     // Slew rate control enabled.
        I2C1CONbits.SMEN = 0;       // Disable SMBus input thresholds.

        // Baud rate generator register calculation, edited for precision.
        I2C1BRG = (((((FCY / required_fscl) * 100000)
                - ((FCY / 10000) * PGD))) / 100000) - 2;

        IEC1bits.MI2C1IE = 0;       // Disable master I2C interrupt.
        IFS1bits.MI2C1IF = 0;       // Clear master I2C1 interrupt flag.
        I2C1CONbits.I2CEN = 1;      // Enable I2C1 module.
    }
}

/**
 * @copydoc i2c1_close()
 * @details
 */
void
i2c1_close(void)
{
    if(i2c1_is_enabled())
    {
        IFS1bits.MI2C1IF = 0;
        I2C1CONbits.I2CEN = 0;
    }
}

/**
 * @copydoc i2c1_start()
 * @details
 */
void
i2c1_start(void)
{
    if(i2c1_is_enabled())
    {
        if((!I2C1CONbits.ACKEN) && (!I2C1CONbits.RCEN) && (!I2C1CONbits.PEN)
                && (!I2C1CONbits.RSEN) && (!I2C1CONbits.SEN))
        {
            I2C1CONbits.SEN = 1;
            i2c1_wait_for_idle();
        }
    }
}

/**
 * @copydoc i2c1_restart()
 * @details
 */
void
i2c1_restart(void)
{
    if(i2c1_is_enabled())
    {
        I2C1CONbits.RSEN = 1;
        i2c1_wait_for_idle();
    }
}

/**
 * @copydoc i2c1_stop()
 * @details
 */
void
i2c1_stop(void)
{
    if(i2c1_is_enabled())
    {
        I2C1CONbits.PEN = 1;
        i2c1_wait_for_idle();
    }
}

/**
 * @copydoc i2c1_send_ack()
 * @details
 */
void
i2c1_send_ack(void)
{
    if(i2c1_is_enabled())
    {
        I2C1CONbits.ACKDT = 0;
        I2C1CONbits.ACKEN = 1;
        i2c1_wait_for_idle();
    }
}

/**
 * @copydoc i2c1_send_nack()
 * @details
 */
void
i2c1_send_nack(void)
{
    if(i2c1_is_enabled())
    {
        I2C1CONbits.ACKDT = 1;
        I2C1CONbits.ACKEN = 1;
        i2c1_wait_for_idle();
    }
}

/**
 * @copydoc i2c1_write()
 * @details
 */
void
i2c1_write(uint8_t data)
{
    if(i2c1_is_enabled())
    {
        I2C1TRN = data;
        i2c1_wait_for_idle();
    }
}

/**
 * @copydoc i2c1_read()
 * @details
 */
uint8_t
i2c1_read(void)
{
    uint8_t data = 0;

    I2C1CONbits.RCEN = 1;
    i2c1_wait_for_idle();
    data = I2C1RCV;
    
    return data;
}

/**
 * @copydoc i2c1_write_device_register()
 * @details
 */
unsigned int
i2c1_write_slave_register(uint8_t device_address, uint8_t reg_address,
        uint8_t reg_data)
{
    unsigned int b_write_error;
    const uint8_t buf_size = 3;
    uint8_t i2c1_write_buf[buf_size];
    const uint8_t * p_buf;

    i2c1_write_buf[0] = (device_address << 1) & ~(0x01);
    i2c1_write_buf[1] = reg_address;
    i2c1_write_buf[2] = reg_data;
    p_buf = i2c1_write_buf;
    b_write_error = 0;

    i2c1_start();

    do
    {
        i2c1_write(*(p_buf++));
    } while((p_buf < (i2c1_write_buf + buf_size)) && (i2c1_ack_received()));

    i2c1_stop();

    if(p_buf < (i2c1_write_buf + buf_size))
    {
        b_write_error = 1;
    }

    return b_write_error;
}

/**
 * @copydoc i2c1_read_device_register()
 * @details
 */
unsigned int
i2c1_read_slave_register(uint8_t device_address, uint8_t reg_address,
        uint8_t * p_reg_data)
{
    unsigned int b_write_error;
    const uint8_t buf_size = 3;
    uint8_t i2c1_write_buf[buf_size];
    const uint8_t * p_buf;

    i2c1_write_buf[0] = (device_address << 1) & ~(0x01);
    i2c1_write_buf[1] = reg_address;
    i2c1_write_buf[2] = (device_address << 1) | (0x01);
    p_buf = i2c1_write_buf;
    b_write_error = 0;

    i2c1_start();

    do
    {
        if(p_buf == (i2c1_write_buf + (buf_size - 1)))
        {
            i2c1_restart();
        }

        i2c1_write(*(p_buf++));
    } while((p_buf < (i2c1_write_buf + buf_size)) && (i2c1_ack_received()));

    if(p_buf < (i2c1_write_buf + buf_size))
    {
        i2c1_stop();
        b_write_error = 1;
    }
    else
    {
        *p_reg_data = i2c1_read();
        i2c1_send_nack();
        i2c1_stop();
    }

    return b_write_error;
}

/**
 * @copydoc i2c1_read_device_registers()
 * @details
 */
unsigned int
i2c1_read_slave_registers(uint8_t device_address, uint8_t starting_address,
        uint8_t regs_to_read, uint8_t * p_read_buf, uint8_t read_buf_len)
{
    unsigned int b_write_error;
    const uint8_t buf_size = 3;
    uint8_t i2c1_write_buf[buf_size];
    uint8_t * p_buf;

    if(regs_to_read != read_buf_len)
    {
        b_write_error = 1;
    }
    else
    {
        i2c1_write_buf[0] = (device_address << 1) & ~(0x01);
        i2c1_write_buf[1] = starting_address;
        i2c1_write_buf[2] = (device_address << 1) | (0x01);
        p_buf = i2c1_write_buf;
        b_write_error = 0;

        i2c1_start();

        do
        {
            if(p_buf == (i2c1_write_buf + (buf_size - 1)))
            {
                i2c1_restart();
            }

            i2c1_write(*(p_buf++));
        } while((p_buf < (i2c1_write_buf + buf_size)) && (i2c1_ack_received()));

        if(p_buf < (i2c1_write_buf + buf_size))
        {
            i2c1_stop();
            b_write_error = 1;
        }
        else
        {
            p_buf = p_read_buf;
            do
            {
                *p_buf = i2c1_read();
                if(p_buf == (p_read_buf + (regs_to_read - 1)))
                {
                    i2c1_send_nack();
                }
                else
                {
                    i2c1_send_ack();
                }
                p_buf++;
            } while(p_buf < (p_read_buf + regs_to_read));

            i2c1_stop();
        }
    }

    return b_write_error;
}