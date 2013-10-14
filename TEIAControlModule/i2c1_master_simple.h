/**
 * @file i2c1_master_simple.h
 * @author Miguel Zea
 * @date 30 de junio de 2013, 10:08 PM
 * @brief Non interrupt driven module for I2C master-slave communication.
 */

#ifndef I2C1_MASTER_SIMPLE_H
#define	I2C1_MASTER_SIMPLE_H

#ifdef	__cplusplus
extern "C" {
#endif

/**
 * Public function like macro that returns the status of I2C module 1.
 */
#define i2c1_is_enabled() (I2C1CONbits.I2CEN)

/**
 * Public function like macro that returns whether an acknowledge was received
 * or not.
 */
#define i2c1_ack_received() (~I2C1STATbits.ACKSTAT)

/**
 * @brief Enables I2C module 1 for master-slave communication.
 * @param[in] required_fscl Required clock rate (low & high speed unavailable).
 * @return Nothing.
 */
void
i2c1_open(uint32_t required_fscl);

/**
 * @brief Disables I2C module 1 and clears its interrupt flag.
 * @param None.
 * @return Nothing.
 */
void
i2c1_close(void);

/**
 * @brief Issues a START condition and waits for the bus to be idle.
 * @param None.
 * @return Nothing.
 */
void
i2c1_start(void);

/**
 * @brief Issues a repeated START condition and waits for the bus to be idle.
 * @param None.
 * @return Nothing.
 */
void
i2c1_restart(void);

/**
 * @brief Issues a STOP condition and waits for the bus to be idle.
 * @param None.
 * @return Nothing.
 */
void
i2c1_stop(void);

/**
 * @brief Initiates an acknowledge sequence (master mode receive operation).
 * @param None.
 * @return Nothing.
 */
void
i2c1_send_ack(void);

/**
 * @brief Initiates a no acknowledge sequence (master mode receive operation).
 * @param None.
 * @return Nothing.
 */
void
i2c1_send_nack(void);

/**
 * @brief Sends data byte via the I2C module 1 bus.
 * @param[in] data Data byte to be sent.
 * @return Nothing.
 */
void
i2c1_write(uint8_t data);

/**
 * @brief Reads a data byte from the I2C module 1 bus.
 * @param None.
 * @return Received data byte.
 */
uint8_t
i2c1_read(void);

/**
 * @brief Writes a byte to a standard I2C slave device.
 * 
 * This function is able to write data to an I2C slave that conforms to the 
 * following (single byte) transmit sequence:
 * Master   | ST    | SAD+W |       | SUB   |       | DATA  |       | SP
 * Slave    |       |       | SACK  |       | SACK  |       | SACK  |
 * For non conformant devices a particular write function must be specified
 * using the more general i2c1 bus functions.
 *
 * @param[in] device_address Target I2C device 7 bit slave address.
 * @param[in] reg_address Address of the register to write.
 * @param[in] reg_data Data byte to write.
 * @return 0 if the write operation was successful.
 *         1 if an error ocurred during I2C communication.
 */
unsigned int
i2c1_write_slave_register(uint8_t device_address, uint8_t reg_address,
        uint8_t reg_data);

/**
 * @brief Reads a byte from a standard I2C slave device.
 *
 * This function is able to read data from an I2C slave that conforms to the
 * following (single byte) receive sequence:
 * Master   | ST    | SAD+W |       | SUB   |       | SR    | SAD+R |       |
 * Slave    |       |       | SACK  |       | SACK  |       |       | SACK  |...
 * -----------------------------------------------------------------------------
 *          |       | NMACK | SP
 * ...      | DATA  |       |
 * For non conformant devices a particular read function must be specified
 * using the more general i2c1 bus functions.
 *
 * @param[in] device_address Target I2C device 7 bit slave address.
 * @param[in] reg_address Addres of the register to read.
 * @param[in,out] p_reg_data Memory location to store the data byte read.
 * @return 0 if the read operation was successful.
 *         1 if an error ocurred during I2C communication.
 */
unsigned int
i2c1_read_slave_register(uint8_t device_address, uint8_t reg_address,
        uint8_t * p_reg_data);

/**
 * @brief Reads a specified number of (consecutive) registers from a standard
 * I2C slave device.
 *
 * This function is able to read data from an I2C slave that conforms to the
 * following receive sequence:
 * Master   | ST    | SAD+W |       | SUB   |       | SR    | SAD+R |       |
 * Slave    |       |       | SACK  |       | SACK  |       |       | SACK  |...
 * -----------------------------------------------------------------------------
 *          |       | MACK  |       | MACK  |       | NMACK | SP
 * ...      | DATA  |       | DATA  |       | DATA  |       |
 * For non conformant devices a particular read function must be specified
 * using the more general i2c1 bus functions.
 *
 * @param[in] device_address Target I2C device 7 bit slave address.
 * @param[in] starting_address Addres of the first register to read.
 * @param[in] regs_to_read Number of registers to read.
 * @param[in,out] p_read_buf Starting location of the array to store the data
 *                read.
 * @param[in] read_buf_len Length of the array to store the data read.
 * @return 0 if the read operation was successful.
 *         1 if an error ocurred during I2C communication.
 */
unsigned int
i2c1_read_slave_registers(uint8_t device_address, uint8_t starting_address,
        uint8_t regs_to_read, uint8_t * p_read_buf, uint8_t read_buf_len);

#ifdef	__cplusplus
}
#endif

#endif	/* I2C1_MASTER_SIMPLE_H */

