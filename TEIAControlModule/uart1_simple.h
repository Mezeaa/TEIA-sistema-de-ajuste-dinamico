/* 
 * File:   uart1_simple.h
 * Author: Miguel Zea
 *
 * Created on 20 de junio de 2013, 02:16 PM
 */

#ifndef UART1_SIMPLE_H
#define	UART1_SIMPLE_H

#ifdef __cplusplus
extern "C" {
#endif

#define LOW_SPEED_MODE  (1)
#define HIGH_SPEED_MODE (2)

#define uart1_data_available() (U1STAbits.URXDA)
#define uart1_tx_is_idle() (U1STAbits.TRMT)
#define uart1_read() (U1RXREG)

void
uart1_open(uint32_t baud_rate, int speed_mode);

void
uart1_write(uint8_t data);

void
uart1_puts(char * p_str);

void
uart1_close(void);

void
uart1_enable_rx_interrupt(void);

void
uart1_disable_rx_interrupt(void);

#ifdef __cplusplus
extern }
#endif

#endif	/* UART1_SIMPLE_H */

