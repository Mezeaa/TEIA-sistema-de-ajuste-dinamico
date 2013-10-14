#include <xc.h>
#include <stdint.h>
#include "hardware_profile.h"
#include "uart1_simple.h"

void
uart1_open(uint32_t baud_rate, int speed_mode)
{
    U1MODE = 0x0000;
    U1STA = 0x0000;

    if(LOW_SPEED_MODE == speed_mode)
    {
        U1MODEbits.BRGH = 0U;
        U1BRG = (FCY/(16 * baud_rate)) - 1;
    }
    else if(HIGH_SPEED_MODE == speed_mode)
    {
        U1MODEbits.BRGH = 1U;
        U1BRG = (FCY/(4 * baud_rate)) - 1;
    }

    U1MODEbits.UARTEN = 1;
    U1STAbits.UTXEN = 1;
}

void
uart1_write(uint8_t data)
{
    while(!uart1_tx_is_idle())
    {

    }
    
    U1TXREG = data;
}

void
uart1_puts(char * p_str)
{
    while(*p_str != '\0')
    {
        uart1_write(*(uint8_t *)p_str);
        p_str++;
    }
}

void
uart1_close(void)
{
    U1MODEbits.UARTEN = 0;
    U1STAbits.UTXEN = 0;
}

void
uart1_enable_rx_interrupt(void)
{
    U1STAbits.URXISEL = 0x1;
    IFS0bits.U1RXIF = 0U;
    IEC0bits.U1RXIE = 1U;
}

void
uart1_disable_rx_interrupt(void)
{
    IEC0bits.U1RXIE = 0U;
    IFS0bits.U1RXIF = 0U;
}