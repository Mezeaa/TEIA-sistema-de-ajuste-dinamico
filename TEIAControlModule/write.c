#include <stdio.h>
#include <stdint.h>
#include "uart1_simple.h"

int
write(int handle, void *buffer, unsigned int len)
{
    int i;

    switch(handle)
    {
        case 0:
        case 1:
        case 2:
            for(i = len; i; --i)
            {
                uart1_write(*(uint8_t *)buffer++);
            }
            break;

        default:
            break;
    }
    return (len);
}