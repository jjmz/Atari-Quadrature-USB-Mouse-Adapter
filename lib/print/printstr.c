#include "../../include/ch554.h"
#include "small_print.h"

void printstr(__code const char *ptr)
{
	char c;

	while (c=*ptr++)
		putch(c);
}

void putch(uint8_t c)
{
    while (!TI); /* assumes UART is initialized */
    TI = 0;
    SBUF = c;	
}
