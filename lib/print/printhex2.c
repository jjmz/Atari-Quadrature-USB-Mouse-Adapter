#include "../../include/ch554.h"
#include "small_print.h"

const char hextab[]="0123456789ABCDEF";

void printhex2(uint8_t h)
{
	putch(hextab[h>>4]);
	putch(hextab[h&0xF]);
}

void printx2(uint8_t h)
{
	putch('0');putch('x');
	printhex2(h);putch(' ');
}

