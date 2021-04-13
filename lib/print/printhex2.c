#include "../../include/ch554.h"
#include "small_print.h"

const char hextab[16]={'0','1','2','3',
                       '4','5','6','7',
					   '8','9','A','B',
					   'C','D','E','F' };

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

