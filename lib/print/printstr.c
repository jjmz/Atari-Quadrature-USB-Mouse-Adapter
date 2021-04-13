#include "../../include/ch554.h"
#include "small_print.h"

void printstr(__code const char *ptr)
{
//	char c;
//	while (c=*ptr++) putch(c);
    ptr;
__asm 
2$:
    clr a
    movc    a,@a+dptr
	jz	3$
4$:
	jnb	_TI, 4$
	clr	_TI
	mov	_SBUF,a
    inc dptr
    sjmp 2$
3$:
__endasm;
}
/*WAS :
 	mov	r6,dpl
	mov	r7,dph
00101$:
	mov	dpl,r6
	mov	dph,r7
	clr	a
	movc	a,@a+dptr
	mov	r5,a
	inc	dptr
	mov	r6,dpl
	mov	r7,dph
	mov	a,r5
	mov	r4,a
	jz	00104$
	mov	dpl,r4
	push	ar7
	push	ar6
	lcall	_putch
	pop	ar6
	pop	ar7
	sjmp	00101$
00104$:
*/

// Based on ASM optimisations in ch55x_sdcc
//void putch(uint8_t c)
//{
    //while (!TI); /* assumes UART is initialized */
    //TI = 0;
    //SBUF = c;	
//}

#pragma callee_saves putch
void putch(uint8_t c)
{
  c;
__asm
    1$:
    jnb _TI, 1$
    clr _TI
    mov _SBUF, dpl
__endasm;
}