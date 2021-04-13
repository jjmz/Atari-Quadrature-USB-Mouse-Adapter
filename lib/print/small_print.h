#include <stdint.h>

#pragma callee_saves putch,printstr

void printstr(__code const char *);

void printhex2(uint8_t);
void printx2(uint8_t);
void printhex4(uint16_t);
void printlf();

void putch(uint8_t);