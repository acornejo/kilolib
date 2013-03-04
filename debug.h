#ifndef __DEBUG_H__
#define __DEBUG_H__

#ifdef DEBUG

#include <stdio.h>

static int debug_putchar(char c, FILE *stream) {
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
    return 0;
}

static FILE debug_stdout = FDEV_SETUP_STREAM(debug_putchar, NULL, _FDEV_SETUP_WRITE);

inline void debug_init() {
	DDRD |= (1<<1);
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);              // No parity, 8 bits comm, 1 stop bit
	UCSR0B |= (1<<TXEN0);                           // Enable transmission
	UCSR0A = 0;
    UBRR0 = ((F_CPU/(19200*16UL))-1);
    stdout = &debug_stdout;
    printf("Kilobot Started\n");
}

#else

inline void debug_init() {}

void printf(const char *fmt, ...) {}

#endif//DEBUG

#endif//__DEBUG_H__
