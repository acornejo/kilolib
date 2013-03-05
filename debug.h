#ifndef __DEBUG_H__
#define __DEBUG_H__

#ifdef DEBUG

#include <stdio.h>
#include "ringbuffer.h"

RB_create(debug_buffer, char, 128);
volatile uint8_t debug_empty = 1;

static int debug_putchar(char c, FILE *stream) {
    if (debug_empty) {
        UDR0 = c;
        debug_empty = 0;
    } else {
        UCSR0B &= ~(1<<TXCIE0);
        RB_back(debug_buffer) = c;
        RB_pushback(debug_buffer);
        UCSR0B |= (1<<TXCIE0);
    }
    return 0;
}

ISR(USART_TX_vect) {
    if (RB_empty(debug_buffer)) {
        debug_empty = 1;
    } else {
        UDR0 = RB_front(debug_buffer);
        RB_popfront(debug_buffer);
    }
}

static FILE debug_stdout = FDEV_SETUP_STREAM(debug_putchar, NULL, _FDEV_SETUP_WRITE);

inline void debug_init() {
	DDRD |= (1<<1);
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);              // No parity, 8 bits comm, 1 stop bit
	UCSR0B |= (1<<TXEN0);                           // Enable transmission
	UCSR0A = 0;
    UBRR0 = ((F_CPU/(19200*16UL))-1);
    stdout = &debug_stdout;
    RB_init(debug_buffer);
    printf("Kilobot Started v %d\n", tx_maskon);
}

#else

inline void debug_init() {}

void printf(const char *fmt, ...) {}

#endif//DEBUG

#endif//__DEBUG_H__
