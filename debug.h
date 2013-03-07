#ifndef __DEBUG_H__
#define __DEBUG_H__

#ifdef DEBUG

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "ringbuffer.h"

#define BAUDRATE 19200

RB_create(debug_buffer, char, 511);
volatile uint8_t debug_empty = 1;

static int debug_putchar(char c, FILE *stream) {
    if (RB_full(debug_buffer)) {
        return -1;
    } else {
        RB_back(debug_buffer) = c;
        RB_pushback(debug_buffer);
        UCSR0B |= (1<<UDRIE0);
        return 0;
    }
}

ISR(USART_UDRE_vect) {
    if (RB_empty(debug_buffer)) {
        UCSR0B &= ~(1<<UDRIE0);
    } else {
        UDR0 = RB_front(debug_buffer);
        RB_popfront(debug_buffer);
    }
}

/* static FILE debug_stdout = FDEV_SETUP_STREAM(debug_putchar, NULL, _FDEV_SETUP_WRITE); */

inline void debug_init() {
	DDRD |= (1<<1);                                 // Set UART TxD pin as output
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);              // No parity, 8 bits comm, 1 stop bit
	UCSR0B |= (1<<TXEN0);                           // Enable transmission
	UCSR0A = 0;
    UBRR0 = ((F_CPU/(BAUDRATE*16UL))-1);
    RB_init(debug_buffer);
    // stdout = &debug_stdout;
    // printf("\n");
    // printf("+-----------------+\n");
    // printf("| Kilobot Started |\n");
    // printf("+-----------------+\n");
}


#else

#define debug_init()
#define printf(...)

#endif//DEBUG

#endif//__DEBUG_H__
