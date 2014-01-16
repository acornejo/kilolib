#ifndef __DEBUG_H__
#define __DEBUG_H__

#ifdef DEBUG

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#ifdef NONBLOCKING
#include "ringbuffer.h"
RB_create(debug_buffer, char, 128);

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

#define debug_init_extra() {\
    RB_init(debug_buffer);\
}

#else
static int debug_putchar(char c, FILE *stream) {
    UDR0 = c;
    while(!(UCSR0A & (1<<UDRE0)));
    return 0;
}

// static int debug_getchar(FILE *stream) {
//     while(!(UCSR0A & (1<<RXC0)));
//     return UDR0;
// }

#define debug_init_extra() {}

#endif


void debug_init() {
    static FILE debug_stdout = FDEV_SETUP_STREAM(debug_putchar, NULL, _FDEV_SETUP_WRITE);
    cli();
    DDRD |= (1<<1);                                 // Set UART TxD pin as output
#ifndef BAUD
#define BAUD 38400
#endif
#include <util/setbaud.h>
    UBRR0 = UBRR_VALUE;
#if USE_2X
    UCSR0A |= (1<<U2X0);
#else
    UCSR0A &= ~(1<<U2X0);
#endif
    UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);              // No parity, 8 bits comm, 1 stop bit
    UCSR0B |= (1<<TXEN0);                           // Enable transmission
    debug_init_extra();
    stdout = &debug_stdout;
    sei();
}


#else

#define debug_init()
#define printf(...)

#endif//DEBUG

#endif//__DEBUG_H__
