#ifndef __MESSAGE_BUFFERED_H__
#define __MESSAGE_BUFFERED_H__

#include "kilolib.h"
#include "message.h"
#include "ringbuffer.h"

#ifndef RXBUFFER_SIZE
#define RXBUFFER_SIZE 16
#endif
#ifndef TXBUFFER_SIZE
#define TXBUFFER_SIZE 4
#endif

RB_create(rxbuffer, message_t, RXBUFFER_SIZE);
RB_create(rxdistbuffer, distance_measurement_t, RXBUFFER_SIZE);
RB_create(txbuffer, message_t, TXBUFFER_SIZE);

uint8_t rxbuffer_size() {
    return RB_size(rxbuffer);
}

void rxbuffer_push(message_t *msg, distance_measurement_t *dist) {
    RB_back(rxbuffer) = *msg;
    RB_pushback(rxbuffer);
    RB_back(rxdistbuffer) = *dist;
    RB_pushback(rxdistbuffer);
}

message_t *rxbuffer_peek(distance_measurement_t *dist) {
    if (RB_empty(rxbuffer))
        return '\0';
    else {
        *dist = RB_front(rxdistbuffer);
        return &RB_front(rxbuffer);
    }
}

void rxbuffer_pop() {
    if (!RB_empty(rxbuffer)) {
        RB_popfront(rxbuffer);
        RB_popfront(rxdistbuffer);
    }
}

uint8_t txbuffer_size() {
    return RB_size(txbuffer);
}

void txbuffer_push(message_t *msg) {
    RB_back(txbuffer) = *msg;
    RB_pushback(txbuffer);
}

message_t *txbuffer_peek() {
    if (RB_empty(txbuffer))
        return '\0';
    else
        return &RB_front(txbuffer);
}

void txbuffer_pop() {
    if (!RB_empty(rxbuffer))
        RB_popfront(txbuffer);
}

inline void kilo_init_buffered() {
    RB_init(rxbuffer);
    RB_init(txbuffer);
    kilo_init(rxbuffer_push, txbuffer_peek, txbuffer_pop);
}

#endif//__MESSAGE_BUFFERED_H__
