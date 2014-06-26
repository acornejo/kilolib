#ifndef __MESSAGE_BUFFERED_H__
#define __MESSAGE_BUFFERED_H__

#include "kilolib.h"
#include "ringbuffer.h"

#ifndef RXBUFFER_SIZE
#define RXBUFFER_SIZE 16
#endif
#ifndef TXBUFFER_SIZE
#define TXBUFFER_SIZE 4
#endif

typedef struct {
    message_t msg;
    distance_measurement_t dist;
} received_message_t;

RB_create(rxbuffer, received_message_t, RXBUFFER_SIZE);
RB_create(txbuffer, message_t, TXBUFFER_SIZE);

uint8_t rxbuffer_size() {
    return RB_size(rxbuffer);
}

void rxbuffer_push(message_t *msg, distance_measurement_t *dist) {
    received_message_t *rmsg = &RB_back(rxbuffer);
    rmsg->msg = *msg;
    rmsg->dist = *dist;
    RB_pushback(rxbuffer);
}

message_t *rxbuffer_peek(distance_measurement_t *dist) {
    if (RB_empty(rxbuffer))
        return '\0';
    else {
        received_message_t *rmsg = &RB_front(rxbuffer);
        *dist = rmsg->dist;
        return &rmsg->msg;
    }
}

void rxbuffer_pop() {
    if (!RB_empty(rxbuffer)) {
        RB_popfront(rxbuffer);
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
    if (!RB_empty(txbuffer))
        RB_popfront(txbuffer);
}

inline void kilo_message_buffered() {
    RB_init(rxbuffer);
    RB_init(txbuffer);
    kilo_message_rx = rxbuffer_push;
    kilo_message_tx = txbuffer_peek;
    kilo_message_tx_success = txbuffer_pop;
}

#endif//__MESSAGE_BUFFERED_H__
