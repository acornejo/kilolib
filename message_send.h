#ifndef __MESSAGE_SEND_H__
#define __MESSAGE_SEND_H__

#include "message.h"

extern volatile uint8_t tx_mask;
uint8_t message_send(const message_t *);

#endif//__MESSAGE_SEND_H__
