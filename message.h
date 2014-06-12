#ifndef __MESSAGES_H__
#define __MESSAGES_H__

#include <stdint.h>

typedef struct {
    uint8_t data[9];
    uint8_t type;
    uint16_t crc;
} message_t;

typedef enum {
    NORMAL = 0,
    GPS,
    SPECIAL = 0x80,
    BOOT = 0x80,
    BOOTPGM_PAGE,
    BOOTPGM_SIZE,
    RESET,
    SLEEP,
    WAKEUP,
    CHARGE,
    VOLTAGE,
    RUN,
    READUID,
    CALIB,
} message_type_t;

#endif//__MESSAGES_H__
