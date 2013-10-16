#ifndef __MESSAGES_H__
#define __MESSAGES_H__

#include <stdint.h>

typedef union {
    uint8_t rawdata[12];
    struct {
        union {
            uint8_t data[9];
            struct {
                uint8_t page_address;
                uint8_t page_offset;
                uint16_t word1;
                uint16_t word2;
                uint16_t word3;
                uint8_t unused;
            } bootmsg;
            struct {
                uint16_t id;
                int16_t x;
                int16_t y;
                int8_t theta;
                uint16_t unused;
            } gpsmsg;
        };
        uint8_t type;
        uint16_t crc;
    };
} message_t;

volatile uint8_t tx_mask;
volatile uint16_t tx_period;

uint16_t message_crc(message_t *);
uint8_t message_send(message_t *);

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
    READUID
} message_type_t;

#endif//__MESSAGES_H__
