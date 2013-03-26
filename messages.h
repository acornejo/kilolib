#ifndef __MESSAGES_H__
#define __MESSAGES_H__

#include <stdint.h>
#include <util/crc16.h>

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
        };
        uint8_t type;
        uint16_t crc;
    };
} message_t;

volatile uint8_t tx_maskon;
volatile uint8_t tx_maskoff;

uint16_t message_crc(message_t *msg) {
    uint8_t i;
    uint16_t crc = 0xFFFF;
    for (i = 0; i<sizeof(message_t)-sizeof(msg->crc); i++)
        crc = _crc_ccitt_update(crc, msg->rawdata[i]);
    return crc;
}

uint8_t message_send(message_t *);

typedef enum {
    NORMAL = 0,
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
} message_type_t;

/* Number of clock cycles per bit. */
#define rx_bitcycles 256
/* Number of clock cycles for an entire message. */
#define rx_msgcycles (10*rx_bitcycles*(sizeof(message_t)+1))
/* Number of timer cycles between consecutive messages. */
#define tx_period 3906

#endif//__MESSAGES_H__
