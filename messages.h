#ifndef __MESSAGES_H__
#define __MESSAGES_H__

#include <stdint.h>

#define tx_period 50000  // number of timer cycles between consecutive messages
#define rx_bitcycles 269 // number of clock cycles between consecutive bits

typedef union {
    uint8_t rawdata[12];
    struct {
        uint8_t payload[8];
        uint32_t crc_payload;
    };
    struct {
        uint8_t data[7];
        uint8_t type;
        uint32_t crc;
    };
    struct {
        uint8_t page_address;
        uint16_t page_offset;
        uint16_t word1;
        uint16_t word2;
        uint8_t type;
        uint32_t crc;
    } bootmsg;
} message_t;

volatile uint8_t tx_maskon;
volatile uint8_t tx_maskoff;

uint32_t message_crc(message_t *);
uint8_t message_send(message_t *);

typedef enum {
    NORMAL = 0,
    SLEEP,
    BOOT,
    WAKEUP,
    PAUSE,
    CHARGE,
    VOLTAGE,
    RUN,
    RESET,
    BOOTLOAD_MSG,

} message_type_t;

#endif//__MESSAGES_H__
