#ifndef __OHC_H__
#define __OHC_H__

#include <stdint.h>
#include "message.h"

// 01010101
#define PACKET_HEADER 0x55
#define PAGE_SIZE 128
#define PACKET_SIZE (PAGE_SIZE+4)
enum {
    PACKET_STOP,
    PACKET_LEDTOGGLE,
    PACKET_FORWARDMSG,
    PACKET_FORWARDRAWMSG,
    PACKET_BOOTPAGE,
    PACKET_GPSFRAME,
    PACKET_FORWARDMSGSINGLE,
};

typedef struct {
    uint16_t id;
    int16_t x;
    int16_t y;
    int8_t theta;
    uint16_t unused;
} gpsmsg_t;

typedef struct {
    uint8_t mode;
    uint16_t uid;
    uint8_t turn_left;
    uint8_t turn_right;
    uint8_t straight_left;
    uint8_t straight_right;
    uint16_t unused;
} calibmsg_t;

enum {
    CALIB_SAVE,
    CALIB_UID,
    CALIB_TURN_LEFT,
    CALIB_TURN_RIGHT,
    CALIB_STRAIGHT
};

#endif//__OHC_H__
