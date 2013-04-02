#include "message.h"
#include <util/crc16.h>  // for optimized crc routines

uint16_t message_crc(message_t *msg) {
    uint8_t i;
    uint16_t crc = 0xFFFF;
    for (i = 0; i<sizeof(message_t)-sizeof(msg->crc); i++)
        crc = _crc_ccitt_update(crc, msg->rawdata[i]);
    return crc;
}
