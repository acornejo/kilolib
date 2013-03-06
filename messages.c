#include "messages.h"
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#ifdef OHC
#define IR_PORT PORTD
#define IR_DDR DDRD
#else
#define IR_PORT PORTB
#define IR_DDR DDRB
#endif

#define ir_on()  IR_DDR |= tx_maskon
#define ir_off() IR_DDR &= tx_maskoff

// Number of clock cycles used by irsend_one/zero routines,
// including the compiler setup overhead.
#define irsend_cycles 16

#define irsend_one()\
    asm volatile (\
        "in __tmp_reg__, %[port]\n\t"\
        "or __tmp_reg__, %[valon]\n\t"\
        "out %[port], __tmp_reg__\n\t"\
        "nop\n\t"\
        "nop\n\t"\
        "nop\n\t"\
        "nop\n\t"\
        "nop\n\t"\
        "nop\n\t"\
        "in __tmp_reg__, %[port]\n\t"\
        "and __tmp_reg__, %[valoff]\n\t"\
        "out %[port], __tmp_reg__\n\t"\
        : /* no outputs */\
        : [port] "I" (_SFR_IO_ADDR(IR_PORT)),\
          [valon] "r" (tx_maskon),\
          [valoff] "r" (tx_maskoff)\
    )

#define irsend_zero()\
    asm volatile (\
        "nop\n\t"\
        "nop\n\t"\
        "in __tmp_reg__, %[port]\n\t"\
        "and __tmp_reg__, %[valoff]\n\t"\
        "out %[port], __tmp_reg__\n\t"\
        "nop\n\t"\
        "nop\n\t"\
        "nop\n\t"\
        "nop\n\t"\
        "nop\n\t"\
        "nop\n\t"\
        "in __tmp_reg__, %[port]\n\t"\
        "and __tmp_reg__, %[valoff]\n\t"\
        "out %[port], __tmp_reg__\n\t"\
        "nop\n\t"\
        : /* no outputs */\
        : [port] "I" (_SFR_IO_ADDR(IR_PORT)),\
          [valoff] "r" (tx_maskoff)\
    )


static const PROGMEM uint32_t crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};

uint32_t message_crc(message_t *msg) {
    uint8_t i;
    uint8_t tbl_idx;
    uint32_t crc = 0;
    for (i = 0; i<sizeof(msg->payload)/sizeof(msg->payload[0]); i++) {
        tbl_idx = crc ^ (msg->payload[i] >> (0 * 4));
        crc = pgm_read_dword_near(crc_table + (tbl_idx & 0x0f)) ^ (crc >> 4);
        tbl_idx = crc ^ (msg->payload[i] >> (1 * 4));
        crc = pgm_read_dword_near(crc_table + (tbl_idx & 0x0f)) ^ (crc >> 4);
    }
    return ~crc;
}

uint8_t message_send(message_t *msg) {
    uint16_t k;
    uint8_t byte_idx;
    uint8_t ddr = IR_DDR;
    uint8_t sreg = SREG;
    cli();

    /* Enable IR output. */
    ir_on();

    /* Send start pulse. */
    irsend_one();
    __builtin_avr_delay_cycles(rx_bitcycles-irsend_cycles);

    /* Check for collisions. 12 cycles per iteration.
     * delay of 8 bits == 269*8/12 = 180 */
    for(k=0;k<180;k++) {
        if((ACSR & (1<<ACO))>0) {
            IR_DDR = ddr;
            SREG = sreg;
            return 0;
        }
        if((ACSR & (1<<ACO))>0) {
            IR_DDR = ddr;
            SREG = sreg;
            return 0;
        }
    }

    /* Loop over each byte in the message */
    for(byte_idx=0; byte_idx<sizeof(message_t); byte_idx++) {
        /* Add start bit to byte. */
        int16_t bitmask = 0x01;
        int16_t byteval = msg->rawdata[byte_idx]<<1 | 1;
        /* Transmit each bit. Loop overhead is 12 cycles per iteration. */
        while(bitmask <= 0x100) {
            if (byteval & bitmask)
                irsend_one();
            else
                irsend_zero();
            bitmask <<=1;
            __builtin_avr_delay_cycles(rx_bitcycles-irsend_cycles-12);
        }
    }

    ACSR |= (1<<ACI);
    IR_DDR = ddr;
    SREG = sreg;
    return 1;
}
