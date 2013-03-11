#include "messages.h"
#include <avr/interrupt.h>
#include <util/crc16.h>

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


uint16_t message_crc(message_t *msg) {
    uint8_t i;
    uint16_t crc = 0xFFFF;
    for (i = 0; i<sizeof(message_t)-sizeof(msg->crc); i++)
        crc = _crc_ccitt_update(crc, msg->rawdata[i]);
    return crc;
}

#ifndef BOOTLOADER // not required in bootloader
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

    /* Check for collisions for the 8 data bits (8 cycles per iteration).
     * delay for 8 bits == 8 * rx_bitcycles/ cycles per iteration = rx_bitcycles. */
    for(k=0; k<rx_bitcycles; k++) {
        if((ACSR & (1<<ACO))>0) {
            IR_DDR = ddr;
            SREG = sreg;
            return 0;
        }
    }

    /* Send stop bit. */
    irsend_one();
    __builtin_avr_delay_cycles(rx_bitcycles-irsend_cycles);

    /* Loop over each byte in the message */
    for(byte_idx=0; byte_idx<sizeof(message_t); byte_idx++) {
        /* 1 Start bit, 8 data bits, 1 stop bit. */
        int16_t byteval = msg->rawdata[byte_idx]<<1 | (1<<0) | (1<<9);
        int16_t bitmask = (1<<0);
        /* Transmit each bit. Loop overhead is 12 cycles per iteration. */
        while(bitmask <= (1<<9)) {
            if (byteval & bitmask)
                irsend_one();
            else
                irsend_zero();
            bitmask <<= 1;
            __builtin_avr_delay_cycles(rx_bitcycles-irsend_cycles-12);
        }
    }

    ACSR |= (1<<ACI);
    IR_DDR = ddr;
    SREG = sreg;
    return 1;
}
#endif
