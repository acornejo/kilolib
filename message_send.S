#define __SFR_OFFSET 0
#include <avr/io.h>

#define rx_bitcycles  269
#define irsend_cycles 16

#ifdef OHC
#ifdef ARDUINO
#define IR_PORT PORTB
#define IR_DDR DDRB
#else
#define IR_PORT PORTD
#define IR_DDR DDRD
#endif
#else
#define IR_PORT PORTB
#define IR_DDR DDRB
#endif

#define sreg     r0
#define tmpreg1  r18
#define tmpreg2  r19
#define bitmaskL r20
#define bitmaskH r21
#define bytevalL r22
#define bytevalH r23
#define byteidx  r24
#define bitidx   r25
#define ddrreg   r26

.extern tx_mask

.section .text

.macro irsend
    lds tmpreg2, tx_mask  ;  [1]
    cpi tmpreg1, 0        ;  [2]
    breq 1f               ;  [3]
    nop                   ;  [4]
    in tmpreg1, IR_PORT   ;  [5]
    or tmpreg1, tmpreg2   ;  [6]
    out IR_PORT, tmpreg1  ;  [7]
    rjmp 2f               ;  [8]
1:
    nop                   ;  [5]
    nop                   ;  [6]
    nop                   ;  [7]
    rjmp 2f               ;  [8]
2:
    nop                   ;  [10]
    nop                   ;  [11]
    nop                   ;  [12]
    com tmpreg2           ;  [13]
    in tmpreg1, IR_PORT   ;  [14]
    and tmpreg1, tmpreg2  ;  [15]
    out IR_PORT, tmpreg1  ;  [16]
.endm                     ;  total = 16 cycles

.macro return retval
    ldi r24, \retval
    ret
.endm

.macro delay_cycles cycles
.if \cycles%3 == 0
    ldi tmpreg1, \cycles/3
.elseif \cycles%3 == 1
    nop
    ldi tmpreg1, \cycles/3
.else
    nop
    nop
    ldi tmpreg1, \cycles/3-1
.endif
1:
    dec tmpreg1
    brne 1b
.endm

.global message_send

message_send:
    ; save interrupt state
    ; sreg = SREG
    in sreg, SREG
#ifndef OHC
    cli
#endif
    ; ddreg = IR_DDR
    in  ddrreg,  IR_DDR

    ; IR_DDR |= tx_mask
    lds tmpreg1, tx_mask
    or tmpreg1, ddrreg
    out IR_DDR, tmpreg1

    ldi tmpreg1, 0x01
    irsend
    delay_cycles (2*rx_bitcycles-irsend_cycles)

    ldi tmpreg1, rx_bitcycles*7/8
checkcollision:
    ; if (ACSR&(1<<ACO)) == 0 goto nocollision
    in tmpreg2, ACSR
    sbrs tmpreg2, ACO
    rjmp nocollision
    ; IR_DDR = ddrreg
    out IR_DDR, ddrreg
    ; IR_PORT &= ~(tx_mask)
    lds tmpreg2, tx_mask
    com tmpreg2
    in tmpreg1, IR_PORT
    and tmpreg1, tmpreg2
    out IR_PORT, tmpreg1
    ; restore interrupt state
    ; SREG = sreg
    out SREG, sreg
    return 0
nocollision:
    nop
    dec tmpreg1
    brne checkcollision

    ldi tmpreg1, 0x01
    irsend
    delay_cycles (rx_bitcycles-irsend_cycles)

    movw r30, r24     ; // copy msg pointer in r24:r25 to r30:r31
    ldi byteidx, 12

sendbyte:
    ; byteval = msg[i++]<<1 | (1<<0) | (1<<9)
    ld bytevalL, Z+
    ldi bytevalH, 0x00
    add bytevalL, bytevalL
    adc bytevalH, bytevalH
    ori bytevalL, 0x01
    ori bytevalH, 0x02
    ; bitidex = 10
    ldi bitidx, 10
    ; bitmask = 0x00001
    ldi bitmaskL, 0x01
    ldi bitmaskH, 0x00

    ; 9 cycles per iteration + irsend
sendbit:
    ; tmpreg = (byteval&bytemask)
    movw tmpreg1, bitmaskL
    and tmpreg1, bytevalL
    and tmpreg2, bytevalH
    or  tmpreg1, tmpreg2

    ; irsend tmpreg
    irsend
    delay_cycles (rx_bitcycles-irsend_cycles-9)

    ; bitmask <<= 1
    add bitmaskL, bitmaskL
    adc bitmaskH, bitmaskH

    ; if (bitidex--) goto sendbit
    dec bitidx
    brne sendbit

    ; if (byteidx--) goto sendbyte
    dec byteidx
    brne sendbyte

    ; ACSR |= (1<<ACI)
    in tmpreg1, ACSR
    ori tmpreg1, (1<<ACI)
    out ACSR, tmpreg1
    ; IR_DDR = ddrreg
    out IR_DDR, ddrreg

    ; IR_PORT &= ~(tx_mask)
    lds tmpreg2, tx_mask
    com tmpreg2
    in tmpreg1, IR_PORT
    and tmpreg1, tmpreg2
    out IR_PORT, tmpreg1

    ; restore interrupt state
    ; SREG = sreg
    out SREG, sreg
    return 1
