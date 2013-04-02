#define __SFR_OFFSET 0
#include <avr/io.h>

#ifdef OHC
#define IR_PORT PORTD
#define IR_DDR DDRD
#else
#define IR_PORT PORTB
#define IR_DDR DDRB
#endif

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
    lds tmpreg2, tx_mask;  1
    cpi tmpreg1, 0        ;  1
    breq 1f               ;  2
    nop             
    in tmpreg1, IR_PORT   ;  1
    or tmpreg1, tmpreg2   ;  1
    out IR_PORT, tmpreg1  ;  1
    rjmp 2f               ;  2
1:
    nop                   ;  1
    nop                   ;  1
    nop                   ;  1
    rjmp 2f               ;  2
2:
    nop                   ;  1
    nop                   ;  1
    nop                   ;  1
    com tmpreg2           ;  1
    in tmpreg1, IR_PORT   ;  1
    and tmpreg1, tmpreg2  ;  1
    out IR_PORT, tmpreg1  ;  1
.endm                     ;  total = 16 cycles
#define irsend_cycles 16
#define rx_bitcycles  256

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
    ; ddreg = IR_DDR
    in  ddrreg,  IR_DDR

    ; IR_DDR |= tx_mask
    lds tmpreg1, tx_mask
    or tmpreg1, ddrreg
    out IR_DDR, tmpreg1

    ldi tmpreg1, 0x01
    irsend 

    delay_cycles (2*rx_bitcycles-irsend_cycles)

    ldi tmpreg1, 224 ;; (256*7/8)
checkcollision:
    ; if (ACSR&(1<<ACO)) == 0 goto nocollision
    in tmpreg2, ACSR
    sbrs tmpreg2, ACO
    rjmp nocollision
    ; IR_DDR = ddrreg
    out IR_DDR, ddrreg
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

sendbit: ; 9 cycles per iteration + irsend
    ; irsend (byteval&bytemask)
    movw tmpreg1, bitmaskL
    and tmpreg1, bytevalL
    and tmpreg2, bytevalH
    or  tmpreg1, tmpreg2
    irsend 

    ; bitmask <<= 1
    add bitmaskL, bitmaskL
    adc bitmaskH, bitmaskH

    delay_cycles (rx_bitcycles-irsend_cycles-9)

    ; if (bitidex--) goto sendbit
    dec bitidx
    brne sendbit

bytedone:
    ; if (byteidx--) goto sendbyte
    dec byteidx
    brne sendbyte

    ; ACSR |= (1<<ACI)
    in tmpreg1, ACSR
    ori tmpreg1, (1<<ACI)
    out ACSR, tmpreg1
    ; IR_DDR = ddrreg
    out IR_DDR, ddrreg
    return 1
