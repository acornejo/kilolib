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
#define bytevalL r24
#define bytevalH r25
#define byteidx  r26
#define bitidx   r27
#define ddrreg   r28

.extern tx_maskon
.global message_send

.section .text

.macro irsend
    lds tmpreg2, tx_maskon;  1
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

.macro return retval
    ldi r24, \retval
    ret
.endm

.macro busywait cycles
.if \cycles%3 == 0
    nop
    nop
    ldi tmpreg1, \cycles/3-1
.elseif \cycles%3 == 1
    ldi tmpreg1, \cycles/3
.else
    nop
    ldi tmpreg1, \cycles/3-1
.endif
1:
    dec tmpreg1
    brne 1b
.endm

message_send:
    ; ddreg = IR_DDR
    in  ddrreg,  IR_DDR

    ; IR_DDR |= tx_maskon
    lds tmpreg1, tx_maskon
    or tmpreg1, ddrreg
    out IR_DDR, tmpreg1

    ldi tmpreg1, 0x01
    irsend 

    busywait (256*2-16)

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
    subi tmpreg1, 0x01
    nop
    brne checkcollision

    ldi tmpreg1, 0x01
    irsend 

    busywait (256-16)

    movw r30, r24     ; // copy msg pointer in r24:r25 to r30:r31
    ldi byteidx, 12

sendbyte:
    ; bitidex = 10
    ldi bitidx, 10
    ; bitmask = 0x00001
    ldi bitmaskL, 0x01
    ldi bitmaskH, 0x00
    ; byteval = msg[i++]<<1 | (1<<0) | (1<<9)
    ld bytevalL, Z+
    ldi bytevalH, 0x00
    add bytevalL, bytevalL
    adc bytevalH, bytevalH
    ori bytevalL, 0x01
    ori bytevalH, 0x02

sendbit: ; 9 cycles per iteration + irsend
    ; irsend (byteval&bytemask)
    movw tmpreg1, bitmaskL
    and tmpreg1, bytevalL
    and tmpreg2, bytevalH
    or  tmpreg1, tmpreg2
    irsend 

    busywait (256-9-16)

    ; bitmask <<= 1
    add bitmaskL, bitmaskL
    adc bitmaskH, bitmaskH

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
