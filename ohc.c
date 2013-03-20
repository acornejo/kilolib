#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include "messages.h"

#define ir_port PORTD
#define ir_mask (1<<3)
#define blue_port PORTD
#define blue_mask (1<<2)
#define green_port PORTB
#define green_mask (1<<1)

int i,j;
uint8_t page;
uint8_t leds_toggle = 0;
message_t msg;

volatile	int ReceivedByte;

int main() {
    // Set port outputs
    DDRB = (1<<1)|(1<<2);         // enable green led & blue led
    DDRD = (1<<2)|(1<<3);         // enable ir led & blue led
    // Turn IR led off
    ir_port &= ~ir_mask;
    // turn off analog comparator (to avoid detecting collisions)
    ACSR |= (1<<ACD);

	//move interrupt vectors to bootloader interupts
	MCUCR = (1<<IVCE);
	MCUCR = (1<<IVSEL);

	cli();
#define BAUD 76800
#include <util/setbaud.h>
    UBRR0 = UBRR_VALUE;
#if USE_2X
    UCSR0A |= (1<<U2X0);
#else
    UCSR0A &= ~(1<<U2X0);
#endif
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);              // No parity, 8 bits comm, 1 stop bit
	UCSR0B |= (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0);    // Enable reception, transmission, and reception interrupts
	sei();

    tx_maskon = ir_mask;
    tx_maskoff = ~tx_maskon;

	// Use LEDs to flash power on indicator signal.
    for (i=0; i<5; i++) {
        blue_port |= blue_mask;
        green_port |= green_mask;
        _delay_ms(200);
        blue_port &= ~blue_mask;
        green_port &= ~green_mask;
        _delay_ms(200);
    }

    for (i=0;i<sizeof(msg.rawdata); i++)
        msg.rawdata[i] = 0;

    /* while(1) { */
    /*     message_send(&msg); */
    /*     _delay_ms(20); */
    /* } */

	while(1) {
		if(ReceivedByte) {
            int message_received = ReceivedByte;
            ReceivedByte = 0;

            switch(message_received) {
            case 'a':
                // send bootload message again
                msg.type = BOOT;
                msg.crc = message_crc(&msg);
				for(i=0;i<100;i++) {
                    message_send(&msg);
                    green_port |= green_mask;
					_delay_ms(10);
                    green_port &= ~green_mask;
					_delay_ms(10);
				}
				_delay_ms(1000);

				// send bootload pages until uart says to stop
                page = 0;
                while (!ReceivedByte) {
                    msg.type = BOOTPGM_PAGE;
                    msg.bootmsg.page_address = page;
                    for (i=0; i<SPM_PAGESIZE; i+=6) {
                        msg.bootmsg.page_offset = i/2;
                        msg.bootmsg.word1 = pgm_read_word(page*SPM_PAGESIZE+i);
                        msg.bootmsg.word2 = pgm_read_word(page*SPM_PAGESIZE+i+2);
                        msg.bootmsg.word3 = pgm_read_word(page*SPM_PAGESIZE+i+4);
                        msg.crc = message_crc(&msg);
                        message_send(&msg);
                    }
                    green_port |= green_mask;
                    _delay_ms(10);
                    green_port &= ~green_mask;
                    _delay_ms(10);
                    page++;
                    if (page >= 220)
                        page = 0;
                }
                for (i=0;i<sizeof(msg.rawdata); i++)
                    msg.rawdata[i] = 0;
                break;
            case 'i':
                leds_toggle = !leds_toggle;
                if (leds_toggle)
                    blue_port |= blue_mask;
                else
                    blue_port &= ~blue_mask;
                break;
            case 'j':
                msg.type = BOOT;
                msg.crc = message_crc(&msg);
				while(!ReceivedByte) {
                    message_send(&msg);
                    green_port |= green_mask;
					_delay_ms(10);
                    green_port &= ~green_mask;
					_delay_ms(10);
				}
                break;
            case 'b':
                msg.type = SLEEP;
                msg.crc = message_crc(&msg);
				while(!ReceivedByte) {
                    message_send(&msg);
                    green_port |= green_mask;
					_delay_ms(10);
                    green_port &= ~green_mask;
					_delay_ms(10);
				}
                break;
            case 'c':
            case 'd':
                msg.type = WAKEUP;
                msg.crc = message_crc(&msg);
				while(!ReceivedByte) {
                    message_send(&msg);
                    green_port |= green_mask;
					_delay_ms(10);
                    green_port &= ~green_mask;
					_delay_ms(10);
				}
                break;
            case 'e':
                msg.type = VOLTAGE;
                msg.crc = message_crc(&msg);
				while(!ReceivedByte) {
                    message_send(&msg);
                    green_port |= green_mask;
					_delay_ms(10);
                    green_port &= ~green_mask;
					_delay_ms(10);
				}
                break;
            case 'f':
                msg.type = RUN;
                msg.crc = message_crc(&msg);
				while(!ReceivedByte) {
                    message_send(&msg);
                    green_port |= green_mask;
					_delay_ms(10);
                    green_port &= ~green_mask;
					_delay_ms(10);
				}
                break;
            case 'g':
                msg.type = CHARGE;
                msg.crc = message_crc(&msg);
				while(!ReceivedByte) {
                    message_send(&msg);
                    green_port |= green_mask;
					_delay_ms(10);
                    green_port &= ~green_mask;
					_delay_ms(10);
				}
                break;
            case 'z':
                msg.type = RESET;
                msg.crc = message_crc(&msg);
				while(!ReceivedByte) {
                    message_send(&msg);
                    green_port |= green_mask;
					_delay_ms(10);
                    green_port &= ~green_mask;
					_delay_ms(10);
				}
                break;
            }
		}
	}
    return 0;
}

ISR(USART_RX_vect) {
    ReceivedByte = UDR0;
}
