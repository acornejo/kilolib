#define ir_port PORTD
#define ir_mask (1<<3)
#define blue_port PORTD
#define blue_mask (1<<2)
#define green_port PORTB
#define green_mask (1<<1)

#define F_CPU 8000000L

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include "messages.h"

#define BAUDRATE 19200

#define NOP asm volatile("nop\n\t")

int i;
uint8_t leds_toggle = 0;
message_t msg;

volatile	int ReceivedByte;

int main() {
    // Set port outputs
    DDRB = (1<<1)|(1<<2);
    DDRD = (1<<2)|(1<<3);
    // Turn IR led off
    PORTD &= ~(1<<3);
    // turn off analog comparator (to avoid detecting collisions)
    ACSR |= (1<<ACD);

	//move interrupt vectors to bootloader interupts
	MCUCR = (1<<IVCE);
	MCUCR = (1<<IVSEL);

	cli();
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);              // No parity, 8 bits comm, 1 stop bit
	UCSR0B |= (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0);    // Enable reception, transmission, and reception interrupts
    UBRR0 = ((F_CPU/(BAUDRATE*16UL))-1);            // Set baud rate
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

	while(1) {
		if(ReceivedByte) {
            int message_received = ReceivedByte;
            ReceivedByte = 0;

            switch(message_received) {
            case 'a':
				//send boot load message first
                msg.type = BOOT;
                msg.crc = message_crc(&msg);
				for(i=0;i<100;i++) {
                    message_send(&msg);
                    green_port |= green_mask;
					_delay_ms(10);
                    green_port &= ~green_mask;
					_delay_ms(10);
				}
				_delay_ms(8000);

				// send bootload pages until uart says to stop
                uint8_t page = 0;
                while (!ReceivedByte) {
                    msg.type = BOOTLOAD_MSG;
                    msg.page_address = page;
                    for (i=0; i<SPM_PAGESIZE; i+=4) {
                        msg.page_offset = i;
                        msg.word1 = pgm_read_word(page*SPM_PAGESIZE+i);
                        msg.word2 = pgm_read_word(page*SPM_PAGESIZE+i+2);
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
            case 'd':
                msg.type = PAUSE;
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
