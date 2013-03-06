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

void send_bootmsg(int,int,int);
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

				//next bootload until uart says to stop
                while (!ReceivedByte) {
                    uint8_t page;
                    for(page=0;page<220 && ReceivedByte == 0; page++) {
                        uint16_t checksum = page;
                        cli();
                        send_bootmsg(page,0,250);
                        _delay_ms(1);
                        for (i=0; i<SPM_PAGESIZE; i+=2) {
                            int high = pgm_read_byte(page*SPM_PAGESIZE+i);
                            int low = pgm_read_byte(page*SPM_PAGESIZE+i+1);
                            send_bootmsg(high,low,0);
                            checksum += high;
                            checksum += low;
                        }
                        _delay_ms(1);
                        uint8_t checksum_low=checksum;
                        uint8_t checksum_high=checksum>>8;
                        send_bootmsg(checksum_high,checksum_low,254);
                        sei(); //allow for uart to rx
                        green_port |= green_mask;
                        _delay_ms(10);
                        green_port &= ~green_mask;
                        _delay_ms(10);
                    }
                }
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

void send_bootmsg(int a, int b, int c)
{
    uint16_t k;
	uint16_t data_out[4];
	uint8_t data_to_send[4]={a,b,c,a+b+c+128};

	//prepare data to send
	for(k=0;k<4;k++) {
		data_out[k]=(data_to_send[k] & (1<<0))*128 +
				(data_to_send[k] & (1<<1))*32 +
				(data_to_send[k] & (1<<2))*8 +
				(data_to_send[k] & (1<<3))*2+
				(data_to_send[k] & (1<<4))/2+
				(data_to_send[k] & (1<<5))/8 +
				(data_to_send[k] & (1<<6))/32 +
				(data_to_send[k] & (1<<7))/128;

		data_out[k]=data_out[k]<<1;
		data_out[k]++;
	}

    //send start pulse
    ir_port |= tx_maskon;
    NOP;
    NOP;
    ir_port &= tx_maskoff;

    //wait for own signal to die down
    for(k=0;k<53;k++)
        NOP;

    //check for collision
    for(k=0;k<193;k++) {
        if((ACSR & (1<<ACO))>0) NOP;
        if((ACSR & (1<<ACO))>0) NOP;
    }

    uint16_t byte_sending;
    for(byte_sending=0;byte_sending<4;byte_sending++) {
        int i=8;
        while(i>=0) {
            if(data_out[byte_sending] & 1) {
                ir_port |= tx_maskon;
                NOP;
                NOP;
            } else {
                ir_port &= tx_maskoff;
                NOP;
                NOP;
            }
            ir_port &= tx_maskoff;
            for(k=0;k<35;k++)
                NOP;
            data_out[byte_sending]=data_out[byte_sending]>>1;
            i--;
        }
    }

    //ensure led is off
    ir_port &= tx_maskoff;

    //wait for own signal to die down
    for(k=0;k<50;k++)
        NOP;

    ACSR |= (1<<ACI);
}

ISR(USART_RX_vect) {
    ReceivedByte = UDR0;
}
