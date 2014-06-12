#include <avr/io.h>        // for port and register definitions
#include <avr/interrupt.h> // for ISR
#include <util/delay.h>    // for _delay_ms
#include <string.h>        // for memcpy
#include "ohc.h"           // for message definitions
#include "message_crc.h"
#include "message_send.h"
#include "bootldr.h"

uint8_t packet_buffer[PACKET_SIZE];
uint8_t packet_head = 0;
uint8_t packet_checksum = 0;
uint8_t new_packet[PACKET_SIZE];
volatile uint8_t packet_type;
volatile uint8_t has_new_packet = 0;
volatile uint8_t tx_mask = 0;
uint8_t leds_toggle = 0;
uint8_t *rawmsg;
message_t msg;
bootmsg_t *bootmsg;
gpsmsg_t *gpsmsg;

#ifdef ARDUINO
#define ir_port PORTB
#define ir_ddr DDRB
#define ir_mask (1<<1)
#define led_port PORTB
#define led_ddr DDRB
#define led_mask (1<<5)
#else
#define ir_port PORTD
#define ir_ddr DDRD
#define ir_mask (1<<3)
#define led_port PORTB
#define led_ddr DDRB
#define led_mask (1<<1)
#endif

int main() {
    cli();
    // Set port outputs
    ir_ddr |= ir_mask;
    led_ddr |= led_mask;
    // Turn off all leds
    led_port &= ~led_mask;
    ir_port &= ~ir_mask;
    // turn off analog comparator (to avoid detecting collisions)
    ACSR |= (1<<ACD);

#ifndef ARDUINO
    //move interrupt vectors to bootloader interupts
    MCUCR = (1<<IVCE);
    MCUCR = (1<<IVSEL);
#endif
#ifdef ARDUINO_16MHZ
    CLKPR = (1<<CLKPCE);
    CLKPR = 1;
#endif

#define BAUD 38400
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

    tx_mask = ir_mask;
    bootmsg = (bootmsg_t*)msg.data;
    gpsmsg = (gpsmsg_t*)msg.data;
    rawmsg = (uint8_t*)&msg;

    // Use LEDs to flash power on indicator signal.
    uint8_t i;
    for (i=0; i<5; i++) {
        led_port |= led_mask;
        _delay_ms(200);
        led_port &= ~led_mask;
        _delay_ms(200);
    }

    while(1) {
        if (has_new_packet) {
            has_new_packet = 0;
            switch(packet_type) {
            case PACKET_STOP:
                break;
            case PACKET_LEDTOGGLE:
                leds_toggle = !leds_toggle;
                if (leds_toggle) {
                    led_port |= led_mask;
                } else {
                    led_port &= ~led_mask;
                }
                break;
            case PACKET_FORWARDMSG:
                for (i = 0; i<sizeof(message_t)-sizeof(msg.crc); i++)
                    rawmsg[i] = new_packet[i+2];
                msg.crc = message_crc(&msg);
                while(!has_new_packet) {
                    message_send(&msg);
                    led_port |= led_mask;
                    _delay_ms(3);
                    led_port &= ~led_mask;
                    _delay_ms(3);
                }
                break;
            case PACKET_FORWARDMSGSINGLE:
                for (i = 0; i<sizeof(message_t)-sizeof(msg.crc); i++)
                    rawmsg[i] = new_packet[i+2];
                msg.crc = message_crc(&msg);
                message_send(&msg);
                led_port |= led_mask;
                _delay_ms(3);
                led_port &= ~led_mask;
                _delay_ms(3);
                break;
            case PACKET_FORWARDRAWMSG:
                for (i = 0; i<sizeof(message_t); i++)
                    rawmsg[i] = new_packet[i+2];
                while(!has_new_packet) {
                    message_send(&msg);
                    led_port |= led_mask;
                    _delay_ms(3);
                    led_port &= ~led_mask;
                    _delay_ms(3);
                }
                break;
            case PACKET_BOOTPAGE:
                msg.type = BOOTPGM_PAGE;
                bootmsg->page_address = new_packet[2];
                bootmsg->unused = 0;
                cli();
                for (i = 0; i<SPM_PAGESIZE && !has_new_packet; i+=6) {
                    bootmsg->page_offset = i/2;
                    memcpy(&(bootmsg->word1), new_packet+3+i, 6);
                    msg.crc = message_crc(&msg);
                    message_send(&msg);
                }
                sei();
                led_port |= led_mask;
                _delay_ms(10);
                led_port &= ~led_mask;
                _delay_ms(10);
                break;
#define GPS_MSGSIZE 8
            case PACKET_GPSFRAME:
                memset(&msg, 0, sizeof(message_t)-sizeof(msg.crc));
                msg.type = GPS;
                cli();
                for (i = 2; i<PACKET_SIZE-GPS_MSGSIZE; i += GPS_MSGSIZE) {
                    memcpy(gpsmsg, new_packet+i, GPS_MSGSIZE);
                    if (gpsmsg->id == 0 && gpsmsg->x == 0 && gpsmsg->y == 0 && gpsmsg->theta == 0 && gpsmsg->unused == 0)
                        break;
                    msg.crc = message_crc(&msg);
                    message_send(&msg);
                    _delay_us(50);
                }
                sei();
                led_port |= led_mask;
                _delay_ms(10);
                led_port &= ~led_mask;
                _delay_ms(10);
                break;
            }
        }
    }

    return 0;
}

ISR(USART_RX_vect) {
    uint8_t rx = UDR0;

    packet_checksum ^= packet_buffer[packet_head];
    packet_buffer[packet_head] = rx;
    packet_checksum ^= rx;
    packet_head++;
    if (packet_head >= PACKET_SIZE)
        packet_head = 0;

    if (packet_buffer[packet_head] == PACKET_HEADER) {
        if (packet_checksum == 0) {
            uint16_t i;
            uint16_t num = PACKET_SIZE-packet_head;
            for (i = 0; i < num; i++)
                new_packet[i] = packet_buffer[i+packet_head];
            for (i = num; i < PACKET_SIZE; i++)
                new_packet[i] = packet_buffer[i-num];
            has_new_packet = 1;
            packet_type = new_packet[1];
        }
    }
}
