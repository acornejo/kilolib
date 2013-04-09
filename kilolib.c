#include <avr/io.h>         // io port and addresses
#include <avr/wdt.h>        // watch dog timer
#include <avr/interrupt.h>  // interrupt handling
#include <avr/eeprom.h>     // read eeprom values
#include <avr/sleep.h>      // enter powersaving sleep mode
#include <util/delay.h>     // delay macros
#include <stdlib.h>         // for rand()

#include "kilolib.h"
#include "macros.h"

#define EEPROM_OSCCAL (uint8_t*)0x01
#define EEPROM_TXMASK (uint8_t*)0x90
#define EEPROM_UID    (uint8_t*)0xB0
#define EEPROM_CCW_IN_PLACE (uint8_t*)0x09
#define TX_MASK_MAX   ((1<<0)|(1<<1)|(1<<2)|(1<<6)|(1<<7))
#define TX_MASK_MIN   ((1<<0))

typedef void (*AddressPointer_t)(void) __attribute__ ((noreturn));
AddressPointer_t reset = (AddressPointer_t)0x0000;
AddressPointer_t bootload = (AddressPointer_t)0x7000;

static message_rx_t message_rx = 0;
static message_tx_t message_tx = 0;
static message_tx_success_t message_tx_success = 0;

uint16_t tx_clock;                 // number of timer cycles we have waited
uint16_t tx_increment;             // number of timer cycles until next interrupt
message_t rx_msg;                  // message being received
distance_measurement_t rx_dist;    // signal strength of message being received
volatile uint8_t rx_busy;          // flag that signals if message is being received
uint8_t rx_leadingbit;             // flag that signals start bit
uint8_t rx_leadingbyte;            // flag that signals start byte
uint8_t rx_byteindex;              // index to the current byte being decoded
uint8_t rx_bytevalue;              // value of the current byte being decoded

static volatile enum {
    SLEEPING,
    IDLE,
    BATTERY,
    RUNNING,
    CHARGING,
    READINGUID,
} kilo_state;

/**
 * Initialize all global variables to a known state.
 * Setup all the pins and ports.
 */
void kilo_init(message_rx_t mrx, message_tx_t mtx, message_tx_success_t mtxsuccess) {
    cli();
    ports_off();
    ports_on();
    tx_timer_setup();
    rx_timer_setup();
    motors_setup();
    acomp_setup();
    adc_setup();
    adc_trigger_setlow();          // set AD to measure low gain

    message_rx = mrx;
    message_tx = mtx;
    message_tx_success = mtxsuccess;
    uint8_t osccal = eeprom_read_byte(EEPROM_OSCCAL);
    if (osccal != 0xFF)
        OSCCAL = osccal;
	tx_mask = eeprom_read_byte(EEPROM_TXMASK);
    if (tx_mask & ~TX_MASK_MAX)
        tx_mask = TX_MASK_MIN;
    tx_clock = 0;
    tx_increment = 255;
    rx_busy = 0;
    rx_leadingbit = 1;
    rx_leadingbyte = 1;
    rx_byteindex = 0;
    rx_bytevalue = 0;
    kilo_ticks = 0;
    kilo_state = IDLE;
    kilo_uid = eeprom_read_byte(EEPROM_UID) | eeprom_read_byte(EEPROM_UID+1)<<8;
    sei();
}

#ifndef BOOTLOADER
// Ensure that wdt is inactive after system reset.
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));

void wdt_init(void) {
    MCUSR = 0;
    wdt_disable();
}

/**
 * Watchdog timer interrupt.
 * Used to wakeup from low power sleep mode.
 */
ISR(WDT_vect) {
    wdt_disable();
}

static uint8_t read_move;

void kilo_loop(void (*program)(void)) {
    int16_t voltage;
    while (1) {
        switch(kilo_state) {
            case SLEEPING:
                cli();
                acomp_off();
                adc_off();
                ports_off();
                wdt_enable(WDTO_8S);
                WDTCSR |= (1<<WDIE);
                set_sleep_mode(SLEEP_MODE_PWR_DOWN);
                cli();
                sleep_enable();
                sei();
                sleep_cpu();
                sleep_disable();
                sei();
                rx_busy = 0;
                ports_on();
                adc_on();
                _delay_us(300);
                acomp_on();

                set_color(RGB(3,3,3));
                _delay_ms(10);
                if (rx_busy) {
                    set_color(RGB(3,0,0));
                    _delay_ms(100);
                }
                set_color(RGB(0,0,0));
                break;
            case IDLE:
                set_color(RGB(0,3,0));
                _delay_ms(1);
                set_color(RGB(0,0,0));
                _delay_ms(200);
                break;
            case BATTERY:
                voltage = get_voltage();
				if(voltage > 682)
					set_color(RGB(0,3,0));
				else if(voltage > 648)
					set_color(RGB(0,0,3));
				else if(voltage > 614)
					set_color(RGB(3,3,0));
				else
					set_color(RGB(3,0,0));
                break;
            case CHARGING:
                if (is_charging()) {
					set_color(RGB(1,0,0));
					_delay_ms(1);
					set_color(RGB(0,0,0));
					_delay_ms(200);
                } else
                    set_color(RGB(0,0,0));
                break;
            case RUNNING:
                program();
                break;
            case READINGUID:
                set_color(RGB(0,0,0));
                switch(read_move) {
                    case 0:
                        set_motors(0,0);
                        break;
                    case 1:
                        set_motors(0,eeprom_read_byte(EEPROM_CCW_IN_PLACE));
                        break;
                    case 2:
                        set_motors(0,255);
                        _delay_ms(20);
                        read_move = 1;
                        break;
                }
                break;
        }
    }
}

inline void process_message() {
    if (rx_msg.type < SPECIAL) {
        message_rx(&rx_msg, &rx_dist);
        return;
    }
    if (rx_msg.type != READUID && rx_msg.type != RUN)
        motors_off();
    switch (rx_msg.type) {
        case BOOT:
            bootload();
            break;
        case RESET:
            reset();
            break;
        case SLEEP:
            kilo_state = SLEEPING;
            break;
        case WAKEUP:
            kilo_state = IDLE;
            break;
        case CHARGE:
            kilo_state = CHARGING;
            break;
        case VOLTAGE:
            kilo_state = BATTERY;
            break;
        case RUN:
            if (kilo_state != RUNNING) {
                motors_on();
                kilo_state = RUNNING;
            }
            break;
        case READUID:
            if (kilo_state != READINGUID) {
                motors_on();
                kilo_state = READINGUID;
                read_move = 0;
            }

            if (kilo_uid&(1<<rx_msg.data[0])) {
                if (read_move == 0)
                    read_move = 2;
            } else
                read_move = 0;
            break;
        default:
            break;
    }
}

void set_motors(uint8_t ccw, uint8_t cw) {
    OCR2A = ccw;
    OCR2B = cw;
}

int16_t get_ambientlight() {
    int16_t light = -1;
	if (!rx_busy) {
		cli();
        adc_setup_conversion(7);
        adc_start_conversion();
        adc_finish_conversion();
        light = ADCW;                             // store AD result
        adc_trigger_setlow();                     // set AD to measure low gain (for distance sensing)
		sei();                                    // reenable interrupts
	}
    return light;
}

int16_t get_voltage() {
    int16_t voltage=-1;
	if (!rx_busy) {
		cli();                                    // disable interrupts
        adc_setup_conversion(6);
        adc_start_conversion();
        adc_finish_conversion();
        voltage = ADCW;                           // store AD result
//        adc_trigger_setlow();                     // set AD to measure low gain (for distance sensing)
		sei();                                    // reenable interrupts
	}
    return voltage;
}

/**
 * Timer0 interrupt.
 * Used to send messages every tx_period ticks.
 */
ISR(TIMER0_COMPA_vect) {
	tx_clock += tx_increment;
    tx_increment = 0xFF;
	OCR0A = tx_increment;
    kilo_ticks++;

	if(!rx_busy && tx_clock>tx_period && kilo_state == RUNNING) {
        message_t *msg = message_tx();
        if (msg) {
            if (message_send(msg)) {
                message_tx_success();
                tx_clock = 0;
            } else {
                tx_increment = rand()&0xFF;
                OCR0A = tx_increment;
            }
        }
    }
}

#else// BOOTLOADER

inline void process_message() {
    message_rx(&rx_msg, &rx_dist);
}

EMPTY_INTERRUPT(TIMER0_COMPA_vect)

#endif

void set_color(uint8_t rgb) {
    if (rgb&(1<<0))
		DDRD |= (1<<5);
	else
		DDRD &= ~(1<<5);

    if (rgb&(1<<1))
		DDRD |= (1<<4);
	else
		DDRD &= ~(1<<4);

    if (rgb&(1<<2))
		DDRC |= (1<<3);
	else
		DDRC &= ~(1<<3);

    if (rgb&(1<<3))
		DDRC |= (1<<2);
	else
		DDRC &= ~(1<<2);

    if (rgb&(1<<4))
		DDRC |= (1<<5);
	else
		DDRC &= ~(1<<5);

    if (rgb&(1<<5))
		DDRC |= (1<<4);
	else
		DDRC &= ~(1<<4);
}

/**
 * Timer1 interrupt.
 * Timeout which is trigerred if stop bit is not received.
 */
ISR(TIMER1_COMPA_vect) {
    rx_timer_off();
    rx_leadingbit = 1;
    rx_leadingbyte = 1;
    rx_busy = 0;
    adc_trigger_setlow();
}

/**
 * Analog comparator trigger interrupt.
 * Triggerred for incoming IR pulses (i.e. individual bits).
 */
ISR(ANALOG_COMP_vect) {
	uint16_t timer = TCNT1;

    rx_busy = 1;
    adc_trigger_stop();

	if(rx_leadingbit) {       // Start bit received.
        rx_timer_on();
        rx_bytevalue = 0;
		rx_leadingbit = 0;
        if (rx_leadingbyte) {
            adc_trigger_sethigh();
        } else if (rx_byteindex == 0) {
            rx_dist.high_gain = ADCW;
        }
	} else {
        // Stray bit received
        if (timer <= rx_bitcycles/2 || timer >= rx_bitcycles*9+rx_bitcycles/2) {
            rx_timer_off();
            rx_leadingbit = 1;
            rx_leadingbyte = 1;
            rx_busy = 0;
            adc_trigger_setlow();
        } else {
            /* uint8_t bitindex = (timer-rx_bitcycles/2)/rx_bitcycles; */
            uint8_t bitindex = ((uint32_t)(timer-rx_bitcycles/2)*244)>>16;
            if (bitindex <= 7) { // Data bit received.
                rx_bytevalue |= (1<<bitindex);
            } else {             // Stop bit received.
                rx_leadingbit = 1;
                if (rx_leadingbyte) {
                    rx_dist.low_gain = ADCW;
                    adc_trigger_setlow();
                    if (rx_bytevalue != 0) { // Collision detected.
                        rx_timer_off();
                        rx_leadingbyte = 1;
                        rx_busy = 0;
                    } else {                // Leading byte received.
                        rx_leadingbyte = 0;
                        rx_byteindex = 0;
                    }
                } else {
                    rx_msg.rawdata[rx_byteindex] = rx_bytevalue;
                    rx_byteindex++;
                    if (rx_byteindex == sizeof(message_t)) {
                        rx_timer_off();
                        rx_leadingbyte = 1;
                        rx_busy = 0;

                        if (rx_msg.crc == message_crc(&rx_msg))
                            process_message();
                    }
                }
            }
        }
	}
}
