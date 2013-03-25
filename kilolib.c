#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <util/delay.h>

#include "kilolib.h"
#include "ringbuffer.h"
#include "macros.h"

#define EEPROM_OSCCAL 0x01
#define EEPROM_TXMASK 0x90

typedef void (*AddressPointer_t)(void) __attribute__ ((noreturn));
AddressPointer_t reset = (AddressPointer_t)0x0000;
AddressPointer_t bootload = (AddressPointer_t)0x7000;

// ring buffers for input/output messages
volatile RB_create(txbuffer, message_t, 4);
volatile RB_create(rxbuffer, message_t, 4);
volatile RB_create(rxdistbuffer, distance_measurement_t, 4);

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
} kilo_state;

/**
 * Initialize all global variables to a known state.
 * Setup all the pins and ports.
 */
void kilo_init() {
    cli();
    ports_off();
    ports_on();
    tx_timer_setup();
    rx_timer_setup();
    motors_setup();

	// initalize analog comparator
	ACSR |= (1<<ACIE)|(1<<ACIS1)|(1<<ACIS0); //trigger interrupt on rising output edge
	DIDR1 = 3;

	// initalize adc
    adc_setup();
    adc_trigger_setlow();          // set AD to measure low gain

    RB_init(txbuffer);
    RB_init(rxbuffer);
    RB_init(rxdistbuffer);
    OSCCAL = eeprom_read_byte((uint8_t *)EEPROM_OSCCAL);
	tx_maskon = eeprom_read_byte((uint8_t *)EEPROM_TXMASK);
    tx_maskoff = ~tx_maskon;
    tx_clock = 0;
    tx_increment = 255;
    rx_busy = 0;
    rx_leadingbit = 1;
    rx_leadingbyte = 1;
    rx_byteindex = 0;
    rx_bytevalue = 0;
    kilo_ticks = 0;
    kilo_state = IDLE;
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
				if(voltage > 400)
					set_color(RGB(0,3,0));
				else if(voltage > 390)
					set_color(RGB(0,0,3));
				else if(voltage > 350)
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
                }
                break;
            case RUNNING:
                program();
                break;
        }
    }
}

void process_message(message_t *msg) {
    if (msg->type < SPECIAL) {
        RB_back(rxbuffer) = *msg;
        RB_pushback(rxbuffer);
        RB_back(rxdistbuffer) = rx_dist;
        RB_pushback(rxdistbuffer);
        return;
    }
    set_color(RGB(0,0,0));
    motors_off();
    /* tx_timer_off(); */
    switch (msg->type) {
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
            motors_on();
            /* tx_timer_on(); */
            kilo_state = RUNNING;
            break;
        default:
            break;
    }
}

void txbuffer_push(message_t *msg)  {
    msg->type = NORMAL;
    msg->crc = message_crc(msg);
    cli();
    RB_back(txbuffer) = *msg;
    RB_pushback(txbuffer);
    sei();
}

uint8_t txbuffer_size() {
    return RB_size(txbuffer);
}

uint8_t rxbuffer_peek_type() {
    return RB_front(rxbuffer).type;
}

void rxbuffer_pop(message_t *msg, distance_measurement_t *dist) {
    cli();
    *msg = RB_front(rxbuffer);
    RB_popfront(rxbuffer);
    *dist = RB_front(rxdistbuffer);
    RB_popfront(rxdistbuffer);
    sei();
}

uint8_t rxbuffer_size() {
    return RB_size(rxbuffer);
}

void set_motors(uint8_t ccw, uint8_t cw) {
    OCR2A = ccw;
    OCR2B = cw;
}

int16_t get_ambientlight() {
    int16_t light = -1;
	if (!rx_busy) {
		while ((ADCSRA&(1<<ADSC))==1);            // wait until previous AD conversion is done
		cli();                                    // disable interrupts
		ADMUX = 7;                                // select ADC source
		ADCSRA = (1<<ADEN)|(1<<ADPS1)|(1<<ADPS0); // enable ADC and set prescalar
		ADCSRA |= (1<<ADSC);                      // start AD conversion
		while ((ADCSRA&(1<<ADSC))==1);            // wait until AD conversion is done
        /* while ((ADCSRA&(1<<ADIF))==0); */
        light = ADCW;                             // store AD result
        adc_trigger_setlow();                     // set AD to measure low gain (for distance sensing)
		sei();                                    // reenable interrupts
	}
    return light;
}

int16_t get_voltage() {
    int16_t voltage=-1;
	if (!rx_busy) {
		while ((ADCSRA&(1<<ADSC))==1);            // wait until previous AD conversion is done
		cli();                                    // disable interrupts
		ADMUX = 6;                                // select ADC source
		ADCSRA = (1<<ADEN)|(1<<ADPS1)|(1<<ADPS0); // enable ADC and set prescalar
		ADCSRA |= (1<<ADSC);                      // start AD conversion
		while ((ADCSRA&(1<<ADSC))==1);            // wait until AD conversion is done
        voltage = ADCW*3/2;                       // store AD result
        adc_trigger_setlow();                     // set AD to measure low gain (for distance sensing)
		sei();                                    // reenable interrupts
	}
    return voltage;
}

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

#include "interrupts.h"
