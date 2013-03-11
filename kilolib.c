#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <util/delay.h>

#include "kilolib.h"
#include "ringbuffer.h"
#include "macros.h"


typedef void (*AddressPointer_t)(void) __attribute__ ((noreturn));
AddressPointer_t reset = (AddressPointer_t)0x0000;
AddressPointer_t bootload = (AddressPointer_t)0x7000;

// ring buffers for input/output messages
volatile RB_create(txbuffer, message_t, 4);
volatile RB_create(rxbuffer, message_t, 4);

uint16_t tx_clock;                 // number of timer cycles we have waited
uint16_t tx_increment;             // number of timer cycles until next interrupt
message_t rx_msg;                  // message being received
volatile uint8_t rx_busy;          // flag that signals if message is being received
uint8_t rx_leadingbit;             // flag that signals start bit
uint8_t rx_leadingbyte;            // flag that signals start byte
uint8_t rx_byteindex;              // index to the current byte being decoded
uint8_t rx_bytevalue;              // value of the current byte being decoded
uint16_t rx_high_gain;             // signal strength of first start bit  (with high gain resistor)
uint16_t rx_low_gain;              // signal strength of second start bit (with low gain resistor)

volatile enum {
    SLEEPING,
    IDLE,
    BATTERY,
    RUNNING,
    CHARGING,
} state;


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
#endif

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
    DDRD |= (1<<1);

	// initalize analog comparator
	ACSR |= (1<<ACIE)|(1<<ACIS1)|(1<<ACIS0); //trigger interrupt on rising output edge
	DIDR1 = 3;

	// initalize adc
    adc_setup();
    adc_trigger_setlow();          // set AD to measure low gain

    RB_init(txbuffer);
    RB_init(rxbuffer);
    OSCCAL = eeprom_read_byte((uint8_t *)0x01);
	tx_maskon = eeprom_read_byte((uint8_t *)0x90);
    tx_maskoff = ~tx_maskon;
    tx_clock = 0;
    tx_increment = 255;
    rx_busy = 0;
    rx_leadingbit = 1;
    rx_leadingbyte = 1;
    rx_byteindex = 0;
    rx_bytevalue = 0;
    rx_high_gain = 0;
    rx_low_gain = 0;
    state = IDLE;
    sei();
}

#ifndef BOOTLOADER
void kilo_loop() {
    int i;
    while (1) {
        switch(state) {
            case SLEEPING:
                cli();
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
                ports_on();
                adc_on();

                for(i=0;i<10;i++) {
                    if (rx_busy) {
                        set_color(3,0,0);
                        _delay_ms(100);
                        break;
                    } else {
                        set_color(3,3,3);
                        _delay_ms(1);
                    }
                }
                break;
            case IDLE:
                set_color(0,3,0);
                _delay_ms(5);
                set_color(0,0,0);
                _delay_ms(500);
                break;
            case BATTERY:
				if(get_voltage()>400)
					set_color(0,7,0);
				else if(get_voltage()>390)
					set_color(0,0,7);
				else if(get_voltage()>350)
					set_color(7,7,0);
				else
					set_color(7,0,0);
                break;
            case CHARGING:
                if (is_charging()) {
					set_color(1,0,0);
					_delay_ms(2);
					set_color(0,0,0);
					_delay_ms(500);
                }
                break;
            case RUNNING:
                program_loop();
                break;
        }
    }
}

void process_message(message_t *msg) {
    if (msg->type == NORMAL) {
        RB_back(rxbuffer) = *msg;
        RB_pushback(rxbuffer);
        return;
    }
    set_color(0,0,0);
    motors_off();
    tx_timer_off();
    switch (msg->type) {
        case BOOT:
            bootload();
            break;
        case SLEEP:
            state = SLEEPING;
            break;
        case PAUSE:
        case WAKEUP:
            state = IDLE;
            break;
        case CHARGE:
            state = CHARGING;
            break;
        case VOLTAGE:
            state = BATTERY;
            break;
        case RUN:
            motors_on();
            tx_timer_on();
            state = RUNNING;
            break;
        case RESET:
            reset();
            break;
        default:
            break;
    }
}

#else

void kilo_loop() {}
#endif

int get_ambientlight() {
	if (!rx_busy) {
		while ((ADCSRA&(1<<ADSC))==1);            // wait until previous AD conversion is done
		cli();                                    // disable interrupts
		ADMUX = 7;                                // select ADC source
		ADCSRA = (1<<ADEN)|(1<<ADPS1)|(1<<ADPS0); // enable ADC and set prescalar
		ADCSRA |= (1<<ADSC);                      // start AD conversion
		while ((ADCSRA&(1<<ADSC))==1);            // wait until AD conversion is done
        adc_trigger_setlow();                     // set AD to measure low gain (for distance sensing)
		sei();                                    // reenable interrupts

		return ADCW;
	}
	else
        return -1;
}

int get_voltage() {
	if (!rx_busy) {
		while ((ADCSRA&(1<<ADSC))==1);            // wait until previous AD conversion is done
		cli();                                    // disable interrupts
		ADMUX = 6;                                // select ADC source
		ADCSRA = (1<<ADEN)|(1<<ADPS1)|(1<<ADPS0); // enable ADC and set prescalar
		ADCSRA |= (1<<ADSC);                      // start AD conversion
		while ((ADCSRA&(1<<ADSC))==1);            // wait until AD conversion is done
        adc_trigger_setlow();                     // set AD to measure low gain (for distance sensing)
		sei();                                    // reenable interrupts

        return ADCW*19/32+2; // (.0059*(double)ADCW+.0156)*100.0;
	}
	else
        return -1;
}

void set_color(int8_t red, int8_t green, int8_t blue) {
    if (blue&1)
		DDRC |= (1<<5);
	else
		DDRC &= ~(1<<5);

    if (blue&2)
		DDRC |= (1<<4);
	else
		DDRC &= ~(1<<4);

    if (red&1)
		DDRD |= (1<<5);
	else
		DDRD &= ~(1<<5);

    if (red&2)
		DDRD |= (1<<4);
	else
		DDRD &= ~(1<<4);

    if (green&1)
		DDRC |= (1<<3);
	else
		DDRC &= ~(1<<3);

    if (green&2)
		DDRC |= (1<<2);
	else
		DDRC &= ~(1<<2);
}

void set_motors(int8_t ccw, int8_t cw) {
    OCR2A = ccw;
    OCR2B = cw;
}

void txbuffer_push(message_t *msg)  {
    msg->crc = message_crc(msg);
    msg->type = NORMAL;
    cli();
    RB_back(txbuffer) = *msg;
    RB_pushback(txbuffer);
    sei();
}

uint8_t txbuffer_size() {
    return RB_size(txbuffer);
}

void rxbuffer_pop(message_t *msg) {
    cli();
    *msg = RB_front(rxbuffer);
    RB_popfront(rxbuffer);
    sei();
}

uint8_t rxbuffer_size() {
    return RB_size(rxbuffer);
}

#include "interrupts.h"
