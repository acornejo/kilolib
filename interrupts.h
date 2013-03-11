#include <stdlib.h> // for rand()

// clock cycles between pulse and interrupt trigger (measured in oscilloscope to be 4.2us)
#define rx_bitdelay  34
#define rx_bitcenter (rx_bitcycles+rx_bitcycles/2)
#define rx_bytetimer (8*rx_bitcycles+rx_bitcycles/2-rx_bitdelay)

#ifndef BOOTLOADER // not required in bootloader
/**
 * Timer0 interrupt.
 * Used to send messages every tx_period ticks.
 */
ISR(TIMER0_COMPA_vect) {
	tx_clock += tx_increment;
    tx_increment = 0xFF;
	OCR0A = tx_increment;

	if(!rx_busy && tx_clock>tx_period && !RB_empty(txbuffer)) {
        message_t *msg = (message_t*)&RB_front(txbuffer);
        if (message_send(msg)) {
            RB_popfront(txbuffer);
            tx_clock = 0;
        } else {
            tx_increment = rand()&0xFF;
            OCR0A = tx_increment;
        }
    }
}

#endif
/**
 * Timer1 interrupt.
 * Triggered for every byte decoded.
 */
ISR(TIMER1_COMPA_vect) {
    rx_timer_off();
    rx_leadingbit = 1;

    if (rx_leadingbyte) {
        if (rx_bytevalue == 0) {      /* Leading byte received. */
            rx_leadingbyte = 0;
            rx_byteindex = 0;
        } else {                      /* Collision occurred. */
            adc_trigger_setlow();
            rx_leadingbyte = 1;
            rx_busy = 0;
//            tx_timer_on();
        }
    } else {
        rx_msg.rawdata[rx_byteindex] = rx_bytevalue;
        rx_byteindex++;
        switch(rx_byteindex) {
            case 1:
                rx_low_gain = ADCW;
                adc_trigger_sethigh();
                break;
            case 2:
                rx_high_gain = ADCW;
                adc_trigger_setlow();
                break;
            case sizeof(message_t)+1:
                rx_leadingbyte = 1;
                rx_busy = 0;
//                tx_timer_on();

                if (rx_msg.crc == message_crc(&rx_msg))
                    process_message(&rx_msg);

                break;
        }
    }
}

/**
 * Analog comparator trigger interrupt.
 * Triggerred for incoming IR pulses (1 bits).
 */
ISR(ANALOG_COMP_vect) {
    PORTD |= (1<<1);
	uint16_t timer = TCNT1;

	if(rx_leadingbit) {
        OCR1A = rx_bytetimer;    // set timeout for end of byte
        rx_timer_on();           // enable end of byte timer
        adc_trigger_stop();
        rx_bytevalue = 0;
		rx_leadingbit = 0;
	} else {
		if (timer<(rx_bitcenter+rx_bitcycles*3)) {
			if (timer<(rx_bitcenter+rx_bitcycles*1)) {
				if (timer<(rx_bitcenter+rx_bitcycles*0))
					rx_bytevalue |= (1<<0);
				else
					rx_bytevalue |= (1<<1);
			} else {
				if (timer<(rx_bitcenter+rx_bitcycles*2))
					rx_bytevalue |= (1<<2);
				else
					rx_bytevalue |= (1<<3);
			}
		} else {
			if (timer<(rx_bitcenter+rx_bitcycles*5)) {
				if (timer<(rx_bitcenter+rx_bitcycles*4))
					rx_bytevalue |= (1<<4);
				else
					rx_bytevalue |= (1<<5);
			} else {
				if (timer<(rx_bitcenter+rx_bitcycles*6))
					rx_bytevalue |= (1<<6);
				else
					rx_bytevalue |= (1<<7);
			}
		}
	}

    tx_timer_off();
    rx_busy = 1;
    PORTD &= ~(1<<1);
}
