#include <stdlib.h> // for rand()

static const uint16_t rx_bitcenter = rx_bitcycles+rx_bitcycles/2;

/**
 * Timer0 interrupt.
 * Used to send messages every tx_period ticks.
 */
ISR(TIMER0_COMPA_vect) {
	tx_clock += tx_increment;
    tx_increment = 255;
	OCR0A = tx_increment;

	if(!rx_busy && tx_clock>tx_period && !RB_empty(txbuffer)) {
        message_t *msg = (message_t*)&RB_front(txbuffer);
        if (message_send(msg)) {
            RB_popfront(txbuffer);
            tx_clock = 0;
        } else {
            tx_increment = rand()%255;
            OCR0A = tx_increment;
        }
    }
}

/**
 * Timer1 interrupt.
 * Triggered for every byte decoded.
 */
ISR(TIMER1_COMPA_vect) {
    if (rx_leadingbyte) {
        if (rx_bytevalue == 0) {      /* Leading byte received. */
            rx_leadingbyte = 0;
            rx_byteindex = 0;
        } else {                      /* Collision occurred. */
            rx_leadingbyte = 1;
            rx_busy = 0;
//            txtimer_on();
        }
    } else {
        rx_msg.rawdata[rx_byteindex] = rx_bytevalue;
        rx_byteindex++;
        switch(rx_byteindex) {
            case 1:
                ADMUX = 1;
                ADCSRA = (1<<ADEN)|(1<<ADATE)|(1<<ADPS1)|(1<<ADPS0);
                ADCSRB = (1<<ADTS0);
                break;
            case 2:
                rx_low_gain = ADCW;
                ADMUX = 0;
                ADCSRA = (1<<ADEN)|(1<<ADATE)|(1<<ADPS1)|(1<<ADPS0);
                ADCSRB = (1<<ADTS0);
                break;
            case 3:
                rx_high_gain = ADCW;
                break;
            case sizeof(message_t)+1:
                printf("%s\n", rx_msg.rawdata);
                // if (rx_msg.crc == message_crc(&rx_msg)) {
                //     if (rx_msg.type != NORMAL) {
                //         process_specialmessage(rx_msg.type);
                //     } else {
                //         RB_back(rxbuffer) = rx_msg;
                //         RB_pushback(rxbuffer);
                //     }
                // }

                rx_leadingbyte = 1;
                rx_busy = 0;
//                txtimer_on();
                break;
        }
    }
    rxtimer_off();
    rx_leadingbit = 1;
}

/**
 * Analog comparator trigger interrupt.
 * Triggerred for incoming IR pulses (1 bits).
 */
ISR(ANALOG_COMP_vect) {
	uint16_t timer = TCNT1;

	if(rx_leadingbit) {
        OCR1A = 8*rx_bitcycles+rx_bitcycles/3; // set timeout for end of byte
        rxtimer_on();                          // enable end of byte timer
		ADCSRA &= ~(1<<ADATE);                 // disable ADC auto trigger conversions
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

    txtimer_off();
    rx_busy = 1;
}
