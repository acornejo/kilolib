#include <stdlib.h> // for rand()

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
 * Triggered after every a message is received.
 */
ISR(TIMER1_COMPA_vect) {
    /* tx_timer_on(); */
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

    tx_timer_off();
    rx_busy = 1;
    adc_trigger_stop();

	if(rx_leadingbit) {       // Start bit received.
        rx_timer_on();
        rx_bytevalue = 0;
		rx_leadingbit = 0;
        if (rx_leadingbyte) {
            rx_low_gain = ADCW;
            adc_trigger_sethigh();
        }
	} else {
        // Stray bit received
        if (timer <= rx_bitcycles/2 || timer >= rx_bitcycles*9+rx_bitcycles/2) {
            /* tx_timer_on(); */
            rx_leadingbit = 1;
            rx_leadingbyte = 1;
            rx_busy = 0;
            rx_timer_off();
            adc_trigger_setlow();
        } else {
            uint8_t bitindex = (timer-rx_bitcycles/2)/rx_bitcycles;
            if (bitindex <= 7) { // Data bit received.
                rx_bytevalue |= (1<<bitindex);
            } else {             // Stop bit received.
                rx_leadingbit = 1;
                if (rx_leadingbyte) {
                    rx_high_gain = ADCW;
                    adc_trigger_setlow();
                    if (rx_bytevalue != 0) { // Collision detected.
                        /* tx_timer_on(); */
                        rx_leadingbyte = 1;
                        rx_busy = 0;
                        rx_timer_off();
                    } else {                // Leading byte received.
                        rx_leadingbyte = 0;
                        rx_byteindex = 0;
                    }
                } else {
                    rx_msg.rawdata[rx_byteindex] = rx_bytevalue;
                    rx_byteindex++;
                    if (rx_byteindex == sizeof(message_t)) {
                        /* tx_timer_on(); */
                        rx_leadingbyte = 1;
                        rx_busy = 0;
                        rx_timer_off();

                        if (rx_msg.crc == message_crc(&rx_msg))
                            process_message(&rx_msg);
                    }
                }
            }
        }
	}
}
