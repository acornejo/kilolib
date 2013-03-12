#define tx_timer_on()       TIMSK0 &= ~(1<<OCIE0A)
#define tx_timer_off()      TIMSK0 |= (1<<OCIE0A)
#define adc_on()            ADCSRA |= (1<<ADEN)
#define adc_off()           ADCSRA &= ~(1<<ADEN)
#define is_charging()       ((PIND&(1<<0)) == 0)

#define adc_setup() {\
    ADCSRA = (1<<ADEN)|(1<<ADPS1)|(1<<ADPS0);\
    ADCSRA |= (1<<ADSC);\
    while ((ADCSRA&(1<<ADSC))==1);\
}

#define adc_trigger_setlow() {\
    ADMUX = 1;\
    ADCSRA = (1<<ADEN)|(1<<ADATE)|(1<<ADPS1)|(1<<ADPS0);\
    ADCSRB = (1<<ADTS0);\
}

#define adc_trigger_sethigh() {\
    ADMUX = 0;\
    ADCSRA = (1<<ADEN)|(1<<ADATE)|(1<<ADPS1)|(1<<ADPS0);\
    ADCSRB = (1<<ADTS0);\
}

#define adc_trigger_stop() ADCSRA &= ~(1<<ADATE)

#define ports_off() {\
    DDRB = 0;\
    DDRC = 0;\
    DDRD = 0;\
    PORTB = 0;\
    PORTC = 0;\
    PORTD = 0;\
}

#define ports_on() {\
    DDRD |= (1<<2);\
    PORTD |= (1<<2);\
}

#define tx_timer_setup() {\
	TCCR0A = 0;\
    OCR0A = 0xFF;\
	TCCR0B = (1<<CS02)|(1<<CS00);\
    TIMSK0 = 0;\
}

#define motors_setup() {\
    DDRD &= ~(1<<3);\
    DDRB &= ~(1<<3);\
    TCCR2A = (1<<COM2A1)|(1<<COM2B1)|(1<<WGM20);\
    TCCR2B = (1<<CS01);\
    OCR2B = 0;\
    OCR2A = 0;\
}

#define motors_off() {\
    DDRD &= ~(1<<3);\
    DDRB &= ~(1<<3);\
    OCR2B = 0;\
    OCR2A = 0;\
}

#define motors_on() {\
    DDRD |= (1<<3);\
    DDRB |= (1<<3);\
    OCR2B = 0x00;\
    OCR2A = 0x00;\
}

#define rx_timer_setup() {\
    TCCR1A = 0;\
    TCCR1B = 0;\
    OCR1A = rx_msgcycles;\
    TIMSK1 = (1<<OCIE1A); /* Interrupt enable on match output compare register A */\
}

#define rx_timer_on() {\
    TCNT1 = 0;  /* reset count */ \
    TCCR1B = 1; /* set prescalar to 1 (enabled). */\
}

#define rx_timer_off() {\
    TCCR1B = 0; /* set prescalar to 0 (disabled). */ \
    TCNT1 = 0;  /* reset count */ \
}
