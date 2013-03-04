#define txtimer_on()        TCCR0B=0x05
#define txtimer_off()       TCCR0B=0
#define adc_on()            ADCSRA |= (1<<ADEN)
#define adc_off()           ADCSRA &= ~(1<<ADEN)
#define is_charging()     (PIND&(1<<0)) != 0

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

#define txtimer_setup() {\
	TCCR0A = 0x00;\
	TCCR0B = 0x00;\
	TIMSK0 = 0x02;\
    OCR0A = 0xFF;\
}

#define motors_setup() {\
    TCCR2A |= (1<<COM2A1)|(1<<COM2B1)|(1<<WGM20);\
    TCCR2B |= (1<<CS01);\
    OCR2B = 0x00;\
    OCR2A = 0x00;\
}

#define motors_off() {\
    DDRD &= ~(1<<3);\
    DDRB &= ~(1<<3);\
    OCR2B = 0x00;\
    OCR2A = 0x00;\
}

#define motors_on() {\
    DDRD |= (1<<3);\
    DDRB |= (1<<3);\
    OCR2B = 0x00;\
    OCR2A = 0x00;\
}

#define rxtimer_on() {\
    TCNT1H = 0;\
    TCNT1L = 0;\
    TCCR1A = 0;\
    TCCR1B = 1; /* set prescalar to 1 */\
}

#define rxtimer_off() {\
    TCNT1H = 0;\
    TCNT1L = 0;\
    TCCR1A = 0;\
    TCCR1B = 0; /* set prescalar to 0 (disabled). */ \
}
