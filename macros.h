#define adc_on()            ADCSRA |= (1<<ADEN)
#define adc_off()           ADCSRA &= ~(1<<ADEN)
#define acomp_on()          ACSR |= (1<<ACIE)|(1<<ACI)
#define acomp_off()         ACSR &= ~(1<<ACIE)
#define is_charging()       ((PIND&(1<<0)) == 0)

#define acomp_setup() {\
	ACSR = (1<<ACIE)|(1<<ACIS1)|(1<<ACIS0); /* trigger interrupt on rising output edge */\
	DIDR1 = 3;\
}

#define adc_setup_conversion(CHANNEL) {\
    ADMUX = CHANNEL;\
    ADCSRA = (1<<ADEN)|(1<<ADPS1)|(1<<ADPS0);\
}

#define adc_start_conversion() ADCSRA |= (1<<ADSC)

#define adc_finish_conversion() while ((ADCSRA&(1<<ADSC)))

#define adc_trigger_low_gain() {\
    ADMUX = 1;\
    ADCSRA = (1<<ADEN)|(1<<ADATE)|(1<<ADPS1)|(1<<ADPS0);\
    ADCSRB = (1<<ADTS0);\
}

#define adc_trigger_high_gain() {\
    ADMUX = 0;\
    ADCSRA = (1<<ADEN)|(1<<ADATE)|(1<<ADPS1)|(1<<ADPS0);\
    ADCSRB = (1<<ADTS0);\
}

#define adc_setup() {\
    adc_setup_conversion(6);\
    adc_start_conversion();\
    adc_finish_conversion();\
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
    OCR2B = 0;\
    OCR2A = 0;\
}

#define tx_timer_setup() {\
    TCCR0A = 0;\
    TCCR0B = (1<<CS02)|(1<<CS00);   /* Set prescalar multiplier. */\
    OCR0A = 0xFF;                   /* Set compare register to 255. */\
    TIMSK0 = (1<<OCIE0A);           /* Enable timer1 interrupt. */\
}

#define tx_timer_on() {\
    TCNT0 = 0;             /* reset count. */\
    TIMSK0 |= (1<<OCIE0A); /* Enable timer1 interrupt. */\
}

#define tx_timer_off() {\
    TIMSK0 &= ~(1<<OCIE0A); /* Disable timer1 interrupt. */\
    TCNT0 = 0;              /* reset count. */\
}

#define rx_timer_setup() {\
    TCCR1A = 0;\
    TCCR1B = 0;             /* Set prescalar to 0 (disabled). */\
    OCR1A = rx_msgcycles;   /* Set compare register to rx_msgcycles. */\
    TIMSK1 = (1<<OCIE1A);   /* Interrupt enable on match output compare register A */\
}

#define rx_timer_on() {\
    TCNT1 = 0;              /* reset count */ \
    TCCR1B = 1;             /* set prescalar to 1 (enabled). */\
}

#define rx_timer_off() {\
    TCCR1B = 0;             /* set prescalar to 0 (disabled). */\
    TCNT1 = 0;              /* reset count */\
}
