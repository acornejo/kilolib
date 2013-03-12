#include "kilolib.h"
#include <avr/eeprom.h>
#include <util/delay.h>
#include "rand.h"
#define DEBUG
#include "debug.h"

#define lightstep 10
#define motorstep 2
uint16_t period;
uint16_t prevlight;
uint16_t curlight;
uint16_t avglight;
uint16_t minlight1;
uint16_t minlight2;
uint16_t mintime1, mintime2;
uint8_t left, right;

enum {
    START,
    START_ROTATE_LEFT,
    FIND_FIRST_MIN,
    FIND_SECOND_MIN,
    ADJUST_SPEED,
} state;

void program_init() {
    state = START_ROTATE_LEFT;
    prevlight = 0;
    curlight = 0;
    minlight1 = 1000;
    minlight2 = 1000;
    left = 0;
    right = 0;
}

void program_loop() {
    switch(state) {
        case START:
            set_color(1,1,1);
            _delay_ms(1000);
            state = START_ROTATE_LEFT;
            break;
        case START_ROTATE_LEFT:
            if (prevlight >= curlight-lightstep && prevlight <= curlight+lightstep) {
                left += motorstep;
                set_motors(0xFF, 0);
                _delay_us(500);
                set_motors(left, right);
                _delay_ms(600);
            } else {
                state = FIND_FIRST_MIN;
            }
            break;
        case FIND_FIRST_MIN:
            if (curlight < minlight1) {
                minlight1 = curlight;
                mintime1 = kilo_clock;
            } else if (curlight > minlight1+10) {
                state = FIND_SECOND_MIN;
            }
            break;
        case FIND_SECOND_MIN:
            if (curlight < minlight2) {
                minlight2 = curlight;
                mintime2 = kilo_clock;
            } else if (curlight > minlight2+10) {
                state = ADJUST_SPEED;
            }
            break;
        case ADJUST_SPEED:
            period = mintime2-mintime1;
            printf("Period: %d, Speed: %d\n", period, left);
            left -= motorstep;
            set_motors(0xFF, 0);
            _delay_us(100);
            set_motors(left, right);
            minlight1 = 1000;
            minlight2 = 1000;
            state = FIND_FIRST_MIN;
            break;
        default:
            break;
    }


    prevlight = curlight;
    curlight = get_ambientlight();
    avglight = (curlight+prevlight)/2;
    printf("Avg Light: %d, last period: %d\n", avglight, period);
}

int main() {
    program_init();
    kilo_init();
    debug_init();
    kilo_loop();

    return 0;
}
