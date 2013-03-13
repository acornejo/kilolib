#include "kilolib.h"
#include <avr/eeprom.h>
#include <util/delay.h>
#include "rand.h"
#define DEBUG
#include "debug.h"

#define IDEALPERIOD 5*TICKS_PER_SEC
#define MAXLIGHT 1000
#define MINLIGHT 0
#define LIGHTSTEP 20
#define MOTORSTEP 5

uint16_t period;
uint16_t prev_minticks;
uint16_t prevlight;
uint16_t curlight;
uint16_t avglight;
uint16_t minlight;
uint16_t maxlight;
uint16_t minticks;
uint8_t precision;
uint8_t left, right;
uint8_t i;

enum {
    START_STATE,
    FIND_THRESHOLD,
    FIND_START_MIN,
    FIND_REAL_MIN,
    END_STATE
} state;

void program_init() {
    state = START_STATE;
    curlight = 0;
    minlight = MAXLIGHT;
    maxlight = MINLIGHT;
    precision = 0;
    prev_minticks = 0;
    left = 0;
    right = 0;
}

inline void update_sensors() {
    prevlight = curlight;
    curlight = get_ambientlight();
    avglight = (curlight+prevlight)/2;
}

void program_loop() {
    update_sensors();

    switch(state) {
        case START_STATE:
            for (i = 0; i < 10; i++) {
                set_color(RGB(1,1,0));
                _delay_ms(100);
                set_color(RGB(0,0,0));
                _delay_ms(100);
                update_sensors();
            }
            state = FIND_THRESHOLD;
            break;
        case FIND_THRESHOLD:
            if (curlight > prevlight+LIGHTSTEP/2 || curlight < prevlight-LIGHTSTEP/2) {
                // Ensure we overshoot power. Otherwise we can get stuck
                // on tiny surface glitches without completing any
                // turns.
                left += MOTORSTEP;
                state = FIND_START_MIN;
            } else {
                left += MOTORSTEP;
                set_color(RGB(1,0,0));
                set_motors(0xFF, 0);
                _delay_us(1000);
                set_color(RGB(0,0,0));
                set_motors(left, 0);
                _delay_ms(1000);
            }
            break;
        case FIND_START_MIN:
            set_color(RGB(0,1,0));
            if (curlight < minlight)
                minlight = curlight;
            if (curlight > maxlight)
                maxlight = curlight;
            if (curlight > minlight+LIGHTSTEP && curlight < maxlight-LIGHTSTEP) {
                state = FIND_REAL_MIN;
                minlight = MAXLIGHT;
            }
            break;
        case FIND_REAL_MIN:
            set_color(RGB(0,0,1));
            if (curlight < minlight) {
                minlight = curlight;
                minticks = kilo_ticks;
            }
            if (curlight> minlight+LIGHTSTEP) {
                if (prev_minticks != 0) {
                    period = prev_minticks - minticks;
                    if (period > IDEALPERIOD) {
                        left += MOTORSTEP>>precision;
                    } else {
                        left -= MOTORSTEP>>precision;
                    }
                    precision++;
                    if (!(MOTORSTEP>>precision))
                        state = END_STATE;
                }
                prev_minticks = minticks;
                minlight = MAXLIGHT;
                maxlight = MINLIGHT;
                state = FIND_START_MIN;
            }
            break;
        case END_STATE:
            set_color(RGB(1,1,1));
            _delay_ms(5);
            set_color(RGB(0,0,0));
            _delay_ms(300);
            printf("Period: %d, Speed: %d\n", period, left);
            break;
        default:
            break;
    }
}

int main() {
    program_init();
    kilo_init();
    debug_init();
    kilo_loop();

    return 0;
}
