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
#define MOTORSTEP 10

uint16_t period;
uint16_t prev_minticks;
uint16_t prevlight;
uint16_t curlight;
uint16_t avglight;
uint16_t minlight;
uint16_t maxlight;
uint16_t minticks;
uint8_t left;
uint8_t right;
uint8_t step;
uint8_t speed;
uint8_t i;

enum {
    START_STATE,
    FIND_THRESHOLD,
    FIND_START_MIN,
    FIND_REAL_MIN,
    END_STATE
} state;

enum {
    LEFT,
    RIGHT,
    NONE
} motor;

void program_init() {
    state = START_STATE;
    motor = LEFT;
    step = MOTORSTEP;
    curlight = 0;
    minlight = MAXLIGHT;
    maxlight = MINLIGHT;
    prev_minticks = 0;
    speed = 0;
    left = 0;
    right = 0;
}

void update_sensors() {
    // Using low-pass filter with alpha=1/2
    prevlight = curlight;
    curlight = prevlight+(get_ambientlight()-prevlight)/2;
}

void update_motors(uint8_t speed) {
    if (motor == LEFT) {
        set_motors(0xFF, 0);
        _delay_ms(1);
        set_motors(speed, 0);
    } else if (motor == RIGHT) {
        set_motors(0, 0xFF);
        _delay_ms(1);
        set_motors(0, speed);
    } else {
        set_motors(0,0);
    }
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
            if (curlight > prevlight+LIGHTSTEP || curlight < prevlight-LIGHTSTEP) {
                // Ensure we overshoot power. Otherwise we can get stuck
                // on tiny surface glitches without completing any
                // turns.
                speed += step;
                update_motors(speed);
                state = FIND_START_MIN;
            } else {
                set_color(RGB(1,0,0));
                speed += step;
                update_motors(speed);
                set_color(RGB(0,0,0));
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
                        speed += step;
                    } else {
                        speed -= step;
                    }
                    step /= 2;
                    update_motors(speed);
                }
                prev_minticks = minticks;
                minlight = MAXLIGHT;
                maxlight = MINLIGHT;
                if (step == 0)
                    state = END_STATE;
                else
                    state = FIND_START_MIN;
            }
            break;
        case END_STATE:
            if (motor == LEFT) {
                left = speed;
                motor = RIGHT;
                speed = 0;
                step = MOTORSTEP;
                update_motors(speed);
                state = START_STATE;
            } else if (motor == RIGHT) {
                right = speed;
                motor = NONE;
                speed = 0;
                step = MOTORSTEP;
                update_motors(speed);
            }

            set_color(RGB(1,1,1));
            _delay_ms(5);
            set_color(RGB(0,0,0));
            _delay_ms(300);
            printf("Period: %d, Left Speed: %d, Right Speed: %d\n", period, left, right);
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
