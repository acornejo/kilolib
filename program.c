#include "kilolib.h"
#include <util/delay.h>

void program_loop() {
    // read ambient light sensor
    int16_t light = get_ambientlight();

    if (light < 200) { // if dark
        // turn left
        set_motors(255, 0);
        _delay_ms(20);
        set_motors(0x80, 0);
        _delay_ms(500);
    } else {          // if bright
        // go straight
        set_motors(255, 255);
        _delay_ms(20);
        set_motors(0x80, 0x80);
        _delay_ms(500);
    }
    // stop
    set_motors(0,0);

    // read voltage sensor
    int16_t voltage = get_voltage();

    if (voltage < 300) { // if low voltage blink red
        set_color(RGB(1,0,0));
        _delay_ms(1000);
        set_color(RGB(0,0,0));
    } else {              // if normal voltage blink green
        set_color(RGB(0,1,0));
        _delay_ms(1000);
        set_color(RGB(0,0,0));
    }

    // blink leds 10 times during 10 seconds
    // in a red->green->blue pattern;
    int8_t i;
    for (i = 0; i < 10; i++) {
        set_color(RGB(1,0,0));
        _delay_ms(333);
        set_color(RGB(0,1,0));
        _delay_ms(333);
        set_color(RGB(0,0,1));
        _delay_ms(333);
    }
    // turn off leds
    set_color(RGB(0,0,0));
}

int main() {
    kilo_init();
    kilo_loop(program_loop);

    return 0;
}
