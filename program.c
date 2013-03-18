#include "kilolib.h"
#include <avr/eeprom.h>
#include <util/delay.h>

void program_loop() {
    // read ambient light sensor
    int16_t light = get_ambientlight();
    // read voltage sensor
    int16_t voltage = get_voltage();

    // go forward for 2 seconds
    set_motors(255,255);
    _delay_ms(2000);
    // go left for 2 seconds
    set_motors(255,0);
    _delay_ms(2000);
    // go right for 2 seconds
    set_motors(0,255);
    _delay_ms(2000);
    // stop
    set_motors(0,0);

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
    kilo_loop();

    return 0;
}
