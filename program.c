#include "kilolib.h"
#include <util/delay.h>

message_t msg;
distance_measurement_t dist;

void program_loop() {
    // If no message on tx buffer
    if (txbuffer_size() == 0) {
        // insert dummy message
        txbuffer_push(&msg);
    }

    // If message in RX buffer
    if (rxbuffer_size() > 0) {
        // Pop message from RX buffer
        rxbuffer_pop(&msg, &dist);
        // Move forward
        set_motors(255,255);
        _delay_ms(20);
        set_motors(0x80, 0x80);
    } else {
        // stop
        set_motors(0,0);
    }

    // read ambient light sensor
    int16_t light = get_ambientlight();

    if (light < 200) { // if dark
        // blink led blue
        set_color(RGB(0,0,1));
        _delay_ms(200);
        set_color(RGB(0,0,0));
        _delay_ms(800);
    } else {          // if bright
        // blink led white
        set_color(RGB(1,1,1));
        _delay_ms(200);
        set_color(RGB(0,0,0));
        _delay_ms(800);
    }

    // read voltage sensor
    int16_t voltage = get_voltage();
    if (voltage < 300) { // if low voltage blink red
        set_color(RGB(1,0,0));
        _delay_ms(200);
        set_color(RGB(0,0,0));
        _delay_ms(800);
    } else {              // if normal voltage blink green
        set_color(RGB(0,1,0));
        _delay_ms(200);
        set_color(RGB(0,0,0));
        _delay_ms(800);
    }

}

int main() {
    kilo_init();
    kilo_loop(program_loop);

    return 0;
}
