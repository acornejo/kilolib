#include "kilolib.h"

// Called when the messaging subsystem receives a message
void message_rx(message_t *msg, distance_measurement_t *dist) { }

// Called when the messaging subsystem decides to broadcast a message
message_t *message_tx() { return '\0'; }

// Called upon a successful message transmission.
void message_tx_success() { }

void setup() {
    // put your setup code here, will be run once at the beginning
}

void loop() {
    // put your main code here, will be run repeatedly

    static int state = 0;
    static uint32_t last_change=0;
    if (kilo_ticks > last_change + 32) {
        last_change = kilo_ticks;
        state++;
        if (state > 5) state = 0;
    }
    switch(state) {
        case 0:
            set_color(RGB(0,0,0));
            break;
        case 1:
            set_color(RGB(0,0,1));
            break;
        case 2:
            set_color(RGB(0,1,0));
            break;
        case 3:
            set_color(RGB(0,1,1));
            break;
        case 4:
            set_color(RGB(1,0,0));
            break;
        case 5:
            set_color(RGB(1,0,1));
            break;
        case 6:
            set_color(RGB(1,1,0));
            break;
        case 7:
            set_color(RGB(1,1,1));
            break;
        default:
            break;

    }
}

int main() {
    kilo_init(message_rx, message_tx, message_tx_success);
    kilo_start(setup, loop);

    return 0;
}
