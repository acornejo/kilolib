#include "kilolib.h"

void message_rx(message_t *msg, distance_measurement_t *dist) { }

// Called when the messaging subsystem decides to broadcast a message
message_t *message_tx() { return '\0'; }

// Called upon a successful message transmission.
void message_tx_success() { }

void program_loop() {
}

int main() {
    kilo_init(message_rx, message_tx, message_tx_success);
    kilo_loop(program_loop);

    return 0;
}
