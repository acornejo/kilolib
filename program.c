#include "kilolib.h"
#include <util/delay.h>

/***
 * Message transmition and reception between kilobots is handled by the
 * next three functions.
 *
 * These functions are called within the interrupts that handle kilobot
 * communication. It is important to remember that interrupts will be
 * disabled while these functions are being executed.
 *
 * For this reason, and to ensure a correct behavior of the kilobot
 * communication system, it is of utmost importance that these functions
 * have as little logic and loops as possible (preferably, none at all).
 *
 * An alternative buffered interface for kilobot communication is
 * provided in message_buffered.h. To avoid problems we recommend using
 * the buffered interface whenever possible.
 */

message_t txmsg, rxmsg;
distance_measurement_t rxdist;
uint8_t txmsg_available;
uint8_t rxmsg_processed;

// Called when the messaging subsystem successfully decodes a message
void message_rx(message_t *msg, distance_measurement_t *dist) {
    rxmsg = *msg;
    rxdist = *dist;
    rxmsg_processed = 0;
}

// Called when the messaging subsystem decides to broadcast a message
message_t *message_tx() {
    if (txmsg_available)
        return &txmsg;
    else
        return '\0';
}

// Called upon a successful message transmission.
void message_tx_success() {
    txmsg_available = 0;
}

void program_loop() {
    // read ambient light sensor
    int16_t light = get_ambientlight();

    if (light < 500) { // if dark
        // blink led blue
        set_color(RGB(0,0,1));
        _delay_ms(200);
        set_color(RGB(0,0,0));
        _delay_ms(300);
    } else {          // if bright
        // blink led green
        set_color(RGB(0,1,0));
        _delay_ms(200);
        set_color(RGB(0,0,0));
        _delay_ms(300);
    }

    if (!txmsg_available) {                  // if no message is available for sending
        txmsg.type = NORMAL;                 // set message type to normal
        txmsg.data[0] = light&0x00FF;        // store low byte of light reading in message
        txmsg.data[1] = (light&0xFF00) >> 8; // store high byte of light reading in message
        txmsg.crc = message_crc(&txmsg);     // compute the crc of the message
        txmsg_available = 1;
    }

    if (!rxmsg_processed) {                  // if received messages hasn't been processed
        uint16_t rxlight;
        rxlight = rxmsg.data[0];             // retreive low byte of light reading in message
        rxlight |= rxmsg.data[1]<<8;         // retreive high byte of light reading in message
        /* Perhaps do something with the received light value */
        rxmsg_processed = 1;                 // set processed flag

        // set motors at full speed to overcome kinetic energy
        set_motors(255,255);
        _delay_ms(20);
        // go straight for 1/4 of a second
        set_motors(65, 65);
        _delay_ms(250);
        // turn motors off
        set_motors(0,0);
    }
}

int main() {
    rxmsg_processed = 1;
    kilo_init(message_rx, message_tx, message_tx_success);
    kilo_loop(program_loop);

    return 0;
}
