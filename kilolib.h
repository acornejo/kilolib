#ifndef __KILOLIB_H__
#define __KILOLIB_H__

#include <stdint.h>
#include "message.h"
#include "message_crc.h"

#define RGB(r,g,b) (r&3)|(((g&3)<<2))|((b&3)<<4)
#define TICKS_PER_SEC 31

typedef struct {
    int16_t low_gain;
    int16_t high_gain;
} distance_measurement_t;

typedef void (*message_rx_t)(message_t *, distance_measurement_t *d);
typedef message_t *(*message_tx_t)(void);
typedef void (*message_tx_success_t)(void);

extern volatile uint32_t kilo_ticks;
extern volatile uint16_t kilo_tx_period;
extern uint16_t kilo_uid;
extern uint8_t kilo_turn_left;
extern uint8_t kilo_turn_right;
extern uint8_t kilo_straight_left;
extern uint8_t kilo_straight_right;

#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
extern "C" {
#endif

int16_t get_voltage();
int16_t get_temperature();
int16_t get_ambientlight();
void set_motors(uint8_t, uint8_t);
void set_color(uint8_t);
void kilo_init(message_rx_t, message_tx_t, message_tx_success_t);
void kilo_loop(void (*program)(void));
void kilo_run();
void kilo_reset();

#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
}
#endif

#endif//__KILOLIB_H__
