#ifndef __KILOLIB_H__
#define __KILOLIB_H__

#include <stdint.h>
#include "message.h"

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
extern uint16_t kilo_uid;

#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
extern "C" {
#endif

void set_motors(uint8_t, uint8_t);
void set_color(uint8_t);
int16_t get_voltage();
int16_t get_temperature();
int16_t get_ambientlight();
void kilo_init(message_rx_t, message_tx_t, message_tx_success_t);
void kilo_loop(void (*program)(void));
void kilo_run();
void kilo_reset();

#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
}
#endif

#endif//__KILOLIB_H__
