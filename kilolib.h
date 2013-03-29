#include <stdint.h>
#include "message.h"

#define RGB(r,g,b) (r&3)|(((g&3)<<2))|((b&3)<<4)

#define TICKS_PER_SEC 31
volatile uint32_t kilo_ticks;
uint16_t kilo_uid;

void set_motors(uint8_t, uint8_t);
void set_color(uint8_t);
int16_t get_voltage();
int16_t get_ambientlight();

typedef struct {
    int16_t low_gain;
    int16_t high_gain;
} distance_measurement_t;

void process_message(message_t *);

void txbuffer_push(message_t *);
uint8_t txbuffer_size();

message_t *rxbuffer_peek();
void rxbuffer_pop(message_t *, distance_measurement_t *d);
uint8_t rxbuffer_size();

void kilo_init();
void kilo_loop(void (*program)(void));
