#include <stdint.h>
#include "messages.h"

#define RGB(r,g,b) (r&3)|(((g&3)<<2))|((b&3)<<4)

// Updated roughly 30 times per second
volatile uint32_t kilo_ticks;
#define TICKS_PER_SEC 31

void set_motors(uint8_t, uint8_t);
void set_color(uint8_t);
int16_t get_voltage();
int16_t get_ambientlight();

void process_message(message_t *);

void txbuffer_push(message_t *);
uint8_t txbuffer_size();

void rxbuffer_pop(message_t *);
uint8_t rxbuffer_size();

void kilo_init();
void kilo_loop(void (*program)(void));
