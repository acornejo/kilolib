#include "messages.h"

void set_motors(int8_t, int8_t);
void set_color(int8_t, int8_t, int8_t);
int get_voltage();
int get_ambientlight();

void process_message(message_t *);

void txbuffer_push(message_t *);
uint8_t txbuffer_size();

void rxbuffer_pop(message_t *);
uint8_t rxbuffer_size();

void kilo_init();
void kilo_loop();
extern void program_loop();
