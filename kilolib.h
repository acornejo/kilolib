#include "messages.h"

void set_motors(int8_t, int8_t);
void set_color(int8_t, int8_t, int8_t);
int get_voltage();
int get_ambientlight();

void txbuffer_push(message_t *);
uint8_t txbuffer_size();

void rxbuffer_pop(message_t *);
uint8_t rxbuffer_size();

void main_init();
void main_loop();
void process_message(message_t *);
extern void program_loop();
