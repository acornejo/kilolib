#include "kilolib.h"
#include "bitfield.h"
#include <avr/io.h>
#include <avr/boot.h>
#include <avr/delay.h>

int8_t  page_count = 0;
int8_t  page_address = 0;
int16_t page_word_count = 0;
int16_t page_buffer[SPM_PAGESIZE];
BF_create(page_table, 220);

inline void goto_program() {
    MCUCR = (1<<IVCE);
    MCUCR = 0;
    asm volatile ("jmp 0x0000");
}

void process_message(message_t *msg) {
    if (msg->type == BOOTLOAD_MSG)
        if (msg->bootmsg.page_address != page_address) {
            page_address = msg->bootmsg.page_address;
            page_word_count = 0;
        }
        page_buffer[msg->bootmsg.page_offset] = msg->bootmsg.word1;
        page_buffer[msg->bootmsg.page_offset+1] = msg->bootmsg.word2;
        page_word_count += 2;
        if (page_word_count*2 == SPM_PAGESIZE && !BF_get(page_table, page_address)) {
            int i,j;
            eeprom_busy_wait ();

            boot_page_erase(page_address*SPM_PAGESIZE);
            boot_spm_busy_wait();

            for (i=0, j=0; i<SPM_PAGESIZE; i+=2, j++)
                boot_page_fill(page_address*SPM_PAGESIZE+i, page_buffer[j]);

            boot_page_write(page_address*SPM_PAGESIZE);
            boot_spm_busy_wait();

            boot_rww_enable ();

            BF_set(page_table, page_address);
            page_count++;
            if (page_count == 220)
                goto_program();
        }
    else {
        if (page_count == 0)
            goto_program();
    }
}

void program_loop() {}

int main() {
    // initialize hardware
    main_init();
	// move interrupt vectors to bootloader interupts
	MCUCR = (1<<IVCE);
	MCUCR = (1<<IVSEL);
    // initalize variables
    BF_init(page_table);
    page_count = 0;
    page_word_count = 0;

    while(1) {
        set_color(3,0,0);
        _delay_ms(1);
        set_color(0,0,0);
        _delay_ms(1);
    }

    return 0;
}
