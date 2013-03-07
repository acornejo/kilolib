#include "kilolib.h"
#include "bitfield.h"
#include <avr/io.h>
#include <avr/boot.h>

static int8_t page_count = 0;
static int8_t page_address = 0;
static int16_t page_word_count = 0;
static int16_t page_buffer[SPM_PAGESIZE];
BF_create(page_table, 220);

void inline goto_program() {
    MCUCR = (1<<IVCE);
    MCUCR = 0;
    asm volatile ("jmp 0x0000");
}

inline void write_page() {
    int i,j;
    int real_address = page_address*SPM_PAGESIZE;
    eeprom_busy_wait ();

    boot_page_erase(page_address*SPM_PAGESIZE);
    boot_spm_busy_wait();

    for (i=0, j=0; i<SPM_PAGESIZE; i+=2, j++)
        boot_page_fill(real_address+i, page_buffer[j]);

    boot_page_write(real_address); // Store buffer in flash page.
    boot_spm_busy_wait();          // Wait until the memory is written.

    boot_rww_enable ();
}

void process_specialmessage(message_t *msg) {
    if (msg->type == BOOTLOAD_MSG)
        if (msg->page_address != page_address) {
            page_address = msg->page_address;
            page_word_count = 0;
        }
        page_buffer[msg->page_offset] = msg->word1;
        page_buffer[msg->page_offset+1] = msg->word2;
        page_word_count += 4;
        if (page_word_count*2 == SPM_PAGESIZE && !BF_get(page_table, page_address)) {
            write_page();
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
    // initalize page table
    BF_init(page_table);
	// move interrupt vectors to bootloader interupts
	MCUCR = (1<<IVCE);
	MCUCR = (1<<IVSEL);
    main_loop();
    return 0;
}
