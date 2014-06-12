#include "kilolib.h"
#include "bitfield.h"
#include "bootldr.h"
#include "message_send.h"
#include <avr/interrupt.h>  // for cli/sei
#include <avr/io.h>         // for port and register definitions
#include <avr/boot.h>       // to write boot pages
#include <util/delay.h>     // for delay_ms

uint8_t  page_total;
uint8_t  page_count;
uint8_t  page_address;
uint16_t page_byte_count;
uint16_t page_buffer[SPM_PAGESIZE/2+2];
bootmsg_t *bootmsg;
BF_create(page_table, 224);

void goto_program() {
    MCUCR = (1<<IVCE);
    MCUCR = 0;
    asm volatile ("jmp 0x0000");
}

void message_rx(message_t *msg, distance_measurement_t *dist) {
    if (msg->type == BOOTPGM_PAGE) {
        bootmsg = (bootmsg_t*)msg->data;
        if (page_address != bootmsg->page_address) {
            page_address = bootmsg->page_address;
            page_byte_count = 0;
        }
        page_buffer[bootmsg->page_offset] = bootmsg->word1;
        page_buffer[bootmsg->page_offset+1] = bootmsg->word2;
        page_buffer[bootmsg->page_offset+2] = bootmsg->word3;
        page_byte_count += 6;
        if (page_byte_count >= SPM_PAGESIZE && !BF_get(page_table, page_address)) {
            /**
             * Write program page to flash.
             *
             * Taken from http://www.nongnu.org/avr-libc/user-manual/group__avr__boot.html
             */

            eeprom_busy_wait ();

            boot_page_erase(page_address*SPM_PAGESIZE);
            boot_spm_busy_wait();

            int i,j;
            for (i=0, j=0; i<SPM_PAGESIZE; i+=2, j++)
                boot_page_fill(page_address*SPM_PAGESIZE+i, page_buffer[j]);

            boot_page_write(page_address*SPM_PAGESIZE);
            boot_spm_busy_wait();

            boot_rww_enable ();

            set_color(RGB(0,3,0));
            BF_set(page_table, page_address);
            page_count++;
            if (page_count == page_total)
                goto_program();
        }
        else
            set_color(RGB(0,0,1));
    } else if (msg->type == BOOTPGM_SIZE) {
        page_total = msg->data[0];
        if (page_count == page_total)
            goto_program();
    } else if (msg->type == BOOT) {
            asm volatile ("jmp 0x7000");
    } else {
        if (page_count == 0)
            goto_program();
    }
}

int main() {
    cli();
    // move interrupt vectors to bootloader interupts
    MCUCR = (1<<IVCE);
    MCUCR = (1<<IVSEL);
    // initalize variables
    BF_init(page_table);
    page_total = 220;
    page_count = 0;
    page_address = 0;
    page_byte_count = 0;
    sei();
    kilo_message_rx = message_rx;
    // initialize hardware
    kilo_init();

    // flash blue led
    while(1) {
        set_color(RGB(0,0,3));
        _delay_ms(5);
        set_color(RGB(0,0,0));
        _delay_ms(1000);
    }

    return 0;
}
