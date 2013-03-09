#include "kilolib.h"
#include <avr/eeprom.h>
/* #define DEBUG */
/* #include "debug.h" */

#define EEPROM_SENSOR_LOW 0x20
#define EEPROM_SENSOR_HIGH 0x50

uint16_t sensor_low_data[14];
uint16_t sensor_high_data[14];
uint8_t print_done;

void program_loop() {
   //read low gain sensor calibration data from eeprom, store in sensor_cal_data_low[]
    /* if (!print_done) { */
    /*     print_done = 1; */
    /*     int i; */
    /*     for(i=0;i<14;i++) */
    /*     { */
    /*         sensor_low_data[i] = */
    /*             eeprom_read_byte((uint8_t *)(EEPROM_SENSOR_LOW+i*2))<<8 | */
    /*             eeprom_read_byte((uint8_t*)(EEPROM_SENSOR_LOW+i*2+1)); */
    /*         sensor_high_data[i] = */
    /*             eeprom_read_byte((uint8_t *)(EEPROM_SENSOR_HIGH+i*2))<<8 | */
    /*             eeprom_read_byte((uint8_t*)(EEPROM_SENSOR_HIGH+i*2+1)); */
    /*     } */
    /*     printf("LOW SENSOR, HIGH_SENSOR\n"); */
    /*     for (i=0; i<14; i++) */
    /*         printf("%d, %d\n", sensor_low_data[i], sensor_high_data[i]); */
    /* } */
}

int main() {
    print_done =0;
    kilo_init();
    /* debug_init(); */
    kilo_loop();
    return 0;
}
