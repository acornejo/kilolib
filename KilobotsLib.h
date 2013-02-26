#ifndef KILOBOTSLIB_H
#define KILOBOTSLIB_H

#define F_CPU 8000000UL //define the clock speed of the processor



#include <avr/eeprom.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <math.h>




//some timing constants
#define time_a 269
#define time_b 269
#define timeb_half time_b/2

//used locations in eeprom for calibration data
#define ee_OSCCAL 0x001  // rc calibration value in eeprom, to be loaded to OSCCAL at startup
#define ee_CW_IN_PLACE 0x004  //motor calibration data in epromm
#define ee_CCW_IN_PLACE 0x008  //motor calibration data in epromm
#define ee_CW_IN_STRAIGHT 0x00B  //motor calibration data in epromm
#define ee_CCW_IN_STRAIGHT 0x013  ///motor calibration data in epromm
#define ee_SENSOR_LOW 0x20  //low gain sensor calibration data in epromm
#define ee_SENSOR_HIGH 0x50  //high-gain calibration data in epromm
#define ee_TX_MASK 0x90  //transmission strength calibration value in eeprom
#define ee_ROLE 0xA0  //Role asignment
#define ee_ID 0xB0  //ID asignment

//function to initialize kilobots
void init_kilobot(void);
//function to handle messages sent by the controller like sleep and resume program
extern inline void controller_messages(void);

//usefull lower level functions
void kprinti(int);
void kprintin(int);
void kprints(char *);
void set_color(int8_t ,int8_t ,int8_t );
void enter_sleep(void);
int measure_voltage(void);
int measure_charge_status(void);
int send_message();
int get_message(void);
int check_message1(uint8_t,uint8_t,uint8_t,uint8_t,uint8_t,uint8_t,uint8_t,uint8_t,uint8_t,uint8_t,uint8_t,uint8_t);
int get_ambient_light(void);
uint8_t compute_distance(int,int);
void message_out(uint8_t ,uint8_t ,uint8_t ,uint8_t ,uint8_t ,uint8_t ,uint8_t );




//variables used for communication
extern volatile int EMISSION_TIME;
extern volatile uint8_t incoming_byte;
extern volatile int adc_high_gain;
extern volatile int adc_low_gain;
extern volatile uint8_t incoming_byte_value;
extern volatile uint8_t message_incoming;
extern volatile uint8_t incoming_message[13];
extern volatile uint8_t leading_bit;
extern volatile int txvalue_buffer[10][4];
extern volatile int txvalue_buffer_pointer;
extern volatile int message_rx[10];
extern volatile uint8_t bit_count[8];
extern volatile uint8_t bit_count_error;
extern volatile uint8_t tx_success;

//clock variables used with timer0
extern volatile int clock_0;
extern volatile int clock_1;
extern volatile int clock_2;
extern volatile int clock_3;
extern volatile int clock_4;
extern volatile int time_since_last;
extern volatile int tx_clock;
extern volatile int special_message_clock;

//calibration data stored here from eeprom for quick usage during program
extern volatile uint8_t	cw_in_place;
extern volatile uint8_t	ccw_in_place;
extern volatile uint8_t	cw_in_straight;
extern volatile uint8_t	ccw_in_straight;
extern volatile int sensor_cal_data_low[14];
extern volatile int sensor_cal_data_high[14];
extern volatile uint8_t tx_mask;
extern volatile int role;
extern volatile int id;

//variables used with the over head controller
extern volatile int special_mode_message;
extern volatile int special_mode;
extern volatile int special_message_buffer[3];//
extern volatile int special_message_pointer;
extern volatile int wakeup;
extern volatile uint8_t enable_tx;
extern volatile uint8_t run_program;

//variable used for generating a good random value
extern volatile int randseed;


//function inline macro for checking the received message buffer
#define get_message()											\
{																\
	int count_pointer=0;										\
	int count_pointer_a=txvalue_buffer_pointer;					\
	message_rx[9]=0;											\
	while(count_pointer<4)										\
	{															\
		if(count_pointer_a==0)									\
			count_pointer_a=3;									\
		else													\
			count_pointer_a--;									\
																\
		if(txvalue_buffer[9][count_pointer_a]==1)				\
		{														\
			cli();												\
			message_rx[0]=txvalue_buffer[0][count_pointer_a];	\
			message_rx[1]=txvalue_buffer[1][count_pointer_a];	\
			message_rx[2]=txvalue_buffer[2][count_pointer_a];	\
			message_rx[3]=txvalue_buffer[3][count_pointer_a];	\
			message_rx[4]=txvalue_buffer[4][count_pointer_a];	\
			message_rx[5]=txvalue_buffer[5][count_pointer_a];	\
			message_rx[6]=txvalue_buffer[6][count_pointer_a];	\
			message_rx[7]=txvalue_buffer[7][count_pointer_a];	\
			message_rx[8]=txvalue_buffer[8][count_pointer_a];	\
			sei();												\
			message_rx[9]=1;									\
			txvalue_buffer[9][count_pointer_a]=0;				\
			break;												\
																\
																\
																\
		}														\
																\
		count_pointer++;										\
																\
	}															\
																\
}	



//inline function macro for setting motor speeds
#define set_motor(ccw,cw) OCR2A=ccw;OCR2B=cw;

#endif
