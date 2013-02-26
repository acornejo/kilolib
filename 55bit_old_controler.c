#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <avr/boot.h>

#define BLUE     0
#define LED1     1
#define LED2     2

#define F_CRYSTAL 8000000



#define BAUDRATE 19200 //Set baud rate
#define UBRRVAL ((F_CRYSTAL/(BAUDRATE*16UL))-1)
#define ee_OSCCAL 0x001  // rc calibration value in eeprom, to be loaded to OSCCAL at startup


void USARTinit();
int send_message(int,int,int);

int send_message_hamming(int,int,int,int,int,int,int,int,int,int,int);
//int send_message_hamming(int,int,int,int,int);

void color_led(int);

static volatile	int ReceivedByte;
static volatile	int message_incoming;


int main()
{
	DDRB = 0xFF; //Set port B to output.
	DDRC = 0x01; //Set PC0 to output.
	DDRD = 0x12; //Set PD4 and PD1 to output.

	//ensure ir led is off
	PORTD &= ~(1<<4);

	//set up rc oscillator
//	eeprom_write_byte((uint8_t *)ee_OSCCAL,0x63);
	OSCCAL=eeprom_read_byte((uint8_t *)ee_OSCCAL);//read rc calibration data from eeprom, and write it to OSCCAL
//	OSCCAL=0x63;
	//move interrupt vectors to bootloader interupts
	MCUCR = (1<<IVCE);//allow changing IVSEL
	MCUCR = (1<<IVSEL);//move interrupt vectors to start of boot loader

	//initalize uart
	USARTinit();


	color_led(0);
	_delay_ms(200);
	color_led(1);
	_delay_ms(200);
	color_led(0);
	_delay_ms(200);
	color_led(1);
	_delay_ms(200);
	color_led(0);
	_delay_ms(200);
	color_led(1);
	_delay_ms(200);
	color_led(0);

	while(1)
	{


		if(ReceivedByte!=0)//new message in uart
		{
			//receivedbyte messages
			//0x01 start bootloader
			//0x02 sleep
			//0x03 wakeup, go to mode 0x04
			//0x04 Robot on, but does nothing active
			//0x05 display battery voltage
			//0x06 execute program code
			//0x07 battery charge
			//0x08 stop current action
			//0x09 toggle blue leds
			//0x0a txjump to bootloader message


			if(ReceivedByte=='a')
			{
				ReceivedByte=0;//clear ReceivedByte

				color_led(1);
				//send boot load message first
				for(int i=0;i<100;i++)
				{
					send_message_hamming(140,0,139,128,139,128,139,128,0,64,236);
					_delay_ms(5);
				}
				color_led(0);
			//	_delay_ms(4000);
			//	send_message(1,0,250);
				_delay_ms(2000);

				//next bootload untill uart says to stop
				while(ReceivedByte==0)
				{
					cli();
					for(uint8_t page=0;page<220;page++)
					{
						uint16_t checksum=page;
						send_message(page,0,250);
						_delay_ms(1);
				        for (int i=0; i<SPM_PAGESIZE; i+=2)
				        {
				            // Set up little-endian word.

				            int high =pgm_read_byte(0x0000+i+page*0x0080);
							int low =pgm_read_byte(0x0000+i+page*0x0080+1);
				          	send_message(high,low,0);
							checksum+=high;
							checksum+=low;
							//_delay_ms(1);


				        }
						_delay_ms(1);
						uint8_t checksum_low=checksum;
						uint8_t checksum_high=checksum>>8;
						send_message(checksum_high,checksum_low,254);
						sei();//allow for uart to rx
						//PORTB |= (1<<LED1);
						color_led(1);
						_delay_ms(10);
						color_led(0);
						//	PORTB &= ~(1<<LED1);
						if(ReceivedByte!=0)
							break;

						cli();
					}
				}
				sei();

			}
			else if(ReceivedByte=='b')
			{
				ReceivedByte=0;//clear ReceivedByte

				//send sleep message
				while(ReceivedByte==0)
				{
					color_led(1);
					send_message_hamming(146,0,139,128,139,128,139,128,0,64,236);
					sei();
					_delay_ms(3);
					color_led(0);
					_delay_ms(2);

				}

			}
			else if(ReceivedByte=='c')
			{
				ReceivedByte=0;//clear ReceivedByte

				//send wakeup message untill uart says to stop
				while(ReceivedByte==0)
				{
					color_led(1);
					send_message_hamming(149,128,139,128,139,128,139,128,0,64,234);
					sei();
					_delay_ms(3);
					color_led(0);
					_delay_ms(3);


				}

			}
			else if(ReceivedByte=='d')
			{
				ReceivedByte=0;//clear ReceivedByte

						//send wakeup message untill uart says to stop
				while(ReceivedByte==0)
				{
					color_led(1);
					send_message_hamming(161,0,139,128,139,128,139,128,0,64,236);
					sei();
					_delay_ms(2);
					color_led(0);
					_delay_ms(1);


				}







			}
			else if(ReceivedByte=='e')
			{
				ReceivedByte=0;//clear ReceivedByte

				//send message
				while(ReceivedByte==0)
				{
					color_led(1);
					send_message_hamming(166,128,139,128,139,128,139,128,0,64,234);
					sei();
					_delay_ms(3);
					color_led(0);
					_delay_ms(2);

				}

			}
			else if(ReceivedByte=='f')
			{
				ReceivedByte=0;//clear ReceivedByte



				while(ReceivedByte==0)
				{

					color_led(1);
					send_message_hamming(184,128,139,128,139,128,139,128,0,64,234);
					sei();
					_delay_ms(2);
					color_led(0);
					_delay_ms(1);

				}


			}else if(ReceivedByte=='g')
			{
				ReceivedByte=0;//clear ReceivedByte

				//send message
				while(ReceivedByte==0)
				{
					color_led(1);
						send_message_hamming(191,0,139,128,139,128,139,128,0,64,104);
					sei();
					_delay_ms(1);
					color_led(0);
					_delay_ms(2);

				}


			}
			else if(ReceivedByte=='i')
			{
				ReceivedByte=0;//clear ReceivedByte

				static int test1=0;





					if(test1== 0)
					{
						color_led(1);
						test1=1;

						}
					else
					{
						color_led(0);
						test1=0;
						}

			}
			else if(ReceivedByte=='j')
			{
				ReceivedByte=0;//clear ReceivedByte

				//send jump to bootlaoder message untill uart says to stop
				while(ReceivedByte==0)
				{
					color_led(1);
					send_message_hamming(140,0,139,128,139,128,139,128,0,64,236);
					sei();
					_delay_ms(5);
					color_led(0);
					_delay_ms(1);


				}

			}
			else if(ReceivedByte=='z')
			{
				ReceivedByte=0;//clear ReceivedByte

				//send message
				for(int i=0;i<100;i++)
				{
					color_led(1);
					send_message_hamming(192,128,139,128,139,128,139,128,0,64,236);
					sei();
					_delay_ms(1);
					color_led(0);
					_delay_ms(2);

				}


			}



		}









	}

return 0;

} //End of main



//*************************************************************
//* Initalization of the USART for communication with the GUI *
//* Dependencies: Global interrupts must be enabled           *
//* Notes: Disable and clear interrupts during initialization *
//*************************************************************

void USARTinit()
{

	cli();					//Disable global interrupts.

	UCSR0B |= (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0);
							// Enable reception, enable reception interrupt,
	                        // enable tranmission. Note: Transmission is enabled
						    // to ensure that the tx pin pulls high. This ensures that
						    // the USB transceiver does not get stuck.
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
	 					    // Async, parity disabled, 8 bit communication, 1 stop bit

    UBRR0 = UBRRVAL;	    //Set buad rate
                            //This must be set as the last parameter.

	sei(); 					//Enable global interrupts

} //End of USARTinit

void color_led(int a)
{
	if(a==1)
	{
	PORTC = (1<<BLUE);
	PORTB |= (1<<LED1);


	}
	else
	{

	PORTC = (0<<BLUE);
	PORTB &= ~(1<<LED1);

	}

}
/*int send_message_hamming(int a,int b,int c,int d,int e)
{



	sei();




	uint16_t data_out[5];
	uint8_t data_to_send[5];




	data_to_send[0]=a;

	data_to_send[1]=b;

	data_to_send[2]=c;

	data_to_send[3]=d;

	data_to_send[4]=e;

	int message_incoming=0;

	//prepare data to send
	for(int i=0;i<5;i++)
	{
		data_out[i]=(data_to_send[i] & (1<<0))*128 +
				(data_to_send[i] & (1<<1))*32 +
				(data_to_send[i] & (1<<2))*8 +
				(data_to_send[i] & (1<<3))*2+
				(data_to_send[i] & (1<<4))/2+
				(data_to_send[i] & (1<<5))/8 +
				(data_to_send[i] & (1<<6))/32 +
				(data_to_send[i] & (1<<7))/128;

		data_out[i]=data_out[i]<<1;
		data_out[i]++;
	}




	uint8_t collision_detected=0;
	cli();//start critical
	if(message_incoming==0)//no incoming message detected
	{
		//send start pulse
		PORTD |= (1<<4);
		asm volatile("nop\n\t");
		asm volatile("nop\n\t");
		PORTD &= ~(1<<4);

		//wait for own signal to die down
		for(int k=0;k<53;k++)
			asm volatile("nop\n\t");


		//check for collision
		for(int k=0;k<193;k++)
		{
			if((ACSR & (1<<ACO))>0)
			{

				asm volatile("nop\n\t");

			}
			if((ACSR & (1<<ACO))>0)
			{

				asm volatile("nop\n\t");

			}


		}

		if(collision_detected==0)
			for(int byte_sending=0;byte_sending<5;byte_sending++)
			{
				int i=8;
				while(i>=0)
				{

					if(data_out[byte_sending] & 1)
					{
						PORTD |= (1<<4);
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");

					}
					else
					{
						PORTD &= ~(1<<4);
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");

					}

						PORTD &= ~(1<<4);
						for(int k=0;k<35;k++)
						{
							asm volatile("nop\n\t");
						}

						data_out[byte_sending]=data_out[byte_sending]>>1;
						i--;
				}
				asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
									asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
									asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
							asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
									asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
									asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");
			}
	}
	else//channel was in use before starting
	{
		//ensure led is off
		PORTD &= ~(1<<4);

		sei();//end critical
		return(5);
	}

	//ensure led is off
	PORTD &= ~(1<<4);
	//_delay_ms(10);
	if(collision_detected==0)
	{
		//wait for own signal to die down
		for(int k=0;k<50;k++)
			asm volatile("nop\n\t");

			ACSR |= (1<<ACI);

		sei();//end critical
		return(1);
	}
	else
	{

		sei();//end critical
		return(0);
	}


}*/
int send_message(int a,int b,int c)
{
	sei();







	uint16_t data_out[4];
	uint8_t data_to_send[4]={a,b,c,255};
//	uint8_t data_to_send[4]={255,255,255,255};


	//prepare data checksum to send
	data_to_send[3]=data_to_send[2]+data_to_send[1]+data_to_send[0]+128;

	//prepare data to send
	for(int i=0;i<4;i++)
	{
		data_out[i]=(data_to_send[i] & (1<<0))*128 +
				(data_to_send[i] & (1<<1))*32 +
				(data_to_send[i] & (1<<2))*8 +
				(data_to_send[i] & (1<<3))*2+
				(data_to_send[i] & (1<<4))/2+
				(data_to_send[i] & (1<<5))/8 +
				(data_to_send[i] & (1<<6))/32 +
				(data_to_send[i] & (1<<7))/128;

		data_out[i]=data_out[i]<<1;
		data_out[i]++;
	}

	uint8_t collision_detected=0;
		uint8_t message_incoming=0;
	cli();//start critical
	if(message_incoming==0)//no incoming message detected
	{
		//send start pulse
		PORTD |= (1<<4);
		asm volatile("nop\n\t");
		asm volatile("nop\n\t");
		PORTD &= ~(1<<4);

		//wait for own signal to die down
		for(int k=0;k<53;k++)
			asm volatile("nop\n\t");


		//check for collision
		for(int k=0;k<193;k++)
		{
			if((ACSR & (1<<ACO))>0)
			{

				asm volatile("nop\n\t");

			}
			if((ACSR & (1<<ACO))>0)
			{

				asm volatile("nop\n\t");

			}


		}

		if(collision_detected==0)
			for(int byte_sending=0;byte_sending<4;byte_sending++)
			{
				int i=8;
				while(i>=0)
				{

					if(data_out[byte_sending] & 1)
					{
						PORTD |= (1<<4);
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");

					}
					else
					{
						PORTD &= ~(1<<4);
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");

					}

						PORTD &= ~(1<<4);
						for(int k=0;k<35;k++)
						{
							asm volatile("nop\n\t");
						}

						data_out[byte_sending]=data_out[byte_sending]>>1;
						i--;
				}
			}
	}
	else//channel was in use before starting
	{
		//ensure led is off
		PORTD &= ~(1<<4);

		sei();//end critical
		return(5);
	}
	//ensure led is off
	PORTD &= ~(1<<4);
	if(collision_detected==0)
	{
		//wait for own signal to die down
		for(int k=0;k<50;k++)
			asm volatile("nop\n\t");

			ACSR |= (1<<ACI);

		sei();//end critical
		return(1);
	}
	else
	{

		sei();//end critical
		return(0);
	}



}
int send_message_hamming(int a,int b,int c,int d,int e,int f,int g,int h,int i,int j,int k)
{

	sei();


	uint16_t data_out[11];

	data_out[0]=(a<<1)+1;
	data_out[1]=(b<<1)+1;
	data_out[2]=(c<<1)+1;
	data_out[3]=(d<<1)+1;
	data_out[4]=(e<<1)+1;
	data_out[5]=(f<<1)+1;
	data_out[6]=(g<<1)+1;
	data_out[7]=(h<<1)+1;
	data_out[8]=(i<<1)+1;
	data_out[9]=(j<<1)+1;
	data_out[10]=(k<<1)+1;

	int message_incoming=0;
	uint8_t collision_detected=0;
	cli();//start critical
	if(message_incoming==0)//no incoming message detected
	{
		//send start pulse
		PORTD |= (1<<4);
		asm volatile("nop\n\t");
		asm volatile("nop\n\t");
		PORTD &= ~(1<<4);

		//wait for own signal to die down
		for(int k=0;k<53;k++)
			asm volatile("nop\n\t");


		//check for collision
		for(int k=0;k<193;k++)
		{
			if((ACSR & (1<<ACO))>0)
			{

				asm volatile("nop\n\t");

			}
			if((ACSR & (1<<ACO))>0)
			{

				asm volatile("nop\n\t");

			}


		}

		if(collision_detected==0)
			for(int byte_sending=0;byte_sending<11;byte_sending++)
			{
				int i=8;
				while(i>=0)
				{

					if(data_out[byte_sending] & 1)
					{
						PORTD |= (1<<4);
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");

					}
					else
					{
						PORTD &= ~(1<<4);
						asm volatile("nop\n\t");
						asm volatile("nop\n\t");

					}

						PORTD &= ~(1<<4);
						for(int k=0;k<35;k++)
						{
							asm volatile("nop\n\t");
						}

						data_out[byte_sending]=data_out[byte_sending]>>1;
						i--;
				}

				_delay_us(300);
			}
	}
	else//channel was in use before starting
	{
		//ensure led is off
		PORTD &= ~(1<<4);

		sei();//end critical
		return(5);
	}

	//ensure led is off
	PORTD &= ~(1<<4);
	//_delay_ms(10);
	if(collision_detected==0)
	{
		//wait for own signal to die down
		for(int k=0;k<50;k++)
			asm volatile("nop\n\t");

			ACSR |= (1<<ACI);

		sei();//end critical
		return(1);
	}
	else
	{

		sei();//end critical
		return(0);
	}









}

/*
int send_message1(int a,int b,int c)
{
       DDRD |= (1<<4);
       uint16_t data_out[4];
       uint8_t data_to_send[4]={a,b,c,255};

       cli();//start critical

       //prepare data checksum to send
       data_to_send[3]=data_to_send[2]+data_to_send[1]+data_to_send[0]+128;

       //prepare data to send
       for(int i=0;i<4;i++)
       {
               data_out[i]=(data_to_send[i] & (1<<0))*128 +
                               (data_to_send[i] & (1<<1))*32 +
                               (data_to_send[i] & (1<<2))*8 +
                               (data_to_send[i] & (1<<3))*2+
                               (data_to_send[i] & (1<<4))/2+
                               (data_to_send[i] & (1<<5))/8 +
                               (data_to_send[i] & (1<<6))/32 +
                               (data_to_send[i] & (1<<7))/128;

               data_out[i]=data_out[i]<<1;
               data_out[i]++;
       }

       uint8_t collision_detected=0;

       if(message_incoming==0)//no incoming message detected
       {
               //send start pulse
               PORTD |= (1<<4);
               asm volatile("nop\n\t");
               asm volatile("nop\n\t");
               PORTD &= ~(1<<4);

               //wait for own signal to die down
               for(int k=0;k<50;k++)
                       asm volatile("nop\n\t");


               //check for collision
               for(int k=0;k<193;k++)
               {
                       if((ACSR & (1<<ACO))>0)
                               collision_detected=1;
                       if((ACSR & (1<<ACO))>0)
                               collision_detected=1;

               }

               if(collision_detected==0)
                       for(int byte_sending=0;byte_sending<4;byte_sending++)
                       {
                               int i=8;
                               while(i>=0)
                               {

                                       if(data_out[byte_sending] & 1)
                                       {
                                               PORTD |= (1<<4);
                                               asm volatile("nop\n\t");
                                               asm volatile("nop\n\t");

                                       }
                                       else
                                       {
                                               PORTD &= ~(1<<4);
                                               asm volatile("nop\n\t");
                                               asm volatile("nop\n\t");

                                       }

                                               PORTD &= ~(1<<4);
                                               for(int k=0;k<35;k++)
                                               {
                                                       asm volatile("nop\n\t");
                                               }

                                               data_out[byte_sending]=data_out[byte_sending]>>1;
                                               i--;
                               }
                       }
       }
       else//channel was in use before starting
       {
               //ensure led is off
               PORTD &= ~(1<<4);
               sei();//end critical
               return(2);
       }
       //ensure led is off
       PORTD &= ~(1<<4);
       if(collision_detected==0)
       {
               //wait for own signal to die down
               for(int k=0;k<100;k++)
                       asm volatile("nop\n\t");

                       ACSR |= (1<<ACI);

               sei();//end critical
               return(1);
       }
       else
       {
               sei();//end critical
               return(0);
       }



}


*/
//*************************************************************
//* Interrupt Service Routine for USART reception             *
//* Dependencies: Global interrupts must be enabled           *
//* Notes: The receive interrupt flag is automatically        *
//* cleared when the receive buffer is read                   *
//*************************************************************

ISR(USART_RX_vect)
	{

		ReceivedByte = UDR0; // Fetch the recieved byte (and clear the interrupt flag)
	PORTB |= (1<<LED1);

	//	if (ReceivedByte == 'c')
	//		PORTC = (1<<BLUE);
	//		_delay_ms(100);
       // if (ReceivedByte == 'd')
	//		PORTC = (0<<BLUE);

	} //End of USART interrupt service

