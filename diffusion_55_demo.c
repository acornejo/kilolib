#include "KilobotsLib.h"
#include "avr/pgmspace.h"


	


int main(void)
{
	//starting initalizations	
	init_kilobot();
	//eeprom_write_byte((uint8_t *)(ee_ID),255);
	while(1)//main program loop
	{	
		
		controller_messages();//allow robot to react to messages from Overhead Controller (OHC)
	
			
		//run_program=1;
		if(run_program==1)//run_program set by overhead contoller 
		{	

			
			/////////////////////////////////////////////////////////////////////////////////////
			//user program code goes here.  this code needs to exit in a resonable amount of time
			//so the special message controller can also run
			/////////////////////////////////////////////////////////////////////////////////////
		
			enable_tx=1;
		
			static uint8_t start=0;
		
			static int neighbor=1;
		
			static int desired_movement=0;
			static int current_movement=0;
		
			message_out(1,1,1,1,1,1,0);

			
			if(start==0)//init some values at the start of execution
			{
				set_color(1,1,0);
				for(int i=0;i<40;i++)
				{
					randseed+=measure_voltage();
					_delay_ms(1);
					randseed+=get_ambient_light();
					_delay_ms(1);
				}
				srand(randseed);
				

				//clear message buffer
				get_message();
				while(message_rx[9]==1)
				{	
					get_message();
				}

				set_color(0,0,0);
				start=1;

			}
		

			//message handler sort and record incoming messages			
			get_message();
			while(message_rx[9]==1)
			{
				if(message_rx[7]>200)			
				{
					neighbor=1;
				}
			

				get_message();
			

			
			}


			//update movement state machine
			if(clock_1>5000)//wait a bit before deciding next move
			{
				
								
				if(neighbor==0)
				{
					set_color(1,0,0);
					desired_movement=0;
				}
				else if(desired_movement==0)
				{
					set_color(0,1,0);
					//randomly generate movement
					int r_value=rand()%100;
					if(r_value<50)
					{
						desired_movement=1;

					}
					else if(r_value<75)
					{
						desired_movement=2;

					}
					else
					{
						desired_movement=3;

					}



				}



				
				clock_1=0;			
				neighbor=0;


			}


			//random movement change
			if(clock_2>30000)
			{
				clock_2=0;
				int r_value=rand()%100;
				
				if((desired_movement==1)&&(r_value<20))
				{
					if((rand()%100)<50)
					{
						desired_movement=2;
					}
					else
					{
						desired_movement=3;
					}

				}
				else if((desired_movement==2)&&(r_value<50))
				{
					desired_movement=1;
				}
				else if((desired_movement==3)&&(r_value<50))
				{
					desired_movement=1;
				}

			
			
			}	


			//desired_movement=0;
			//movement handler
			if(current_movement!=desired_movement)
			{
				if(desired_movement==0)//stop
				{
					set_motor(0,0);

				}
				else if(desired_movement==1)//straight
				{
					set_motor(0xff,0xff);
					_delay_ms(20);
					set_motor(ccw_in_straight,cw_in_straight);

				}
				else if(desired_movement==2)//cw
				{
					set_motor(0,0xff);
					_delay_ms(20);
					set_motor(0,cw_in_place-5);

				}
				else if(desired_movement==3)//ccw
				{
					set_motor(0xff,0);
					_delay_ms(20);
					set_motor(ccw_in_place-5,0);

				}
				current_movement=desired_movement;


			}

		
		}

	}

}


