//include this .c file's header file
#include "Robot.h"

//static function prototypes, functions only called in this file


//file scope variables
static char serial_string[200] = {0};
volatile uint8_t dataByte1=0, dataByte2=0, dataByte3=0, dataByte4=0;    	// data bytes received
volatile bool new_message_received_flag=false;
volatile uint16_t LS = 0;
volatile uint16_t RS = 0;
volatile uint16_t FS = 0;
volatile uint16_t LSADC = 0; //sensor adc values converted into flattened curve using formula below
volatile uint16_t RSADC = 0;
volatile uint16_t FSADC = 0;


int main(void)
{
	// initialisation
	
	adc_init();
	
	serial0_init(); 	// terminal communication with PC
	serial2_init();    	// microcontroller communication to/from another Arduino
	// or loopback communication to same Arduino
	
	DDRA |= (1<<DDA0)|(1<<DDA1)|(1<<DDA2)|(1<<DDA3); //the direction control lines for the DC motors
	PORTA = 0xFF;
	
	DDRB = 0xFF;
	
	DDRE = 0xFF; //when using OCR3, the output thing is DDRE
	
	DDRC = 0xFF; //used for autonomous LED
	PORTC = 0;
	
	DDRG = 0xFF; //used for battery voltage monitor LED
	PORTG = 0;

	TCCR3A = 0; TCCR3B = 0; //DC motor set up
	TCCR3B |= (1 << WGM33); // Phase correct, 8 bit PWM
	TCCR3A |= (1 << COM3A1); // Clear on up-count, set on down-count
	TCCR3B |= (1 << CS31); //prescaler of 8

	TCCR3A |= (1 << COM1B1); // Clear on up-count, set on down-count
	
	ICR3 = 10000; //TOP value calculated
	
	
	TCCR1A = 0; TCCR1B = 0; //Servo motor set up
	TCCR1B |= (1 << WGM13); // Phase correct, 8 bit PWM
	TCCR1A |= (1 << COM1A1); // Clear on up-count, set on down-count
	TCCR1B |= (1 << CS11); //prescaler of 8

	TCCR1A |= (1 << COM1B1); // Clear on up-count, set on down-count
	
	
	ICR1 = 20000; //TOP value calculated

	
	
	
	
	static int16_t lm = 0; //signed because sometimes they'll be positive and sometimes be negative
	static int16_t rm = 0;
	
	uint8_t sendDataByte1=0, sendDataByte2=0, sendDataByte3=0, sendDataByte4=0;    	// data bytes sent
	
	uint32_t current_ms=0, last_send_ms=0;        	// used for timing the serial send
	
	UCSR2B |= (1 << RXCIE2); // Enable the USART Receive Complete interrupt (USART_RXC)
	
	milliseconds_init();
	sei();
	
	while(1)
	{
		if(adc_read(0) < 700){ //if battery voltage goes below 7 Volts
			
			PORTG |= (1<<PG2); //turn LED on
		}
		else{
			PORTG &= ~(1<<PG2); //turn LED off
		}
		
		
		
		current_ms = milliseconds;
		//sending section
		if(current_ms-last_send_ms >= 100) //sending rate controlled here one message every 100ms (10Hz)
		{
			
			sendDataByte1 = (LS)>>3; //left short sensor shifted 3 bits right
			
			sendDataByte2 = (FS)>>3; //front long sensor
			
			sendDataByte3 = (RS)>>3; //right short sensor

			sendDataByte4 = adc_read(0)>>3; //raw voltage

			last_send_ms = current_ms;
			serial2_write_byte(0xFF);     	//send start byte = 255
			serial2_write_byte(sendDataByte1); 	//send first data byte: must be scaled to the range 0-253
			serial2_write_byte(sendDataByte2); 	//send second parameter: must be scaled to the range 0-253
			serial2_write_byte(sendDataByte3); 	//send first data byte: must be scaled to the range 0-253
			serial2_write_byte(sendDataByte4); 	//send second parameter: must be scaled to the range 0-253
			serial2_write_byte(0xFE);     	//send stop byte = 254
		}

		//if a new byte has been received
		if(new_message_received_flag)
		{

			OCR1A = 2000 - (dataByte3<<3); //setting speed of servo motor

			int16_t fc = 0;
			int16_t rc = 0;
			
			if(adc_read(4) == 0){
				LSADC = (0.6*LSADC) + (0.4*1); 
			}
			else{
				LSADC = (0.6*LSADC) + (0.4*adc_read(4)); //flattening the adc curve to remove outliers
			}
			
			if(adc_read(6) == 0){
				RSADC = (0.6*RSADC) + (0.4*1);
			}
			else{
				RSADC = (0.6*RSADC) + (0.4*adc_read(6)); //see above
			}
			
			if(adc_read(1) == 0){
				FSADC = (0.6*FSADC) + (0.4*1);
			}
			else{
				FSADC = (0.6*FSADC) + (0.4*adc_read(1)); //see above
			}
			

			LS = ((21543/LSADC)+26); //conversion of adc to millimeters
			RS = ((21543/RSADC)+26); //conversion of adc to millimeters
			FS = (6000/FSADC); //conversion of adc to centimeters

			if(dataByte4 == true){ //this means manual mode is set to on
				PORTC &= ~(1<<PC4); //turn off LED
				
				fc = (dataByte1<<1); //control movement using controllers joysticks
				rc = (dataByte2<<1);

			}
			
			else{
				PORTC |= (1<<PC4); //turn on LED
				
				if(FS>29) //if more than 29cm of clearance
				{
					fc = 25; //move at 96% of full speed forward
					rc = 126;
					if(LS>RS){ //if theres more clearance on the left sensor than the right sensor
						rc = 90; //turn left
					}
					else if(RS>LS){
						rc = 160; //turn right
					}
				}
				else if(FS<=29)
				{
					fc = 131; //keep robot still
					rc = 126;
					
					if(LS>RS){
						rc = 35; //make a sharp turn left
					}
					else if(RS>LS){
						rc = 205; //make a sharp turn right
					}
					
					
				}
				
			}
			
			rm = fc + rc - 253; //WE SWAPPED RM AND LM TO MAKE IT WORK FOR ROBOT
			lm = fc - rc;

			OCR3A = (int32_t)abs(lm)*10000/126; //lm speed from magnitude of lm
			OCR3B = (int32_t)abs(rm)*10000/126; //lm speed from magnitude of rm

			
			sprintf(serial_string, "ls: %3d, rs: %3d, fs: %2d, ls adc: %3u, rs adc: %3u, fs adc: %3u \n\r", LS, RS, FS, LSADC, RSADC, FSADC); //printing to serial0 for testing purposes
			serial0_print_string(serial_string);  // print the received bytes to the USB serial to make sure the right messages are received


			
			if(lm>=0) //if lm is positive
			{
				//set direction forwards
				PORTA |= (1<<PA0);
				PORTA &= ~(1<<PA1);
			}
			else
			{
				//set direction reverse
				PORTA &= ~(1<<PA0);
				PORTA |= (1<<PA1);
			}

			if(rm>=0) //if rm is positive
			{
				//set direction forwards
				PORTA |= (1<<PA2);
				PORTA &= ~(1<<PA3);
			}
			else
			{
				//set direction reverse
				PORTA &= ~(1<<PA2);
				PORTA |= (1<<PA3);
			}
			
			
			new_message_received_flag=false;	// set the flag back to false
		}
	}
	return(1);
} //end main


ISR(USART2_RX_vect)  // ISR executed whenever a new byte is available in the serial buffer
{
	static uint8_t recvByte1=0, recvByte2=0, recvByte3=0, recvByte4=0;    	// data bytes received
	static uint8_t serial_fsm_state=0;                                	// used in the serial receive ISR
	uint8_t	serial_byte_in = UDR2; //move serial byte into variable
	
	switch(serial_fsm_state) //switch by the current state
	{
		case 0:
		//do nothing, if check after switch case will find start byte and set serial_fsm_state to 1
		break;
		case 1: //waiting for first parameter
		recvByte1 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 2: //waiting for second parameter
		recvByte2 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 3: //waiting for second parameter
		recvByte3 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 4: //waiting for second parameter
		recvByte4 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 5: //waiting for stop byte
		if(serial_byte_in == 0xFE) //stop byte
		{
			// now that the stop byte has been received, set a flag so that the
			// main loop can execute the results of the message
			dataByte1 = recvByte1;
			dataByte2 = recvByte2;
			dataByte3 = recvByte3;
			dataByte4 = recvByte4;
			
			new_message_received_flag=true;
		}
		// if the stop byte is not received, there is an error, so no commands are implemented
		serial_fsm_state = 0; //do nothing next time except check for start byte (below)
		break;
	}
	if(serial_byte_in == 0xFF) //if start byte is received, we go back to expecting the first data byte
	{
		serial_fsm_state=1;
	}
}
