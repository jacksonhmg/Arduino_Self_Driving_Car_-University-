//include this .c file's header file
#include "Controller.h"

//static function prototypes, functions only called in this file


//file scope variables
static char output_string[20] = {0};
volatile uint8_t dataByte1=0, dataByte2=0, dataByte3=0, dataByte4=0;    	// data bytes received
volatile bool new_message_received_flag=false;
volatile bool buttonClick = true; //for changing robot control between manual and auto
volatile bool buttonClick2 = true; //for changing servo control between manual and auto

int main(void)
{
	// initialisation
	adc_init();
	lcd_init();
    
	serial0_init(); 	// terminal communication with PC
	serial2_init();    	// microcontroller communication to/from another Arduino
	// or loopback communication to same Arduino
    
	DDRD = 0; //for the joystick button clicks
	PORTD = 0xFF;
    
	uint8_t sendDataByte1=0, sendDataByte2=0, sendDataByte3=0, sendDataByte4=0;    	// data bytes sent
    
	uint32_t current_ms=0, last_send_ms=0;        	// used for timing the serial send
    
	UCSR2B |= (1 << RXCIE2); // Enable the USART Receive Complete interrupt (USART_RXC)
    
	milliseconds_init();
	sei();
    
	uint8_t counter = 0; //for auto servo control
    
	bool counterswapper = true; //for auto servo control
    
    
	while(1)
	{
    	current_ms = milliseconds;
    	//sending section
    	if(current_ms-last_send_ms >= 100) //sending rate controlled here one message every 100ms (10Hz)
    	{
       	 
        	if (!(PIND & (1<<PD0)))
        	{
            	// PORTC = 0;
            	buttonClick = !buttonClick; //swaps between manual and auto robot
            	_delay_ms(200);
        	}   
			if (!(PIND & (1<<PD1)))
			{
				buttonClick2 = !buttonClick2; //swaps between manual and auto servo
				_delay_ms(200);
			}
    	 if(buttonClick2==true)
    	 {
	    	 if(counterswapper == true){
		    	 counter ++;
	    	 }
	    	 
	    	 if(counterswapper == false){
		    	 counter --;
	    	 }
	    	 if((counter <= 15 )){ //stays here for 1.5 seconds
		    	 sendDataByte3 = (1023)>>3; //maximum right turn of servo
	    	 }
	    	 if((counter > 15) && (counter <= 30)){ //stays here for 1.5 seconds
		    	 sendDataByte3 = (767)>>3; //halfway between max right and centre positions
	    	 }
	    	 if((counter > 30) && (counter <= 45)){ //1.5 seconds
		    	 sendDataByte3 = (511)>>3; //centre position
	    	 }
	    	 if((counter > 45) && (counter <= 60)){ //1.5 seconds
		    	 sendDataByte3 = (255)>>3; //halfway between centre and max left 
	    	 }
	    	 if((counter > 60)){ //see below for how this stays for 1.4 seconds
		    	 sendDataByte3 = (0)>>3; //maximum left position
	    	 }
	    	 if(counter > 67){ //CHOSE 7 BECAUSE ITS HALF OF 15. WILL INCREMEMENT UP TO 67 AT MAX LEFT POSITION AND WILL DECREMENT DOWN TO 60 WHILST STILL BEING IN MAX LEFT POSITION. 1.4 OVERALL SECONDS IN MAX LEFT POSITION
		    	 counterswapper = false; //swaps it so now counter decrements (see above)
	    	 }
	    	 if(counter <= 0){
		    	 counterswapper = true; //swaps it so now counter increments (see above)
	    	 }
    	 }
    	 else
    	 {
	    	 sendDataByte3 = (adc_read(14))>>3; //horizontal movement of left joystick
    	 }

        	sendDataByte1 = adc_read(0)>>3; //send vertical movement of right joystick shifted 3 bits
        	sendDataByte2 = adc_read(1)>>3; //horizontal movement of right joystick
       	 
        	sendDataByte4 = buttonClick; //boolean sent to robot so it knows whether to be in auto or manual
       	 
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

        	uint16_t LS = dataByte1<<3;
   			uint16_t RS = dataByte3<<3;
			uint16_t FS = dataByte2<<3;
       	 
       	 
        	lcd_home(); //goes to first line of lcd
        	sprintf(output_string, "L:%.3dmm R:%.3dmm ", LS, RS); //puts this into output_string
        	lcd_puts(output_string); //prints the output_string to the lcd (left sensor and right sensor values)
       	 
        	serial0_print_string(output_string); //sent to serial0 for debugging purposes using putty.exe
        	serial0_print_string("\n");
       	 
       	 
        	lcd_goto(0x40); //goes to second line of lcd screen
        	sprintf(output_string, "F:%.2d cm V:%.2dV ", FS, (dataByte4<<3)); //print front sensor and voltage values
        	lcd_puts(output_string);
       	 
        	serial0_print_string(output_string);
        	serial0_print_string("\n");
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
