/*	+++LIST OF BUILT-IN FUNCTION:
	_delay_ms(): to delay in ms
	
	+++I/O pin in AVR
	(x can be replaced by A,B,C,D as per the AVR you are using)
		– DDRx register
		– PORTx register
		– PINx register
	DDRx: configure the direction of port - which port pins will be used for input/output
		  writing 0 to a bit in DDRx makes the corresponding port pin as input, while writing
		  1 to a bit in DDRx makes the corresponding pin as output
		  + Make all pins of port A as input: DDRA = 0b00000000;
		  + Make all pins of port A as output: DDRA = 0b11111111;

	+++HOW TO USE PORTB
	Set port b as input/output: DDRB |= 0x01;
 	After trigger port b:
 		 	- PORTB:	assign value to port b
			- PINB:		read value from port b
	
	Example:
	if (PINB & (1 << PINB0)){
        A = true;
    }
	-> PINB = 1: read the pin successfully
	-> (1 << PINB0) makes 0th bit 1
	-> The entire if function check whether the pin 0 in port b is high or low			
*/

/*
 * uart.c
 *
 * Asynchronous UART example tested on ATMega328P (16 MHz)
 *
 * Toolchain: avr-gcc (4.3.3)
 * Editor: Eclipse Kepler (4)
 * Usage:
 * 		Perform all settings in uart.h and enable by calling initUART(void)
 * 		Compile:
 * 				make all
 *
 * Functions:
 * 		- First call initUART() to set up Baud rate and frame format
 *		- initUART() calls macros TX_START() and RX_START() automatically
 *		- To enable interrupts on reception, call RX_INTEN() macros
 *		- Call functions getByte() and putByte(char) for character I/O
 *		- Call functions writeString(char*) and readString() for string I/O
 *
 *  Created on: 21-Jan-2014
 *      Author: Shrikant Giridhar
 */

#define F_CPU 16000000UL  //telling controller crystal frequency attached 16 MHz
#include <avr/io.h>		  //header to enable data flow control over pins
#include <util/delay.h>   //header to enable delay function in program
#include "usart.h"
#include <avr/interrupt.h>

volatile unsigned char receivedByte;
volatile unsigned char trickortreat;

void initUART(void)
{
	// Set baud rate; lower byte and top nibble
	UBRR0H = ((_UBRR) & 0x000);
	UBRR0L = (uint8_t) ((_UBRR) & 0x6E);

	TX_START();			// Enable the receiver
	RX_START();			// Enable the transmitter

	// Set frame format = 8-N-1
	UCSR0C = (_DATA << UCSZ00);
}

uint8_t getByte(void)				// for character I/O
{
	// Check to see if something was received
	while (!(UCSR0A & _BV(RXC0)));
	return (uint8_t) UDR0;
}

void putByte(char data)	// for character I/O
{
	// Stay here until data buffer is empty
	while (!(UCSR0A & _BV(UDRE0)));
	UDR0 = (unsigned char) data;
}

void writeString(char *str)			// for string I/O
{
	while (*str != '\0')
	{
		putByte(*str);
		++str;
	}
}

char * readString(void)				// for string I/O
{
	static char rxstr[RX_BUFF];
	static char* temp;
	temp = rxstr;

	while((*temp = getByte()) != '\r')
	{
		++temp;
	}

	return rxstr;
}

void Wait(int msec)
{
	int k;
	for (k=0;k<msec;k++)
	{
 		_delay_ms (1);
 	}
}



/*void wait_for_button()
{
	while(PIND & (1<<DDD7)); //button not pressed
	_delay_ms(20);          //button initially pressed

	while(!(PIND & (1<<DDD7)))  
	
	trickortreat = 1;
	return trickortreat;
}
*/


/******************************PuTTY  Testing*******************************/
// Not used except when testing
void PrintNumber(int N, int Base, int digits)
{ 
	char HexDigit[]="0123456789ABCDEF";
	int j;
	#define NBITS sizeof(int)
	char buff[NBITS+1];
	buff[NBITS]=0;

	j=NBITS-1;
	while ( (N>0) | (digits>0) )
	{
		buff[j--]=HexDigit[N%Base];
		N/=Base;
		if(digits!=0) digits--;
	}
	usart_pstr(&buff[j+1]);
}

void usart_pstr(char *s)
{
	while (*s)
	{
		putByte(*s); // loop through entire string
		s++;
	}
}


/************************Accelerometer Tracking****************************/

// Reads value fed into ADC Channel
unsigned int ReadChannel(char mux)
{
	ADCSRA = (1<<ADEN) | (1<<ADPS1) | (1<<ADPS0); // frequency prescaler
	ADMUX = mux; // channel select
	ADMUX |= (1<<REFS1) | (1<<REFS0); 
	ADCSRA |= (1<<ADSC); // Start conversion
	while ( ADCSRA & (1<<ADSC) ) ;
	ADCSRA |= (1<<ADSC); // a transformation single conversion
	while ( ADCSRA & (1<<ADSC) );
	ADCSRA &= ~(1<<ADEN); 
	// Disable ADC
	return ADCW;
}

/*************************************************************************/

int main(void)
{
	
	initUART();
	
	// Variable Declarations For Accelerometer
	unsigned int buffer_x;
	char mode = 'p';
	unsigned int buffer_y;
	unsigned int position_x;
	unsigned int position_y;
	int offset_x = 177; 
	int offset_y = 88; 
	

	PORTD |= (1 << PD2); 	// read INPUT at port D, pin D2 - the button
	PORTD |= (1 << PD3); 	// read INPUT at port D, pin D2 - the button
	PORTD |= (1 << PD4); 	// read INPUT at port D, pin D2 - the button
	PORTC |= (1 << PC5); 	// read INPUT at port D, pin D2 - the button
	PORTC |= (1 << PC4); 	// read INPUT at port D, pin D2 - the button
	PORTC |= (1 << PC3); 	// read INPUT at port D, pin D2 - the button

	
	DDRB |= (1 << PB0); 	//Data Direction Register B: writing a 1 to the bit enables output
	
	DDRD |= (1 << DDD6);
	// PD6 is now an output
		
	OCR0A = 62;
	// adjust this to get the correct frequency
			
	TCCR0A |= (1 << COM0A0);
	// set normal port operation, OC0A disconnected
			
	TCCR0A |= (0 << WGM02) | (1 << WGM01) | (0 << WGM00);
	// set CTC Mode
		
	TCCR0B |= (1 << CS01);
	// set prescaler to 8 and starts PWM


	PORTB = 0b00000000;
	
	/*************************************************************************/
	// Checks which button was pressed for the 2 control modes: 
	// Push buttons, or
	// Motion tracking.
	
	// Note: So I guess it's gonna stay in whichever mode until reset?
	
	/*if(P2_2) // We can change these buttons 
			mode = Push;
		else if (P2_1)
			mode = Accel;
	/*************************************************************************/

	while(1)
	{	
		// Control Mode: Push Buttons
		while(mode == 'p'){
			
			if (bit_is_clear( PIND, PD2)) 
			{ 
					DDRD &= ~(1 << DDD6);

					PORTB = 0b00000001;
					
					_delay_ms(5);
					
					DDRD |= (1 << DDD6);
					
					
			} 
			else if (bit_is_clear( PIND, PD3)) 
			{ 
					DDRD &= ~(1 << DDD6);

					PORTB = 0b00000001;
					
					_delay_ms(800);
									
					DDRD |= (1 << DDD6);
			} 
			else if (bit_is_clear( PIND, PD4)) 
			{ 
					DDRD &= ~(1 << DDD6);

					PORTB = 0b00000001;
					
					_delay_ms(1800);
									
				DDRD	 |= (1 << DDD6);
			} 
			else if (bit_is_clear( PINC, PC4)) 
			{ 
					DDRD &= ~(1 << DDD6);

					PORTB = 0b00000001;
					
					_delay_ms (2800);
					
					DDRD |= (1 << DDD6);
			} 
			else if (bit_is_clear( PINC, PC5)) 
			{ 
					DDRD &= ~(1 << DDD6);

					PORTB = 0b00000001;
					
					_delay_ms (3700);
					
					DDRD |= (1 << DDD6);
			} 
			else if (bit_is_clear( PINC, PC3)) 
			{
					DDRD |= (1 << DDD6);
					PORTB = 0b00000001;
			}
		//	else 
		//	{
		//	}
		}
		
		/************************************Accelerometer*************************************/
		// Control Mode: Accelerometer
		while(mode == 'a'){	
			
			// Gets Accelerometer Data from ADC Channels 0 and 1
			buffer_x = ReadChannel(0);  
			buffer_y = ReadChannel(1);
			
			// Accelerometer Position Calculations
			position_x = 2 * buffer_x - offset_x;		// x2 b/c ADC doesn't read negative values
			position_y = 2 * buffer_y - offset_y;		
			
			
			//We can use this to test/calibrate Accelerometer values, but is not needed when demo-ing.
			
			
			// PuTTY Test:
			if(position_x < 560 ){
				usart_pstr("x-coordinates: -");
				PrintNumber( position_x, 10, 3);
			}
			else {
				usart_pstr("x-coordinates: ");
				PrintNumber(position_x, 10, 3);
			}
			
			if(position_y < 266 ){
				usart_pstr(", y-coordinates: -");
				PrintNumber(position_y, 10, 3);
				usart_pstr("   \r");
			}
			else{
				usart_pstr(", y-coordinates: ");
				PrintNumber(position_y, 10, 3);
				usart_pstr("   \r");
			}
			
			//_delay_ms(500);
			
			
			if( position_x > 570  ) 	// (526)
			{ 
					DDRD &= ~(1 << DDD6);

					PORTB = 0b00000001;
					
					_delay_ms (3700);
					
					DDRD |= (1 << DDD6);
			} 
				
			else if( position_x < 555  ) 	// (526)
			{ 
					DDRD &= ~(1 << DDD6);

					PORTB = 0b00000001;
					
					_delay_ms (2800);
					
					DDRD |= (1 << DDD6);
			} 	
				
			if( position_y > 300 ) 		// (266)
			{
					DDRD &= ~(1 << DDD6);

					PORTB = 0b00000001;
					
					_delay_ms(800);
									
					DDRD |= (1 << DDD6);
			}
				
			else if( position_y < 290  ) 	// (266)
			{ 
					DDRD &= ~(1 << DDD6);

					PORTB = 0b00000001;
					
					_delay_ms(5);
					
					DDRD |= (1 << DDD6);
					
					
			} 

			else 
			{
					DDRD |= (1 << DDD6);
					PORTB = 0b00000001;
			}		
		} 
	/*************************************************************************/
	
	}
	
	return 0;
}
