// lab3.c 
// Benson Ton
// 10.26.21

//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
//  PORTB bit 7 goes to the PWM transistor base.

//#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0
#define BUTTON_ONE 1
#define BUTTON_ZERO 0

#define NEUTRAL 0x11
#define HALFWAY 0X00
#define LEFTP 0x01
#define RIGHTP 0x10

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>

//GLOBAL VARIABLES
int count = 0;

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5] = {0xC0}; 

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[12]={0xC0,0xF9,0xA4 ,0xB0 ,0x99 ,0x92 ,0x82,0xF8 ,0x80 ,0x98,0xFF,0x07 };//numbers from 0-9, all blank, blank colon


uint8_t raw_encoder_val = 0;

enum states {position0, position1, position2, position3 };

// TIMER0 overflow
ISR( TIMER0_OVF_vect ) {
//might have to save portb in the beginning and restore it at the end to prevent flickering 


}


//Processing the data for the encoders
// taking in data from the miso pin
// This only takes for one of the encoders
int8_t encoder_data(uint8_t en_var){
static uint16_t state = {0}; //holds the current state of the encoder value in bits

uint8_t a_pin, b_pin;


//encoders are active low; a_pin and b_pin assigned a new value if they are triggered
a_pin = (( en_var & 0x01 ) == 0) ? 0 : 1;
b_pin = ((en_var & 0x02 ) == 0) ? 0 : 1;

// update the shift by looking at the a pin
state = (state << 1) | a_pin | 0xE0;


//return 1 for cw, return 0 for ccw
//comparing the result of a_pin to determine the direction of the rotation
	if(state == 0xF0){	
		return (b_pin) ? 1 : 0;
	}

	else{return -1;} // no movement has been made

}


//*******************************************************************************
//                            pressed_bit_0                                  
// Checks the state of pushbutton S0 It shifts in ones till the button is pushed. 
// Function returns a 1 only once per debounced button push so a debounce and toggle 
// function can be implemented at the same time. Expects active low pushbutton on 
// Port A  bit zero. Debounce time is determined by external loop delay times 12. 
//*******************************************************************************
uint8_t pressed_bit_0() {
  static uint16_t state = 0; //holds present state
  state = (state << 1) | (! bit_is_clear(PINA, 0)) | 0xE000;
  if (state == 0xF000){ 
	  return TRUE;
  }
  return FALSE;
}

//**
uint8_t pressed_bit_1() {
 static uint16_t state = 0; //holds present state
 state = (state << 1) | (! bit_is_clear(PINA, 1)) | 0xE000;
  if (state == 0xF000){ 
	return TRUE;
 }
 return FALSE;
} 

//Note: BUTTON_ONE = 1 & BUTTON_ZERO = 0
//check if either bit 1 or bit 0 is pressed
uint8_t check_buttons(){

if(pressed_bit_0()){return BUTTON_ZERO;}

if(pressed_bit_1()){return BUTTON_ONE;}


}


//***********************************************************************************
// 					spi_init
// Initialize the SPI port on the mega128. 
//
//***********************************************************************************
void spi_init(void){

SPCR |= ( (1 << MSTR) | (1 << SPE) ); //master mode, SPI enabled, clk low on idle, leading edge sample
//SPSR = (1 << SPI2X);
}

//***********************************************************************************
//                                   segment_sum                                    
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit 
//BCD segment code in the array segment_data for display.                       
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
//**********************************************************************************
void segsum(uint16_t sum) {

//initialzing the local variables of the function
int digit_index = 0;
int temp = 0;
int i;

if(sum == 0){segment_data[digit_index] = dec_to_7seg[0];};

	while(sum > 0){
		temp = sum % 10; // isolate to a single digit
	        if(digit_index == 2){digit_index++;};
		segment_data[digit_index] = dec_to_7seg[temp]; //use the digit to store the hexidecimal value to seg array
		sum = sum/10; // continue to the next digit
		digit_index++; // increase the digit index
	}
	segment_data[2] = dec_to_7seg[11]; //replace the colon with a blank colon
	
//need to turn off the rest of the digits
	for(i = 5; i > digit_index; i--){
       		segment_data[i] = dec_to_7seg[10];	       
	}

}
//segment_sum

//***********************************************************************************


//*************
//
void bar_send_out(uint8_t data){


}


//****************
//
//takes switch on LED for bit_0 being pressed
void bar_bit_0(){
  
	int display_one = 1;
	PORTD |= (1 << PD2); 

}


//once the state machine completes all 4 cycles then it means the knob has been shifted once


  //clearSegment();

}


//***********************************************************************************
uint8_t main()
{
//set port bits 4-7 B as outputs
DDRB |= 0xF0; // set PORTB 4-7 bits as outputs
DDRA = 0x03; // set PORTA 6-7 bits as inputs
DDRD = (1 << PD2); // set PD2 as output
DDRB |= ( (1 << PB2) | ( 1<< PB1) | (1<<PB0) ); //mosi and serial clock
DDRE = 0xFF;
//initializing SPI on mega128
SPCR |= ( (1 << MSTR) | (1 << SPE) ); //master mode, SPI enabled, clk low on idle, leading edge sample

//SPSR = (1 << SPI2X);
//initializing timer/counter0 (TCNT0) 
TIMSK |= (1<<TOIE0);	//enable interrupts
TCCR0 |= (1 <<CS02) | (1<<CS00); //normal mode, prescale by 128

//initialzing the count for led display
//int count = 0;
int i;

uint8_t junk_data = 0x21; //to start input from the SPI
//uint8_t raw_encoder; //raw encoder value

//start at position 1
enum states state = position0;//default state
int test = 0;
int inc = 0;
int dec = 0;




/*
PORTE &= ~(1 << PE6);
PORTE |= (1 << PE6);

SPDR = junk_data;

while(bit_is_clear(SPSR,SPIF)){} // read data in

raw_encoder_val = SPDR;//save the data


//process the raw data for the first encoder
uint8_t old_a_pin_LE, old_b_pin_LE;
uint8_t a_pin_LE, b_pin_LE;

//saving current positions to compare
old_a_pin_LE = a_pin_LE; 
old_b_pin_LE = b_pin_LE;
*/



while(1){
//insert loop delay for debounc_delay_ms(5);
//_delay_ms(1);

PORTA = 0xFF;  //make PORTA an input port with pullups 
//PORTB = 0x50;  //enable tristate buffer for pushbutton switches

_delay_ms(.1); 
//******************** NEW CODE

PORTE &= ~(1 << PE6);
PORTE |= (1 << PE6);

SPDR = junk_data;

while(bit_is_clear(SPSR,SPIF)){} // read data in

raw_encoder_val = SPDR;//save the data


//process the raw data for the first encoder
uint8_t old_a_pin_LE, old_b_pin_LE;
uint8_t a_pin_LE, b_pin_LE;

//saving current positions to compare
old_a_pin_LE = a_pin_LE; 
old_b_pin_LE = b_pin_LE;


//current
//Breaking up the raw data from the encoder into a and b pins
a_pin_LE = ((raw_encoder_val & 0x01) ==0) ? 0:1;
b_pin_LE = ((raw_encoder_val & 0x02) ==0) ? 0:1;

//int sum = 0;

//state machine
switch(state){
	//when A and B are high (11)
     	case position0:{
//	if(inc > 4){inc = 1;}
		 if( (a_pin_LE != 1) || (b_pin_LE != 1) ){ // if no buttons are pressed then stay in this position 
			if( (a_pin_LE == 1) && (b_pin_LE == 0) ) { //if A goes high and B goes low then go to position1
		//		inc++;
				inc = 0; // reset increment when the knob goes CCW
				dec++; // increase decrement when knob goes CCW
				state = position1;
				break;
			}	
			else if( (a_pin_LE == 0) && (b_pin_LE == 1) ){ 
			//	if(inc == 4){inc =0;}
				inc++;
				dec = 0;
				state = position2;
				break;
			}
		}
		else { 	
			state = position0;
			break;}
		      		       
	}
	//when A is high and B is low
	case position1:{
		if( (a_pin_LE == 1) && ( b_pin_LE == 1) ){
			dec = 0;
			inc = 0;
	//		inc++;
	//	   	sum++;
			state = position0;
		   	break;
		}
		else if( (a_pin_LE == 0) && (b_pin_LE ==0) ){
	//	  	inc++;
			inc = 2; //cant go backwards in middle of rotation
			dec++;
			state = position3;
		  	break;
		}	     			
	}
	//When A is low and B is high
	case position2:{
		if( (a_pin_LE == 1) && (b_pin_LE == 1) ){
	//		sum++;
			inc = 0; //might have to do nothing maybe
			dec++;
			state = position0;
			break;
		}
		else if( (a_pin_LE == 0) && (b_pin_LE == 0) ){
			inc++;
			dec = 0;
			state = position3;
			break;
		}
	}
	//when both pins are low
	case position3:{
		if( (a_pin_LE == 1) && (b_pin_LE == 0)){
			inc++;
		//	state = position1;
			dec = 1;
			if(inc >= 2){
				state = position1;
//				sum++;
				break;
			}
			else{
				inc = 0;
				state = position0;
				break;
			}
		}
		else if( (a_pin_LE == 0) && (b_pin_LE == 1 ) ){
			dec++;
			inc = 1; //changed
		//	inc++;
			if(dec >= 2){
			//	sum++;
				state = position2; 
				break;
			}
			else{
			//	inc = 0;
				dec = 0;
				state = position0;
				break;
			}
		}	
	}	       

}



/*
//int test = 0;
if(sum == 4) {
	sum = 0;	
	test = 1;
	}

SPDR = test;
_delay_ms(1);


while (bit_is_clear(SPSR,SPIF)) {} //wait till data is sent out
PORTD |= (1 << PD2); //SEND data to bargraph
PORTD &= ~(1<<PD2);

*/
_delay_ms(1);


if(inc == 4){
//sum= 0;
	count++;
//	inc = 0;
}
_delay_ms(1);
if(dec == 4){
count--;

}
//if(dec == 4){
//	count--;
//}
//

//*********END OF NEW CODE



//send to SPI and output an LED to turn on that a button is pressed
// change the encoder to increase by two
	
	
//disable tristate buffer for pushbutton switches
 PORTB = 0x70;

  //bound the count to 0 - 1023
  if (count > 1023){
	  count = 0;
     }
// bound the count 1023 - 0
if( count < 0){
	count = 1023;

}
//count = 100;
//    for(i=0; i<=4; i++){_delay_ms(5);}         //0.5 sec delay

//count = 100;
  //break up the disp_value to 4, BCD digits in the array: call (segsum)
 segsum(count); // send the current count to LED Display



 //DIGIT SHIFTS
 //bound a counter (0-4) to keep track of digit to display ;
PORTB = 0x00;
 // put it in a for loop
int index;
//PORTA = 0XFF;


for(index = 0; index < 5; index++){
	PORTB = index << 4; // within 4 digits for the hex value
	PORTA = segment_data[index]; //send 7 segment code to LED segments
	if( (PORTB == 0x40)&&(count< 1000) ){
	segment_data[5] = dec_to_7seg[10];
	PORTA = segment_data[5];}

	_delay_ms(.1); // add delay
}




//make PORTA an output
DDRA = 0xFF;  






}//while
}//main

