// lab4.c 
// Benson Ton
// 11.7.21

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

#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>

#include "hd44780.h"
//#include "ADC.h"

//GLOBAL VARIABLES

div_t    fp_adc_result, fp_low_result;  //double fp_adc_result; 


unsigned char 	new_data = FALSE;
uint16_t 	adc_data; //holds the adc raw values

char     	lcd_str_h[16];  //holds string to send to lcd  
char     	lcd_str_l[16];  //holds string to send to lcd  

volatile int 	hours = 0;
volatile int 	minutes = 0;
volatile int 	seconds = 0;
volatile int 	count = 0; //LED display count
volatile int 	increment = 0; // count increment status
volatile uint8_t mode = 0; // mode select state

volatile uint8_t raw_encoder_val;// raw data for the left encoder
volatile uint8_t raw_encoder_valR; // raw data for the right encoder

volatile int read = 0; // status of whether the encoder turned CW or CCW

int state_counter = 0; // status of the pins being contacted of the left encoder

int r_counter = 0; // status of the pins being contacted of the right encoder

//left encoder process raw values
volatile int 	old_A;
volatile int 	old_B;
//right encoder process raw values
volatile int 	old_C;
volatile int 	old_D;

//save the status of the ports
volatile uint8_t saveB;
volatile uint8_t saveA;

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5] = {0xC0}; 

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[12]={0xC0,0xF9,0xA4 ,0xB0 ,0x99 ,0x92 ,0x82,0xF8 ,0x80 ,0x98,0xFF,0x07 };//numbers from 0-9, all blank, blank colon

/*
// ADC for dimming 
ISR(ADC_vect){
saveA = PORTA;
saveB = PORTB;


//clear_display();
adc_data = ADC; // reading data 
itoa(adc_data,lcd_str_h,10); // converts into decimal
//string2lcd(lcd_str_h);
//clear_display();
string2lcd(lcd_str_h);

// for(int i=0;i<=10;i++){_delay_ms(20);}  //delay 0.5 sec
// clear_display();


PORTA = saveA;
PORTB = saveB;


}
//End of ADC ISR
*/

//dimming for LED display 
ISR( TIMER2_COMP_vect){
if(adc_data > 70){OCR2 = 10;}
else{OCR2 = 200;}

}

// TIMER0 overflow
ISR( TIMER0_OVF_vect ) {

 // static uint8_t timer=0;  //hold value of count between interrupts
 // timer++;                 //extend counter
  //toggle PB0 each time this happens
//  if((timer % 128) == 0){count++;} // timer

/*
//save the states of PORTA and PORTB
saveA = PORTA;
saveB = PORTB;

PORTB = 0x50;  //enable tristate buffer for pushbutton switches
_delay_ms(.1);

	if( (pressed_bit_0() == TRUE) ){
		mode ^= ( 1 << 3);//toggle mode and LED to light up on bar graph
	}//select the mode or toggle spi LED

	else if( (pressed_bit_1() == TRUE) ){
		mode ^= ( 1 << 6); // toggle mode and LED to light up on bar graph
	}

PORTB = 0x70;

increment = 1; //if no buttons are pressed then increment by one

	if(mode == (0x48)){increment = 0;} //if both buttons are pressed then dont increment
	if(mode == ((0x08))){increment = 2;} // if right one is pressed then increment by 2
	if(mode == (0x40)){increment = 4;} // if left one is pressed then increment by 4


write_SPI(mode);//write to bar graph

read_SPI();//read in from the SPI
read = process_EN(); // decrypt the data from the SPI and determine the encoder movement
update_EN(read);// increase the count regarding the modes


//restore the state when leaving the ISR
PORTA = saveA;
PORTB = saveB;

DDRA = 0xFF; //set PORTA to all outputs
*/
}//end of ISR Timer0


/********************************************************************
 *				update_encoder
 */

void update_EN(int val_rot){

	if(val_rot == 1){ count += increment;} //if rotating to the right for left encoder then increment 
	else if(val_rot ==0){count-= increment;}//if rotating to the left for left encoder then decrement
	else if(val_rot == 2){count-= increment;} // if rotating to the left for right encoder then decrement
	else if(val_rot == 3){count+= increment;}// if rotatiing to the right for right encoder then increment
	else{val_rot =-1;} // dont do anything if nothing is rotated

}





/********************************************************************
 *				read_SPI
 */
void read_SPI(void){
//shift clock register

PORTE &= ~(1 << PE6);//falling edge
PORTE |= (1 << PE6); //rising edge

SPDR = 0x20; //send junk data to read in from SPI

while(bit_is_clear(SPSR,SPIF)){} // read data in

raw_encoder_val = SPDR;//save the data


}
/********************************************************************
 *				write_SPI
 */
void write_SPI(uint8_t value){
SPDR = value; // take in which mode it is currently on and display it 
while (bit_is_clear(SPSR,SPIF)) {} //wait till data is sent out
PORTD |= (1 << PD2); //SEND data to bargraph, rising edge
PORTD &= ~(1<<PD2); // falling edge

}



//*******************************************************************************
//                            pressed_bit_0                                  
// Checks the state of pushbutton S0 It shifts in ones till the button is pushed. 
// Function returns a 1 only once per debounced button push so a debounce and toggle 
// function can be implemented at the same time. Expects active low pushbutton on 
// Port A  bit zero. Debounce time is determined by external loop delay times 12. 
//*******************************************************************************
int pressed_bit_0() {
  static uint16_t state = 0; //holds present state
  state = (state << 1) | (! bit_is_clear(PINA, 0)) | 0xE000;
  if (state == 0xF000){ //returns true if button is pressed
	  return TRUE;
  }
  return FALSE;
}
//*******************************************************************************
//                            pressed_bit_1                                  
// Checks the state of pushbutton S0 It shifts in ones till the button is pushed. 
// Function returns a 1 only once per debounced button push so a debounce and toggle 
// function can be implemented at the same time. Expects active low pushbutton on 
// Port A  bit zero. Debounce time is determined by external loop delay times 12. 
//*******************************************************************************
int pressed_bit_1() {
 static uint16_t state = 0; //holds present state
 state = (state << 1) | (! bit_is_clear(PINA, 1)) | 0xE000;
  if (state == 0xF000){ 
	return TRUE; //returns true if it is pressed
 }
 return FALSE;
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

//handle the case if it is 0 and increase the digit index
if(sum == 0){segment_data[digit_index] = dec_to_7seg[0];
	digit_index++; 
	}

	while(sum != 0){
		temp = sum % 10; // isolate to a single digit
	        if(digit_index == 2){digit_index++;};
		segment_data[digit_index] = dec_to_7seg[temp]; //use the digit to store the hexidecimal value to seg array
		sum = sum/10; // continue to the next digit
		digit_index++; // increase the digit index
	}
	segment_data[2] = dec_to_7seg[11]; //replace the colon with a blank colon
	
//need to turn off the rest of the digits
	 if (digit_index < 5){ //if there are less digits than segment numbers
        for (i = digit_index; i < 5; i++)
        {
                segment_data[i] = dec_to_7seg[10]; //blank them out
        }
  }
}
//segment_sum

//***********************************************************************************





/**********************************************************************
 *				process_EN
 */
//once the state machine completes all 4 cycles then it means the knob has been shifted once
//encoder function for the left and right knob
int process_EN(void){
//Breaking up the raw data from the encoder into a and b pins
uint8_t new_A = ((raw_encoder_val & 0x01) ==0) ? 0:1;
uint8_t new_B = ((raw_encoder_val & 0x02) ==0) ? 0:1;

//process the right encoder
uint8_t new_C = ((raw_encoder_val & 0x04) ==0) ? 0:1;
uint8_t new_D = ((raw_encoder_val & 0x08) ==0) ? 0:1;

int en_val = 0;
//old_A = new_A;
//old_B = new_B;

en_val = -1; // default return value , no change

// process the right encoder inputs and determine the position movement
if (( new_C != old_C ) || ( new_D != old_D )){ // if change occurred

if (( new_C == 0) && ( new_D == 0)) { // once the state has been moved either right or left from current state
	if ( old_C == 1){ r_counter++;}
	else { r_counter--;}
}
else if (( new_C == 0) && ( new_D == 1)) { // update state if it has been moved from position from left to right 
	if ( old_C == 0){ r_counter++;}
	else { r_counter--;}
}

else if (( new_C == 1) && ( new_D == 1)) {// detent position
	
	if ( old_C == 0){ if( r_counter == 3){ en_val =2;}} // one direction. Increment 

	else {if( r_counter == -3){ en_val =3;}} // or the other. Decrement
	
	r_counter = 0; // count is always reset in detent position
}
else if (( new_C == 1) && ( new_D == 0)) {
	if ( old_C == 1) { r_counter++;}
	else { r_counter--;}
}
	old_C = new_C ; // save what are now old values
	old_D = new_D ;

	} // if change occurred


// process the left encoder inputs and determine the position movement
if (( new_A != old_A ) || ( new_B != old_B )){ // if change occurred

if (( new_A == 0) && ( new_B == 0)) { // once the state has been moved either right or left from current state
	if ( old_A == 1){ state_counter++;}
	else { state_counter--;}
}
else if (( new_A == 0) && ( new_B == 1)) { // update state if it has been moved from position from left to right 
	if ( old_A == 0){ state_counter++;}
	else { state_counter--;}
}

else if (( new_A == 1) && ( new_B == 1)) {// detent position
	
	if ( old_A == 0){ if( state_counter == 3){ en_val =0;}} // one direction. Increment 

	else {if( state_counter == -3){ en_val =1;}} // or the other. Decrement
	
	state_counter = 0; // count is always reset in detent position
}
else if (( new_A == 1) && ( new_B == 0)) {
	if ( old_A == 1) { state_counter++;}
	else { state_counter--;}
}
	old_A = new_A ; // save what are now old values
	old_B = new_B ;

	} // if change occurred


return ( en_val ); // return encoder state

}

void process_ADC_data(void){
new_data = FALSE;
}

void spi_init(void){
DDRB |= ( (1 << PB2) | ( 1<< PB1) | (1<<PB0) ); //mosi and serial clock
SPCR |= ( (1 << MSTR) | (1 << SPE) ); //master mode, SPI enabled, clk low on idle, leading edge sample


DDRB |= 0xF0; // set PORTB 4-7 bits as outputs
DDRA = 0x03; // set PORTA 6-7 bits as inputs
DDRD = (1 << PD2); // set PD2 as output
DDRE = 0xFF;// set PORTE to all inputs


}


void ADC_init(void){

	ADMUX = (1 << REFS0) | (1 << ADLAR) | 0x07; // reference to AVCC(5v), left adjusted (only reading  8bit), and channel 7

	//ADC enable, Single-conversion, clk/128
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | ( 1 << ADPS1) | ( 1 << ADPS0) ; // still need to start it
}


//***********************************************************************************
int main()
{

spi_init();// initialize SPI and port configurations
lcd_init();
clear_display();
//uint2lcd(8);
//char2lcd('h');

//ADC
DDRF  &= ~(_BV(DDF7)); //make port F bit 7 the ADC input  
PORTF &= ~(_BV(PF7));  //port F bit 7 pullups must be off
ADC_init(); //initialize ADC

//initialiing timer/counter2 (TCNT2) for the dimming pwm
// 
TCCR2 |= (1 << WGM21) | ( 1 << WGM20) | (1 << COM21) | ( 1 << CS20); // no prescale, clear on OC2 match, fast pwm mode
TIMSK |= (1 << OCIE2); // Compare match interrupt enable timer/counter0
//DDRB |= (1 << DDB7); //PB7 as PWM output
TCNT2 = 0;


//initializing timer/counter0 (TCNT0) 
ASSR   |=  1<<AS0; //ext osc TOSC in 32kHz 
TIMSK  |=  1<<TOIE0; //enable TCNT0 overflow interrupt
TCCR0  |=  1<<CS00; //normal mode, no prescale



while(1){ // main loop

	ADCSR |= (1<<ADSC); //start writing 
	while(bit_is_clear(ADCSRA, ADIF)){};

ADCSR |= (1 << ADIF);//clear flag by writing one
adc_data = ADCH; // store ADC values

PORTA = 0xFF;

PORTB = 0x50;  //enable tristate buffer for pushbutton switches
_delay_ms(.1);

	if( (pressed_bit_0() == TRUE) ){
		mode ^= ( 1 << 3);//toggle mode and LED to light up on bar graph
	}//select the mode or toggle spi LED

	else if( (pressed_bit_1() == TRUE) ){
		mode ^= ( 1 << 6); // toggle mode and LED to light up on bar graph
	}

PORTB = 0x70; // disable push buttons
//_delay_ms(.1);
increment = 1; //if no buttons are pressed then increment by one

	if(mode == (0x48)){increment = 0;} //if both buttons are pressed then dont increment
	if(mode == ((0x08))){increment = 2;} // if right one is pressed then increment by 2
	if(mode == (0x40)){increment = 4;} // if left one is pressed then increment by 4


write_SPI(mode);//write to bar graph

read_SPI();//read in from the SPI
read = process_EN(); // decrypt the data from the SPI and determine the encoder movement
update_EN(read);// increase the count regarding the modes

//DDRA = 0xFF; //set PORTA to all outputs


//test code for ADC values 
//itoa(adc_data,lcd_str_h,10); // converts integer into ascii string  decimal
//string2lcd(lcd_str_h);

//for(int i=0;i<=10;i++){_delay_ms(50);}  //delay 0.5 sec

//clear_display();

//cursor_home();

/*
	if(new_data){
		process_ADC_data();
	}
*/
	
int temp= 0; // dummy variable 

PORTA = 0xFF;  //make PORTA an input port with pullups 	

  //bound the count to 0 - 1023
  if (count > 1023){
	  count = 0;
     }
// bound the count 1023 - 0
if(count < 0){temp = count;}//need to save the negative value to add toward the decrement 
if( count < 0){
	count = 1024 + temp; // add the decrement

}
  //break up the disp_value to 4, BCD digits in the array: call (segsum)
 segsum(count); // send the current count to LED Display

_delay_ms(.1);
 //DIGIT SHIFTS
 //bound a counter (0-4) to keep track of digit to display ;
PORTB = 0x00;
 // put it in a for loop
//int index;

for(int index = 0; index < 5; index++){
	PORTB = index << 4; // within 4 digits for the hex value
	PORTA = segment_data[index]; //send 7 segment code to LED segments
	if( (PORTB == 0x40)&&(count< 1000) ){
	segment_data[5] = dec_to_7seg[10];
	PORTA = segment_data[5];}

	_delay_ms(1); // add delay
}




//make PORTA an output

DDRA = 0xFF;  
sei(); // ISR will return here
//clear_display();
}//while

}//main

