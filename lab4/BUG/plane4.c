// plane4.c 
// 11/9/21 flight to orlando changes
// Benson Ton


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


unsigned char 	new_data = FALSE;
uint16_t 	adc_data; //holds the adc raw values

char    lcd_string_array[16] = {"alarm"};  //holds a string to refresh the LCD

volatile int time_setting = FALSE; //12 hr or 24 hr setting

//alarm variables
volatile int	al_hr12 = 12;
volatile int 	al_min12 = 0;
volatile int 	al_sec = 0; //test
volatile int 	alarm = FALSE;
volatile int	sec_temp;
volatile int	min_temp;
volatile int	hr_temp;
volatile int dp = FALSE;
volatile int toggle = FALSE;
volatile int holder;


//time variables
volatile int 	hours = 12;
volatile int 	minutes = 59;
volatile int 	seconds = 0;
volatile int 	count = 0; //LED display count
volatile int 	increment = 0; // count increment status
volatile uint8_t mode = 0; // mode select state

//encoder variables
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
uint8_t segment_data[6] = {0xFF}; 

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[13]={0xC0,0xF9,0xA4 ,0xB0 ,0x99 ,0x92 ,0x82,0xF8 ,0x80 ,0x98,0xFF,0x07,0x7F};//numbers from 0-9, all blank, blank colon



//NOTE:: TRY TO FIX BUG
// LOOK OVER PRESSING THE CHANGE TIME AND ALARM BUTTON AT THE SAME TIME CONDITIONS


/////////////////////////////////////oscilator tone
ISR(TIMER1_COMPA_vect){
	//if alarm matches and not in alarm toggle mode then trigger the ALARM!
	if( ( (alarm_compare()) && (mode != 0x40)) ){
		
TCCR3A = (1 << COM3A1 | (1 << WGM31) ); //normal operation w/ OC3A is disconnected
		
TCCR1A = 0x00; // normal operation w/ OC1A is disconnected
		DDRE |= 1 << PE3;
		PORTC ^= 1 << PC0;
	}
}


/////////////////////////////////////dimming for LED display 
ISR( TIMER2_COMP_vect){
	
//	if(adc_data > 70){OCR2 = 10;} // dim the light if it is bright
//	else{OCR2 = 200;} // brighten the light otherwise
	OCR2 = adc_data;
}

/////////////////////////////////////////// TIMER0 overflowIS
ISR( TIMER0_COMP_vect ) {

//save the states of PORTA and PORTB
saveA = PORTA;
saveB = PORTB;

//read_SPI();//read in from the SPI
//read = process_EN(); // decrypt the data from the SPI and determine the encoder movement
//update_EN(read);// increase the count regarding the modes
//alarm = FALSE;
static uint8_t timer = 0;  //hold value of count between interrupts
timer++;  //extend counter
int seconds_holder = 1;

increment = 0;

if((timer% 64) == 0){ // turn off for half a second 
	segment_data[2] = 0x07; //turn off colon
} 

if((timer% 128) == 0){
	segment_data[2] = 0xC; //turnign on colon
	seconds += seconds_holder; //one second has passed
}	

time_tracker_12(seconds, minutes); //keep track of the time in 12 hr format 

/*
//STOP CLOCK to change time
if( (mode != 0x08) && (mode != 0x40)){//if setting the time stop the clock 

//	if((timer% 512) == 0){ //trigger 1 second increments 
//		segment_data[2] = 0xC; // turn on colon
 //		seconds++; // increment seconds 
//	}//512 final
   		if((timer% 512) == 0){
			segment_data[2] = 0xC;
 	   		seconds++;
		} // test, FASTER

time_tracker_12(seconds, minutes); //keep track of the time in 12 hr format 
//time_tracker_24(seconds, minutes); // keep track of the time in 24 hr format
}
*/
/*
//SET ALARM
if((mode == 0x40) ){//if setting the time stop the clock 
		alarm = TRUE;
		dp = TRUE;
		read_SPI();//read in from the SPI
		read = process_EN(); // decrypt the data from the SPI and determine the encoder movement
		update_EN_alarm(read);// increase the count regarding the modes

//you can set an if statement to change the bounds whether setting == true or false
		alarm_bound_12(); //bound the time for 12hr mark
//		alarm_bound_24(); //bound the time for 24hr mark	
		alarm_set(); //save temp values
	}	

*/


//time_bound_12(); // set bounds for changing the time with the encoders 12 hr
//time_bound_24(); // set bounds for changing the time with encoders 24 hr

PORTB = 0x50;  //enable tristate buffer for pushbutton switches
_delay_ms(.1);//need a delay to active buffer

//increment = 0; // initial increment of the encoders would be 0 when timer is running

//ALERT; WHEN TRIGGERING ALARM AND TIME IT IS UNDER THE SAME MODE!!!! IT WILL CAUSE AN ERROR SO FIX THAT
//MAYBE LOOK OVER A NEW VARIABLE AND USE THE COMBINATION(ADDITION) OF THEM TO SET A STATEMENT
//SAME WITH THE SNOOZE


	//MIGHT NEED TO CHANGE IT TO THE MAIN ISR FOR TIME RUNNING FUNCTION AND HAVE THE CHECK BUTTONS TO STOP THE TIME

//SETS TIME
//if( (chk_buttons(7) == TRUE) ){
//	mode ^= ( 1 << 3);
  //	increment = 1;	
//	if((timer% 512) == 0){
//			segment_data[2] = 0xC;
// 	   		seconds++;
//	}	

//} //toggle mode and LED to light up on bar graph

//_delay_ms(.1);



//time_tracker_12(seconds, minutes); //keep track of the time in 12 hr format 
/*
//SETS ALARM
if( (chk_buttons(0) == TRUE) ){//mode ^= (1 << 6);
		alarm = TRUE;
		dp = TRUE;
		read_SPI();//read in from the SPI
		read = process_EN(); // decrypt the data from the SPI and determine the encoder movement
		update_EN_alarm(read);// increase the count regarding the modes
		increment = 1;
//you can set an if statement to change the bounds whether setting == true or false
		alarm_bound_12(); //bound the time for 12hr mark
//		alarm_bound_24(); //bound the time for 24hr mark	
		alarm_set(); //save temp values


}// toggle mode and LED to light up on the bar graph
*/

//alarm = FALSE;

//SETS TIME
if( (chk_buttons(7) == TRUE) ){
	mode ^= ( 1 << 3);
//_delay_ms(.1);
//		increment = 1; //encoder increment
//		seconds_holder = 0; // seconds increment
	
} //toggle mode and LED to light up on bar graph


//_delay_ms(.1);

// SETS ALARM
else if( (chk_buttons(0) == TRUE) ){
//	mode ^= (1 << 6);
//_delay_ms(.1);
		alarm = TRUE;
		dp = TRUE;
	//	read_SPI();//read in from the SPI
	//	read = process_EN(); // decrypt the data from the SPI and determine the encoder movement
	//	update_EN_alarm(read);// increase the count regarding the modes

//you can set an if statement to change the bounds whether setting == true or false
		alarm_bound_12(); //bound the time for 12hr mark
//		alarm_bound_24(); //bound the time for 24hr mark	
		alarm_set(); //save temp values

	}
//_delay_ms(.1);


//SNOOZE ALARM
else if( (chk_buttons(3) == TRUE) ) { 
//	mode ^= ( 1 << 2);
//_delay_ms(.1);
//holder = hr_temp; //save the current hour	
//	hr_temp = -1; // give it a null value
		holder = hr_temp; //save the current hour	
		hr_temp = -1; // give it a null value
		if( (seconds % 10) == 0){hr_temp = holder;} //after 10 seconds has passed then trigger alarm


} // toggle mode and LED to light up on bar graph


//if(mode == 0x04){
//	if( (seconds % 10) == 0){hr_temp = holder;} //after 10 seconds has passed then trigger alarm
//}


//_delay_ms(.1);
//DISABLE ALARM
else if( (chk_buttons(2) == TRUE) ){ // turn off the alarm permanently and dont display on bar graph.
//	mode ^= 1 << 7;
	alarm = FALSE;	
	dp = FALSE;	
	clear_display();
	hr_temp = -1;
	PORTC &= ~(1 << PC0);
	//change the tccr? stop the tone
	DDRE &= ~(1 << PE3);
//_delay_ms(.1);
       //change  tccr? to stop the volume	
}// check to disable the alarm

//else{mode = 0;}

//_delay_ms(.1);
PORTB = 0x70;//disable tristate

write_SPI(mode); // write to the bar graph

/*
if(mode == 0x08){
	increment = 1; // increment by 1 when timer is not running for the encoders
}

if(mode == 0x40){
	increment = 1; //increment by 1 when setting the alarm
}
*/

//alarm = FALSE;


switch(mode){
	case 0x40: //setting alarm PA0
		alarm = TRUE;
		dp = TRUE;
		increment = 1;
//		read_SPI();//read in from the SPI
//		read = process_EN(); // decrypt the data from the SPI and determine the encoder movement
//		update_EN_alarm(read);// increase the count regarding the modes

		//you can set an if statement to change the bounds whether setting == true or false
		alarm_bound_12(); //bound the time for 12hr mark
		//alarm_bound_24(); //bound the time for 24hr mark	
		alarm_set(); //save temp values
		break;
	case 0x08: //setting time
		increment = 1; //encoder increment
		seconds_holder = 0; // seconds increment
		break;
	
	case 0x04: //snooze alarm
		holder = hr_temp; //save the current hour	
		hr_temp = -1; // give it a null value
		if( (seconds % 10) == 0){hr_temp = holder;} //after 10 seconds has passed then trigger alarm
		break;
} 

time_bound_12(); // set bounds for changing the time with the encoders 12 hr


read_SPI();//read in from the SPI
read = process_EN(); // decrypt the data from the SPI and determine the encoder movement
update_EN(read);// increase the count regarding the modes
//update_EN_alarm(read);// increase the count regarding the modes
//read_SPI();
//write_SPI(mode); // write to the bar graph

//if(mode == 0x08){
//alarm_bound_12(); //bound the time for 12hr mark
//alarm_set(); //save temp values
//}

//restore the state when leaving the ISR

PORTA = saveA;
PORTB = saveB;

DDRA = 0xFF; //set PORTA to all outputs


}//end of ISR Timer0

/********************************************************************
 *				alarm_set
 *******************************************************************/
void alarm_set(void){
//save the alarm time that the user chooses
hr_temp = al_hr12;
min_temp = al_min12;
}


/********************************************************************
 *				alarm_compare
 *******************************************************************/
int alarm_compare(void){

//return true if alarm time and real time match
if( (hours == hr_temp) && (minutes == min_temp)){
	return TRUE;
}
return FALSE; // return false otherwise
}

/********************************************************************
 *				alarm_bound_24
 * used to bound the alarm with respect to the time setting when setting
 * the alarm with the encoders.
 ********************************************************************/
void alarm_bound_24(void){
//bound minutes from 0 to 60
if (al_min12 > 59){
	  al_min12 = 0;
}

if(al_min12 < 0){al_min12 = 59;}//need to save the negative value to add toward the decrement 

//bound the hours from 12 to 1 and 1 to 12
if(al_hr12 > 23){al_hr12 = 0;}
if(al_hr12 < 0){al_hr12 = 23;}
}


/********************************************************************
 *				alarm_bound_12
 * used to bound the alarm with respect to the time setting when setting
 * the alarm with the encoders.
 ********************************************************************/
void alarm_bound_12(void){
//bound minutes from 0 to 60
if (al_min12 > 59){
	  al_min12 = 0;
}

if(al_min12 < 0){al_min12 = 59;}//need to save the negative value to add toward the decrement 

//bound the hours from 12 to 1 and 1 to 12
if(al_hr12 > 12){al_hr12 = 1;}
if(al_hr12 < 1){al_hr12 = 12;}
}

/********************************************************************
 *				time_bound_24
 * used to bound the correct time when wanting to change the real time 
 * with encoders
 ********************************************************************/
void time_bound_24(void){
//bound the minutes from 0 to 60
if (minutes > 59){
	  minutes = 0;
}

// bound the minutes from 60 to 0
if(minutes < 0){minutes = 59;}//need to save the negative value to add toward the decrement 

//bound the hours from 24 to 0 and 0 to 24
if(hours > 23){hours = 0;}
if(hours < 0){hours = 23;}
}

/********************************************************************
 *				time_bound_12
 * used to bound the correct time when wanting to change the real time 
 * with encoders
 ********************************************************************/
void time_bound_12(void){
//bound the minutes from 0 to 60
if (minutes > 59){
	  minutes = 0;
}

// bound the minutes from 60 to 0
if(minutes < 0){minutes = 59;}//need to save the negative value to add toward the decrement 

//bound the hours from 12 to 1 and 1 to 12
if(hours > 12){hours = 1;}
if(hours < 1){hours = 12;}
}
/***************************************************************
/				time_tracker_12
****************************************************************/
void time_tracker_12(int sec, int min){

	//if 60 seconds has been reached then increase the minutes by one
	if(sec == 60){
		minutes++; // increment minutes 
		min = minutes; 
		seconds = 0; // reset seconds
		//increment hours if 60 minutes has passed
		if(min > 59){ 
			hours++;
			minutes = 0;//reset minutes
			if(hours > 12){ hours = 1;} // reset hours
		}
	}
}
/***************************************************************
/				time_tracker_24
****************************************************************/
void time_tracker_24(int sec, int min){

	//if 60 seconds has been reached then increase the minutes by one
	if(sec == 60){
		minutes++; // increment minutes 
		min = minutes; 
		seconds = 0; // reset seconds
		//increment hours if 60 minutes has passed
		if(min > 59){ 
			hours++;
			minutes = 0;//reset minutes
			if(hours > 23){ hours = 0;} // reset hours
		}
	}
}

/********************************************************************
 *				update_encoder_alarm
 ********************************************************************/

void update_EN_alarm(int val_rot){
//changes the alarm time without affecting real time
	if(val_rot == 1){ al_hr12 += increment;} //if rotating to the right for left encoder then increment 
	else if(val_rot ==0){al_hr12-= increment;}//if rotating to the left for left encoder then decrement
	else if(val_rot == 2){al_min12-= increment;} // if rotating to the left for right encoder then decrement
	else if(val_rot == 3){al_min12+= increment;}// if rotatiing to the right for right encoder then increment
	else{val_rot =-1;} // dont do anything if nothing is rotated
}

/********************************************************************
 *				update_encoder
 ********************************************************************/

void update_EN(int val_rot){
//changes the real time without affecting alarm time
	if(val_rot == 1){ hours += increment;} //if rotating to the right for left encoder then increment 
	else if(val_rot ==0){hours-= increment;}//if rotating to the left for left encoder then decrement
	else if(val_rot == 2){minutes-= increment;} // if rotating to the left for right encoder then decrement
	else if(val_rot == 3){minutes+= increment;}// if rotatiing to the right for right encoder then increment
	else{val_rot =-1;} // dont do anything if nothing is rotated
}


/********************************************************************
 *				read_SPI
 ********************************************************************/
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
 ********************************************************************/
void write_SPI(uint8_t value){

SPDR = value; // take in which mode it is currently on and display it 
	while (bit_is_clear(SPSR,SPIF)) {} //wait till data is sent out
PORTD |= (1 << PD2); //SEND data to bargraph, rising edge
PORTD &= ~(1<<PD2); // falling edge
}

//*******************************************************************************
//                            chk_buttons                                  
// Checks the state of pushbutton S0 It shifts in ones till the button is pushed. 
// Function returns a 1 only once per debounced button push so a debounce and toggle 
// function can be implemented at the same time. Expects active low pushbutton on 
// Port A  bit zero. Debounce time is determined by external loop delay times 12. 
//*******************************************************************************
int chk_buttons(int button) {
static uint16_t states[8] = {0}; // an array to store the states of all buttons on the button board

// states[0] corresponds to S1 on the board and states[7] corresponds to S8
states[button] = (states[button]<<1 | (! bit_is_clear(PINA, button)) | 0xE000); //first extract the bit that corresponds to the button
//Check if button is pressed
if (states[button] == 0xF000) {return TRUE;} 
return FALSE;
}

//***********************************************************************************
//                                   minutes_seg                                    
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit 
//segment_data will hold hours and minutes.
//indices 3-4 will hold the hours
//indices 0-1 will hold the minutes                      
//**********************************************************************************

void minutes_seg(uint16_t min, uint16_t hr) {

//initialzing the local variables of the function
int minutes_index = 0; // focus on the lower number indices
int temp = 0; //temprary variable
int hours_index = 3; //focus on the higher number indices

//base case starting at 0 minutes and display 0|0
if(min == 0){
	segment_data[0] = dec_to_7seg[0];
	segment_data[1] = dec_to_7seg[0];
minutes_index++;
}
//convert the min value and store into segment_data array 
	while(min != 0){
		temp = min % 10; // isolate to a single digit
	        if(minutes_index == 2){minutes_index++;};
		segment_data[minutes_index] = dec_to_7seg[temp]; //use the digit to store the hexidecimal value to seg array
		min = min/10; // continue to the calculation to the next digit
		minutes_index++; // increase the minute index
	}

//convert the hr value and store into segment_data array 
	while(hr != 0){
		temp = hr % 10; //isolate to a single digit
		segment_data[hours_index] = dec_to_7seg[temp];//use the digit to store the hexidecimal value to seg array
		hr = hr/10; // continute the calculation to the next digit
		hours_index++; // increase hr index
	}
//clear the leading blanks for the hours
if(hours_index == 4){
	if(hr == 0){
	segment_data[4] = dec_to_7seg[0];
	segment_data[5] = dec_to_7seg[0];
	}
	else{for(int i = 4; i < 5; i++){segment_data[i] = dec_to_7seg[10];}}
}
  }



/**********************************************************************
 *				process_EN
 *
 * once the state machine completes all 4 cycles then it means the knob has been shifted once
 * encoder function for the left and right knobs 
 * Handles all cases of any rotation of the encoders
 **********************************************************************/
int process_EN(void){
//Breaking up the raw data from the encoder into a and b pins

uint8_t new_A = ((raw_encoder_val & 0x01) ==0) ? 0:1;
uint8_t new_B = ((raw_encoder_val & 0x02) ==0) ? 0:1;

//process the right encoder
uint8_t new_C = ((raw_encoder_val & 0x04) ==0) ? 0:1;
uint8_t new_D = ((raw_encoder_val & 0x08) ==0) ? 0:1;

int en_val = 0;

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



/**********************************************************************
 *				spi_init
 **********************************************************************/
void spi_init(void){
DDRB |= ( (1 << PB2) | ( 1<< PB1) | (1<<PB0) ); //mosi and serial clock
SPCR |= ( (1 << MSTR) | (1 << SPE) ); //master mode, SPI enabled, clk low on idle, leading edge sample


SPSR |= 1 << SPI2X;
DDRB |= 0xF0; // set PORTB 4-7 bits as outputs
DDRA = 0x03; // set PORTA 6-7 bits as inputs
DDRD = (1 << PD2); // set PD2 as output
DDRE = 0xFF;// set PORTE to all outputs
}

/**********************************************************************
 *				volume_init
 **********************************************************************/
void volume_init(void){
//DDRE = 0xFF; //Set PORTE as outputs
//TCCR3A = (1 << COM3A1 | (1 << WGM31) ); //normal operation w/ OC3A is disconnected
TCCR3B |= (  (1 << CS30) | ( 1 << WGM32) | (1 << WGM33) ); //fast pwm 
ICR3 = 10; // Set overflow top 
OCR3A = 5; // set compare match
}

/**********************************************************************
 *				tone_init
 **********************************************************************/
void tone_init(void){
DDRC = 0xFF; // set all of PORTC as outputs
//TCCR1A = 0x00; // normal operation w/ OC1A is disconnected
TIMSK |= 1 << OCIE1A; //output compare enable for timercounter1A
OCR1A = 3999; // 4kHz and triggers ISR
TCCR1B |= (1<<WGM12 | 1<<CS10); // CTC mode, clear with OCR and no prescaling
}

/**********************************************************************
 *				clock_init
 **********************************************************************/
void clock_init(void){
ASSR   |=  1<<AS0; //ext osc TOSC in 32kHz 
//TIMSK |= 1 << OCIE0; //interrupt flag on compare register 0
//TCCR0 |= ((1 << WGM01) | (1 << CS00)); //CTC mode, with clk/no pre-scaling
//OCR0 = 127; // Set top 
//OCR0 = 150;
TIMSK |= 1 << TOIE0;
TCCR0 |= 1 << CS00;

}

/**********************************************************************
 *				dimming_init
 **********************************************************************/
void dimming_init(void){
TCCR2 |= (1 << WGM21) | ( 1 << WGM20) | (1 << COM21) | ( 1 << CS20); // no prescale, clear on OC2 match, fast pwm mode
TIMSK |= (1 << OCIE2); // Compare match interrupt enable timer/counter0
TCNT2 = 0; //set count at 0
}



/**********************************************************************
 *				ADC_init
 **********************************************************************/
void ADC_init(void){
DDRF  &= ~(_BV(DDF7)); //make port F bit 7 the ADC input  
PORTF &= ~(_BV(PF7));  //port F bit 7 pullups must be off
	
ADMUX = (1 << REFS0) | (1 << ADLAR) | 0x07; // reference to AVCC(5v), left adjusted (only reading  8bit), and channel 7
//ADC enabled, single conversion, division factor is 128
ADCSRA = (1 << ADEN) | (1 << ADPS2) | ( 1 << ADPS1) | ( 1 << ADPS0) ; 
}


//***********************************************************************************
int main()
{
DDRE &= ~(1 << PE3);
spi_init();	//initialize SPI and port configurations
lcd_init();	//initialize LCD display
clear_display(); //Clean LCD display

ADC_init();	//initialize ADC
dimming_init();	//initialiing timer/counter2 (TCNT2) for the dimming pwm
tone_init();	//initialize osciallting tone w/ timer/counter1 
clock_init();	//initializing timer/counter0 (TCNT0) 
volume_init();	//initialize timer/counter3



while(1){ // main loop

ADCSR |= (1<<ADSC); //start writing 
	while(bit_is_clear(ADCSRA, ADIF)){};

ADCSR |= (1 << ADIF);//clear flag by writing one
adc_data = ADCH; // store ADC values
	

PORTA = 0xFF;  //make PORTA an input port with pullups 	

	//if ALARM is set then change the LED screen to let user to choose a desired time
	if(alarm == TRUE){minutes_seg(al_min12,al_hr12);
	//send to LCD to let user know ALARM is set
		string2lcd(lcd_string_array);
  		cursor_home(); 
	}

	else{minutes_seg(minutes, hours);} // send to LED to display time

//DIGIT SHIFTS
//bound a counter (0-4) to keep track of digit to display ;
PORTB = 0x00;

	for(int index = 0; index < 5; index++){
		PORTB = index << 4; // within 4 digits for the hex value
		PORTA = segment_data[index]; //send 7 segment code to LED segments
		_delay_ms(1); // add delay	
}

	//handles decimal point display when alarm is triggered
	if(dp == TRUE){
		PORTB = 0; // digit 0
		PORTA = 0x7F; // turn on decimal point
	}
	
DDRA = 0xFF;  //make PORTA an output
sei(); // ISR will return here

}//while

}//main

