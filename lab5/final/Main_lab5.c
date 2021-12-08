// Main_lab5.c 
// final folder for lab 5
// Benson Ton
//implented master code

//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
//  PORTB bit 7 goes to the PWM transistor base.

#define TRUE 1
#define FALSE 0
#define BUTTON_ONE 1
#define BUTTON_ZERO 0


#include <string.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#include "uart_functions.h" 
#include "lm73_functions.h"
#include "twi_master.h"
#include "hd44780.h"


//GLOBAL VARIABLES
uint16_t 	i;

//Temperature variables
extern uint8_t lm73_rd_buf[2];
extern uint8_t lm73_wr_buf[2];
char    	uart_buf[16];
volatile uint8_t  rcv_rdy;
char              rx_char;

//Remote temperature variables 
char		alarm_str[32] = {"ALARM"};
char		lcd_temp_array[32] = {"local:          remote:         "};//LCD format for temperatures
char		temp_str[32];
volatile int	lcd_display = FALSE;


//ADC variables
unsigned char 	new_data = FALSE;
uint16_t 	adc_data; //holds the adc raw values


volatile int time_setting = FALSE; //12 hr or 24 hr setting

//alarm variables
volatile int 	alarm = FALSE;		//alarm flag
volatile int 	dp = FALSE;		//decimal point flag
volatile int 	time = 1000;      	//set time
volatile int 	alarm_time = 1200;	//set start alarm time as 1200
int 		tone = FALSE; 		//tone flag
int 		display = FALSE; 	//display the alarm flag
int 		snooze_time = FALSE; 	//snooze flag 
volatile int	snooze_seconds = 0;	//snooze time

//time variables
volatile int 	seconds = 0;  	//seconds of real time
volatile int 	incr_time = 0; 	// count increment status
volatile int 	incr_alarm = 0; // count increment alarm status
volatile uint8_t mode = 0; 	// mode select state
volatile uint8_t setting = 0;

volatile int seconds_holder = 1;

//encoder variables
volatile uint8_t raw_encoder_val;	// raw data for the left encoder
volatile uint8_t raw_encoder_valR; 	// raw data for the right encoder
volatile int read = 0; 			// status of whether the encoder turned CW or CCW
int state_counter = 0; 			// status of the pins being contacted of the left encoder
int r_counter = 0; 			// status of the pins being contacted of the right encoder

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
uint8_t segment_data[5] = {0xFF}; 

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[13]={0xC0,0xF9,0xA4 ,0xB0 ,0x99 ,0x92 ,0x82,0xF8 ,0x80 ,0x98,0xFF,0x07,0x7F};//numbers from 0-9, all blank, blank colon


/********************************************************************
 *				alarm_compare
 *******************************************************************/
void alarm_compare(void){
	//checks alarm flag and whether button is pressed
	if(( mode != 0x08 && mode != 0x40)){  

		//triggers the alarm if alarm time matches with real time 
		if((alarm_time == time) && (tone == TRUE)){
		OCR3A = 5; 
		TCCR1B |= 1 << CS10;
		}
	}	
}

//*******************************************************************************
//                            chk_buttons                                  
// Checks the state of pushbutton S0 It shifts in ones till the button is pushed. 
// Function returns a 1 only once per debounced button push so a debounce and toggle 
// function can be implemented at the same time. Expects active low pushbutton on 
// Port A  bit zero. Debounce time is determined by external loop delay times 12. 
//*******************************************************************************
uint8_t chk_buttons(int button) {
static uint16_t states[8] = {0}; // an array to store the states of all buttons on the button board

// states[0] corresponds to S1 on the board and states[7] corresponds to S8
states[button] = (states[button]<<1 | (! bit_is_clear(PINA, button)) | 0xE000); //first extract the bit that corresponds to the button
//Check if button is pressed
if (states[button] == 0xF000) {return TRUE;} 
return FALSE;
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




/********************************************************************
 *				update_encoder
 ********************************************************************/

void update_EN(int val_rot){
//changes the real time without affecting alarm time
	if(val_rot == 1){ time += (100*incr_time);} //if rotating to the right for left encoder then increment 
	else if(val_rot ==0){time -= (100*incr_time);}//if rotating to the left for left encoder then decrement
	else if(val_rot == 2){time -= incr_time;} // if rotating to the left for right encoder then decrement
	else if(val_rot == 3){time += incr_time;}// if rotatiing to the right for right encoder then increment
	else{val_rot =-1;} // dont do anything if nothing is rotated
}

/********************************************************************
 *				update_encoder_alarm
 ********************************************************************/
void update_EN_alarm(int val_rot){
//changes the alarm time without affecting real time
	if(val_rot == 1){ alarm_time += (100*incr_alarm);} //if rotating to the right for left encoder then increment 
	else if(val_rot ==0){alarm_time -= (100*incr_alarm);}//if rotating to the left for left encoder then decrement
	else if(val_rot == 2){alarm_time -= incr_alarm;} // if rotating to the left for right encoder then decrement
	else if(val_rot == 3){alarm_time += incr_alarm;}// if rotatiing to the right for right encoder then increment
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


//oscilator tone
ISR(TIMER1_COMPA_vect){


	DDRE |= 1 << PE3; // PE1 will output for volume
	PORTC ^= 1 << PC0; // PC0 will toggle tone
}


//dimming for LED display 
ISR( TIMER2_COMP_vect){
	
	OCR2 = adc_data;
}


// TIMER0 Compare Match
ISR( TIMER0_COMP_vect ) {


//save the states of PORTA and PORTB
saveA = PORTA;
saveB = PORTB;

refresh_lcd(lcd_temp_array); //display the interface with the temperature and alarms

alarm_compare(); //checks if the alarm set time and the clock time matches then trigger alarm


//populate the top right corner of the LCD for the alarm
if(alarm == TRUE){
	int a = strlen(alarm_str);
	for(i = 0; i < a; i++ ){
		lcd_temp_array[11 + i] = alarm_str[i];
	}
}


static uint16_t timer = 0;  //hold value of count between interrupts
timer++;  //extend counter


if((timer% 128) == 0){ // turn off for half a second 
	segment_data[2] = 0x07; //turn off colon
} 

if((timer% 512) == 0){
	uart_puts(temp_str);		//send a string through the UART
	uart_putc('\0');		//ADD a null character
	segment_data[2] = 0x0C; 	//turn on colon
 	seconds+= seconds_holder; 	//increment seconds
	snooze_seconds += seconds_holder;//increment seconds of snooze
} 


PORTB = 0x50;  //enable tristate buffer for pushbutton switches
_delay_ms(.1);//need a delay to active buffer


if(chk_buttons(0)){mode ^= 1<<6;}//sets alarm

if (chk_buttons(3)){mode ^= 1 << 2;}//snooze alarm
if (chk_buttons(2)){mode ^= 1 << 5;}//disable alarm
if (chk_buttons(7)){mode ^= 1 << 3;} //sets time
	
PORTB = 0x70;//disable tristate


read_SPI();		//read in from the SPI
read = process_EN(); 	// decrypt the data from the SPI and determine the encoder movement
update_EN(read);	// increase the time with the encoder movement 
update_EN_alarm(read);	// increase the alarm_time with the encoder movement

write_SPI(mode); 	// write to the bar graph

//restore the state when leaving the ISR
PORTA = saveA;
PORTB = saveB;

DDRA = 0xFF; 		//set PORTA to all outputs


}//end of ISR Timer0




/********************************************************************
 *				disable_alarm
 *******************************************************************/
void disable_alarm(void){

		alarm = FALSE;	//alarm is off
		dp = FALSE;	//turn off the decimal point
		tone = FALSE;	//turn off the tone
		snooze_time = FALSE; //disable snooze
		int a = strlen(alarm_str);//get the length of the alarm string
		//clear alarm string
		for(i = 0; i < a; i++ ){
			lcd_temp_array[11 + i] = ' ';
		}
		refresh_lcd(lcd_temp_array);//update the lcd display
		OCR3A = 0;	// clear the compare
		TCCR1B &= ~(1 << CS10);	//clear the clock
		mode &= ~(1 << 5); // set conditions only once so clear the bit
}


/********************************************************************
 *				alarm_bound_12
 * used to bound the alarm with respect to the time setting when setting
 * the alarm with the encoders.
 ********************************************************************/
void alarm_bound_12(void){
int hours = alarm_time/100;  // convert integer time into hours
int minutes = alarm_time - (hours*100); // convert the integer time into minutes

if(minutes == 99){alarm_time -= 40;alarm_time+=100;} // decrement hours; ex (200 -> 159)

//bound the minutes from 0 to 60
if (minutes == 60){
	  alarm_time += 40;
	  alarm_time -= 100;
}


// bound the minutes from 60 to 0
//bound the hours from 12 to 1 and 1 to 12
if(hours > 12){alarm_time = 100;alarm_time += minutes;}
if(hours < 1){alarm_time = 1200;alarm_time += minutes;}

}



/********************************************************************
 *				time_bound_12
 * used to bound the correct time when wanting to change the real time 
 * with encoders
 ********************************************************************/
void time_bound_12(void){
int hours = time/100;  // convert integer time into hours
int minutes = time - (hours*100); // convert the integer time into minutes

if(minutes == 99){
	time -= 40;
	if(time == 59){time +=1200;} //FIX THE 100 TO 1259 BUG
	else{time+=100;}
} // decrement hours; ex (200 -> 159)
//bound the minutes from 0 to 60
if (minutes == 60){
	  time += 40;
	  time -= 100;
}


// bound the minutes from 60 to 0

//bound the hours from 12 to 1 and 1 to 12

if(hours > 12){time = 100;time += minutes;}
if(hours < 1){time = 1200;time += minutes;}
}


/***************************************************************
/				time_tracker_12
****************************************************************/
void time_tracker_12(void){
int hours = time/100;
int minutes = time - (hours*100);
	//if 60 seconds has been reached then increase the minutes by one
	if(seconds > 59){
		time++; // increment minutes 
		seconds = 0; // reset seconds
	}
		//increment hours if 60 minutes has passed
	if(minutes > 59){ 
	time += 40; // increment the time ex (160 -> 200)
	hours++;
	}
	
	if(hours > 12){ time = 100;} // reset hours to 1 o'clock
}


/**********************************************************************
 *				digit_display
 **********************************************************************/
void digit_display(void){
int index;

PORTB = 0x00;

	for(index = 0; index < 5; index++){
		PORTA = segment_data[index]; //send 7 segment code to LED segments
		_delay_ms(1.5);
		PORTB += 0x10; // within 4 digits for the hex value
	}
}

/**********************************************************************
 *				spi_init
 **********************************************************************/
void spi_init(void){
DDRB |= ( (1 << PB2) | ( 1<< PB1) | (1<<PB0) ); //mosi and serial clock
SPCR |= ( (1 << MSTR) | (1 << SPE) ); //master mode, SPI enabled, clk low on idle, leading edge sample
SPSR |= 1 << SPI2X;

//DDRF |= 0x08; //lcd strobe bit
DDRB |= 0xF0; // set PORTB 4-7 bits as outputs
DDRA = 0x03; // set PORTA 6-7 bits as inputs
DDRD = (1 << PD2); // set PD2 as output
DDRE = 0xFF;// set PORTE to all outputs
}

/**********************************************************************
 *				volume_init
 **********************************************************************/
void volume_init(void){
//volume pin is PE3
TCCR3A = (1 << COM3A1 | (1 << WGM31) ); //normal operation w/ OC3A is disconnected
TCCR3B |= (  (1 << CS30) | ( 1 << WGM32) | (1 << WGM33) ); //fast pwm 
ICR3 = 10; // Set overflow top ; controls frequency 
OCR3A = 0; // set compare match; duty cycle

}

/**********************************************************************
 *				tone_init
 **********************************************************************/
void tone_init(void){
DDRC = 0xFF; // set all of PORTC as outputs
TCCR1A = 0x00; // normal operation w/ OC1A is disconnected
TIMSK |= 1 << OCIE1A; //output compare enable for timercounter1A
OCR1A = 3999; // 4kHz and triggers ISR
//clk is not set
TCCR1B |= (1 << WGM12); // CTC mode, clear with OCR and no prescaling
}

/**********************************************************************
 *				clock_init
 **********************************************************************/
void clock_init(void){
ASSR   |=  1<<AS0; //ext osc TOSC in 32kHz 
TIMSK |= 1 << OCIE0; //interrupt flag on compare register 
TCCR0 |= ((1 << WGM01) | (1 << CS00)); //CTC mode, with clk/no pre-scaling
OCR0 = 63; // Set top 
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


/**********************************************************************
 *				segsum
 *Takes in an 16-bit value and display it on the LED screen 
 *********************************************************************/
void segsum(uint16_t sum) {

//initialzing the local variables of the function
int digit_index = 0;
int temp = 0;
int i;

//display 0 if the value is 0
if(sum == 0){segment_data[digit_index] = dec_to_7seg[0];};

        while(sum > 0){
                temp = sum % 10; // isolate to a single digit
                if(digit_index == 2){digit_index++;};
                segment_data[digit_index] = dec_to_7seg[temp]; //use the digit to store the hexidecimal value to seg array
                sum = sum/10; // continue to the next digit
                digit_index++; // increase the digit index
        }
        //segment_data[2] = dec_to_7seg[11]; //replace the colon with a blank colon

//need to turn off the rest of the digits
	if (digit_index < 5) //if there are less digits than segment numbers
	{
        	for (i = digit_index; i < 5; i++){segment_data[i] = dec_to_7seg[10];} //blank them out
	}
}

//***********************************************************************************
uint8_t main()
{
spi_init();	//initialize SPI and port configurations
init_twi(); //initalize TWI (twi_master.h)  
uart_init(); 	//initialize uart
lcd_init();	//initialize LCD display
clear_display(); //Clean LCD display

ADC_init();	//initialize ADC
dimming_init();	//initialiing timer/counter2 (TCNT2) for the dimming pwm
tone_init();	//initialize osciallting tone w/ timer/counter1 
clock_init();	//initializing timer/counter0 (TCNT0) 
volume_init();	//initialize timer/counter3

uint16_t lm73_temp;  //a place to assemble the temperature from the lm73

refresh_lcd(lcd_temp_array);
while(1){ // main loop

ADCSR |= (1<<ADSC); //start writing 
while(bit_is_clear(ADCSRA, ADIF)){};

ADCSR |= (1 << ADIF);//clear flag by writing one
adc_data = ADCH; // store ADC values


if(lcd_display){
lcd_display = FALSE;
refresh_lcd(lcd_temp_array);
}


//temperature sensor
 twi_start_rd(LM73_ADDRESS, lm73_rd_buf, 2) ; //read temperature data from LM73 (2 bytes) 
  _delay_ms(2);    //wait for it to finish
  lm73_temp =  lm73_rd_buf[0];//save high temperature byte into lm73_temp
  lm73_temp = ( lm73_temp << 8); //shift it into upper byte 
  lm73_temp = lm73_temp | lm73_rd_buf[1]; //"OR" in the low temp byte to lm73_temp 
  lm73_temp = lm73_temp >> 7;
  itoa( lm73_temp, temp_str, 10); //convert to string in array with itoa() from avr-libc                           

  //displaying temperature
for(i = 0; i < strlen(temp_str); i++ ){
	lcd_temp_array[6 + i] = temp_str[i];//insert the temperature characters to the LCD

}

//populate the lcd array to display the new temperature everyone one second 
if(rcv_rdy == 1){
	for(i = 0; i < strlen(uart_buf); i++ ){
		lcd_temp_array[23 + i] = uart_buf[i]; 
	}
	rcv_rdy = 0;
}

switch(mode){
	case 0x40: //Setting alarm
		alarm = TRUE;	//alarm is on
		tone = TRUE;	//turn on tone
		dp = TRUE;	//turn on decimal point
		display = TRUE;	//set to display alarm time
		incr_alarm = 1;	//increment alarm time

//you can set an if statement to change the bounds whether setting == true or false
		alarm_bound_12(); //bound the time for 12hr mark
		break;

	case 0x20: //disable alarm	
		disable_alarm();
		break;

	case 0x08: // setting time
		incr_time = 1;	//increment time
		seconds_holder = 0;	//seconds is not counting
		time_bound_12();	//bound the encoders
		break;
	
	case 0x04://snooze alarm
	if(snooze_time == FALSE){
		snooze_seconds = 0;
		tone = FALSE; //turn off the tone
		OCR3A = 0; // turn off the volume
		TCCR1B &= ~(1 << CS10);	//clear the clock for the tone
		}	
		
	snooze_time = TRUE; //set flag for snooze
		//if snooze flag is initiated then trigger count 
	if(snooze_time){
		if(snooze_seconds == 10){
			snooze_seconds = 0;
			tone = TRUE;
			OCR3A = 5;
			TCCR1B |= 1 << CS10;
		}	
	}
		break;

	default:
		incr_time = 0; 	// encoders should not change time when clock is running
		incr_alarm = 0; // encoders should not change alarm time when clock is running 
		seconds_holder = 1; // increment seconds 
}

time_tracker_12(); //keep track of the time in 12 hr format 



PORTA = 0xFF;  //make PORTA an input port with pullups 	

if((alarm == TRUE) && (mode == 0x40)){
	segsum(alarm_time);
}
else{segsum(time);}

//DIGIT SHIFTS
//bound a counter (0-4) to keep track of digit to display ;

PORTB = 0x00;

	for(int index = 0; index < 5; index++){
		PORTB = index << 4; // within 4 digits for the hex value
		PORTA = segment_data[index]; //send 7 segment code to LED segments
		_delay_ms(1); // add delay	
	}

	//handles decimal point display when alarm is triggered
	if(alarm){
		PORTB = 0; // digit 0
		PORTA = 0x7F; // turn on decimal point
	}
DDRA = 0xFF;  //make PORTA an output
sei(); // ISR will return here

}//while

}//main


//ISR triggers whenever UART receives any data
ISR(USART0_RX_vect){
// stores what it receives into a buffer and once it gets a null character
// let the program know it is received
static uint8_t j = 0;
rx_char = UDR0;
uart_buf[j++] = rx_char;

if(rx_char == '\0'){
	rcv_rdy = 1;
	uart_buf[--j] = (' ');
	uart_buf[j+1] = (' ');
	uart_buf[j+2] = (' ');
	j = 0;
}
}
