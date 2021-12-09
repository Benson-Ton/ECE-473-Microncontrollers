// final project
// Benson Ton
//implented master code

//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
//  PORTB bit 7 goes to the PWM transistor base.

//#define TRUE 1
//#define FALSE 0


#include <string.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#include "si4734_driver/si4734.h"
#include "twi_master/twi_master.h"
#include "uart_functions.h" 
#include "lm73_functions.h"
#include "hd44780.h"

//GLOBAL VARIABLES
uint16_t 		i;
volatile uint16_t	flag = 0;
volatile int	 	display_flag = 0;

//RADIO Variables
volatile uint8_t STC_interrupt;     //indicates tune or seek is done
enum radio_band{FM, AM, SW};
volatile enum radio_band current_radio_band;
int radio = FALSE;
uint8_t radio_status = FALSE;
uint8_t radio_on_off = FALSE;
uint8_t radio_alarm_flag = FALSE;

//EEPROM Radio 
uint16_t eeprom_fm_freq;
uint16_t eeprom_am_freq;
uint16_t eeprom_sw_freq;
uint8_t  eeprom_volume;

//Frequency
uint16_t current_am_freq;
uint16_t current_sw_freq;
uint8_t  current_volume;
uint8_t incr_freq;

//radio station examples
//10470 -> 104.7
//9990 -> 99.0
uint16_t  current_fm_freq; //0x2706, arg2, arg3; 99.9Mhz, 200khz steps
volatile uint16_t  fm_freq = 9990;

//TWI Radio Variables
uint8_t  si4734_wr_buf[9];
uint8_t  si4734_rd_buf[9];
uint8_t  si4734_tune_status_buf[8];

//SETTING USER INTERFACE CONFIGURATIONS
int sound = FALSE;	//default 0 -> ALARM tone and 1 -> Radio tone
int volume = 0;

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

ISR(INT7_vect){STC_interrupt = TRUE;}



/********************************************************************
 *				alarm_compare
 *******************************************************************/
void alarm_compare(void){
	//checks alarm flag and whether button is pressed
if(tone){
	//
	if(( mode != 0x08) && (mode != 0x40 )){  
		//triggers the alarm if alarm time matches with real time 
		if( (mode != 0x48) && ( radio_alarm_flag != TRUE)){
			if((alarm_time == time) && (sound == FALSE) && (radio == FALSE)){
			TCCR1B |= 1 << CS10; //turn on alarm
			OCR3A = 8;  //set volume
			}
		}
		
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
	if(val_rot == 1){ 
		time += (100*incr_time); //increment time
		if(radio){fm_freq += incr_freq;} // increment frequency
		alarm_time += (100*incr_alarm); // increment alarm
	} //if rotating to the right for left encoder then increment 
	
	else if(val_rot ==0){
		time -= (100*incr_time); //increment time
		if(radio){fm_freq -= incr_freq;} // increment frequency
		alarm_time -= (100*incr_alarm); // increment alarm
	}//if rotating to the left for left encoder then decrement
	
	else if(val_rot == 2){
		time -= incr_time; //increment time
		alarm_time -= incr_alarm; // increment frequency
		display_flag = TRUE; // increment alarm
		if((mode != 0x08) && (mode != 0x40)){volume -= 1;}
	} // if rotating to the left for right encoder then decrement
	
	else if(val_rot == 3){
		time += incr_time; //increment time
		alarm_time += incr_alarm; // incrememt frequency
		display_flag = TRUE; // icrement alarm
		if((mode != 0x08) && (mode != 0x40)){volume += 1;}
	}// if rotatiing to the right for right encoder then increment
	
	else{val_rot =-1;} // dont do anything if nothing is rotated
}

/********************************************************************
 *				detect_volume
 ********************************************************************/
void detect_volume(uint8_t value){
//right encoder movement
if(value == 2){
	display_flag = TRUE; // set flag to display volume
	volume -= 1;
} //left
else if(value == 3){
	display_flag = TRUE; //set flag to display volume
	volume += 1;
} //right



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

/********************************************************************
 *				volume_bound
 ********************************************************************/
void volume_bound(void){

if(volume > 10){volume = 10;}
if(volume < 0){volume = 0;}

}

/********************************************************************
 *	 		TIMER_COUNTER1 COMPARE ISR
 ********************************************************************/
ISR(TIMER1_COMPA_vect){

	DDRE |= 1 << PE3; // PE1 will output for volume
	PORTC ^= 1 << PC0; // PC0 will toggle tone
}


/********************************************************************
 *	 		TIMER_COUNTER2 COMPARE ISR
 *dimming for LED display 
 ********************************************************************/
ISR( TIMER2_COMP_vect){	
	OCR2 = adc_data; //update dimming with CdS readings
}


/********************************************************************
 *	 		TIMER_COUNTER0 COMPARE ISR
 *Over timing of the real clock and checks for buttons and encoders movement 
 ********************************************************************/
ISR( TIMER0_COMP_vect ) {

//save the states of PORTA and PORTB
saveA = PORTA;
saveB = PORTB;

refresh_lcd(lcd_temp_array); //display the interface with the temperature and alarms

alarm_compare(); //checks if the alarm set time and the clock time matches then trigger alarm

if(current_fm_freq != fm_freq){flag = 1;} // if the frequency is changed then set tune flag

static uint16_t timer = 0;  //hold value of count between interrupts
timer++;  //extend counter

if((timer% 128) == 0){ // turn off for half a second 
	segment_data[2] = 0x07; //turn off colon
} 

if((timer% 512) == 0){
	uart_puts(temp_str);		//send a string through the UART
	uart_putc('\0');		//ADD a null character
	flag = 1;			//set flag to tune frequency

		if( (radio != TRUE) && (display_flag != TRUE)){segment_data[2] = 0x0C;}//turn on colon
 
	seconds+= seconds_holder; 	//increment seconds
	snooze_seconds += seconds_holder;//increment seconds of snooze
}

if((timer%2058) == 0){if(display_flag){display_flag = FALSE;}} //display the volume for about 2s

PORTB = 0x50;  //enable tristate buffer for pushbutton switches
_delay_ms(.1);//need a delay to active buffer


if(chk_buttons(0)){mode ^= 1 << 6;}//sets alarm, 0x40
if(chk_buttons(6)){mode ^= 1 << 1;}//set tone to be alarm or radio (default: ALARM), 0x02
if(chk_buttons(7)){mode ^= 1 << 4;}// sets Radio frequency, 0x10
if(chk_buttons(2)){mode ^= 1 << 5;}//disable alarm, 0x20
if(chk_buttons(3)){mode ^= 1 << 2;}//snooze alarm, 0x04
if(chk_buttons(1)){mode ^= 1 << 3;} //sets time, 0x08
	
PORTB = 0x70;//disable tristate


read_SPI();		//read in from the SPI
read = process_EN(); 	// decrypt the data from the SPI and determine the encoder movement
update_EN(read);	// increase the time with the encoder movement 
volume_bound();         //bound the volume from 0 - 10
write_SPI(mode); 	// write to the bar graph

static uint8_t index = 0;
PORTB = 0x00;			//start at digit 0
PORTB = index << 4;		//within 4 digits for the hex value
PORTA = segment_data[index]; 	//send 7 segment code to LED segments
_delay_ms(.8); 			// add delay	
index++;			//increase to the next digit

if(index == 5){index = 0;}	//set index from 0-4

//restore the state when leaving the ISR
PORTA = saveA;
PORTB = saveB;

//have the colon flash every one second 
      if(alarm){
              PORTB = 0; // digit 0
              PORTA = 0x7F; // turn on decimal point
      }


DDRA = 0xFF; 		//set PORTA to all outputs


}//end of ISR Timer0




/********************************************************************
 *				disable_alarm
 *******************************************************************/
void disable_alarm(void){
		sound = FALSE;
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
		TCCR1B &= ~(1 << CS10);	//clear the clock
		mode &= ~(1 << 5); // set conditions only once so clear the bit
}

/********************************************************************
 *				freq_bound
 * used to bound the radio with respect to the frequencies when setting
 * the frequency with the encoders.
 ********************************************************************/
void freq_bound(void){
//might need to put them in 32 bit array and connect them. They are incrementing by 20? 


//88.1 -> 100.8
if(fm_freq > 10800){fm_freq = 8810;};

//108.1 -> 88.1
if(fm_freq < 8800){fm_freq = 10810;}

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

DDRF |= 0x08; //lcd strobe bit
DDRB |= 0xF0; // set PORTB 4-7 bits as outputs
DDRA = 0x03; // set PORTA 6-7 bits as inputs
DDRD = (1 << PD2); // set PD2 as output
//DDRE = 0xFF;// set PORTE to all outputs
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
//OCR1A = 3999; // 4kHz and triggers ISR
OCR1A = 3999;
//OCR1A = 18517;
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
 *				radio_init
 **********************************************************************/
void radio_init(void){

// external interrupts configuration
EICRB |= (1 << ISC71) | (1 << ISC70); //set on rising edge for external interrupt
EIMSK |= 1 << INT7;//enable mask for bit 7


//Port E inital values and setup.  This may be different from yours for bits 0,1,6.

//                           DDRE:  0 0 0 0 1 0 1 1
//   (^ edge int from radio) bit 7--| | | | | | | |--bit 0 USART0 RX
//(shift/load_n for 74HC165) bit 6----| | | | | |----bit 1 USART0 TX
//                           bit 5------| | | |------bit 2 (radio reset, active high)
//                  (unused) bit 4--------| |--------bit 3 (TCNT3 PWM output for volume control)

DDRE  |= 0x04; //Port E bit 2 is active high reset for radio 
DDRE  |= 0x40; //Port E bit 6 is shift/load_n for encoder 74HC165
DDRE  |= 0x08; //Port E bit 3 is TCNT3 PWM output for volume


PORTE |= 0x04; //radio reset is on at powerup (active high), PE2
PORTE |= 0x40; //pulse low to load switch values, else its in shift mode, PE7
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
if(sum == 0){
	segment_data[digit_index] = dec_to_7seg[0];
	digit_index++;
}

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

/**********************************************************************
 *                              temp_display
 *********************************************************************/
void temp_display(void){

 //display temperature locally
for(i = 0; i < strlen(temp_str); i++ ){
	lcd_temp_array[6 + i] = temp_str[i];//insert the temperature characters to the LCD

}

//populate the lcd array to display the new temperature everyone one second 
if(rcv_rdy == 1){
	//display temperature remotely 
	for(i = 0; i < strlen(uart_buf); i++ ){
		lcd_temp_array[23 + i] = uart_buf[i]; 
	}
	rcv_rdy = 0;
}

}

/**********************************************************************
 *                              temp_sens
 *communication to the temperature sensor through TWI 
 *********************************************************************/
void temp_sens(void){
uint16_t lm73_temp;  //a place to assemble the temperature from the lm73
//temperature sensor TWI communications
 twi_start_rd(LM73_ADDRESS, lm73_rd_buf, 2) ; //read temperature data from LM73 (2 bytes) 
  _delay_ms(2);    //wait for it to finish
  lm73_temp =  lm73_rd_buf[0];//save high temperature byte into lm73_temp
  lm73_temp = ( lm73_temp << 8); //shift it into upper byte 
  lm73_temp = lm73_temp | lm73_rd_buf[1]; //"OR" in the low temp byte to lm73_temp 
  lm73_temp = lm73_temp >> 7;
  itoa( lm73_temp, temp_str, 10); //convert to string in array with itoa() from avr-libc                           
}

/**********************************************************************
 *                              lcd_radio
 *display radio on lcd  
 *********************************************************************/

void lcd_radio(void){

//If the user wants to use radio as the alarm 
	char temp[32] = "RADIO";

	if(mode == 0x10){

		//populate the lcd screen on the top right with "RADIO"
		for(i = 0; i < strlen(temp); i++){
			lcd_temp_array[27+i] = temp[i];
		}
	}
}
/**********************************************************************
 *                              radion_on
 *power sequence and tuning for the radio
 *********************************************************************/
void radio_on(void){
if( ((radio_status == FALSE) && (mode == 0x10)) | (radio_alarm_flag)){
        fm_pwr_up();        	//power on radio
        _delay_ms(1);
        while(twi_busy()){} 	//spin while twi is busy
	radio_status = TRUE;    
	flag = 1;		//set tune flag
}

if( (((mode == 0x10) && (flag == 1)) && (radio == TRUE)) | (radio_alarm_flag)){
 	OCR3A = volume;			//set volume
	if(radio_alarm_flag){OCR3A = 6;} // if the alarm is on then set volume
  	current_fm_freq = fm_freq;	//set the frequency
	fm_tune_freq();     		//tune to frequency
   flag = 0;
}
}
/**********************************************************************
 *                              radio_reset
 * reset the radio for TWI config
 *********************************************************************/

void radio_reset(void){
PORTE &= ~(1<<PE7); //int2 initially low to sense TWI mode
DDRE  |= 0x80;      //turn on Port E bit 7 to drive it low
PORTE |=  (1<<PE2); //hardware reset Si4734 
_delay_us(200);     //hold for 200us, 100us by spec         
PORTE &= ~(1<<PE2); //release reset 
_delay_us(30);      //5us required because of my slow I2C translators I suspect
                    //Si code in "low" has 30us delay...no explaination given
DDRE  &= ~(0x80);   //now Port E bit 7 becomes input from the radio interrupt
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


radio_init(); //initlize radio ports
radio_reset();//reset radio on TWI 

while(1){ // main loop


ADCSR |= (1<<ADSC); //start writing 
while(bit_is_clear(ADCSRA, ADIF)){};

ADCSR |= (1 << ADIF);//clear flag by writing one
adc_data = ADCH; // store ADC values

temp_sens();	//Commuincate the temp sensor through TWI:W
temp_display();	//display the remote and local temperatures


switch(mode){
	case 0x40: //Setting alarm
		alarm = TRUE;		//alarm is on
		sound = FALSE;  	// regular alarm tone is defaulted
		tone = TRUE;		//turn on tone
		dp = TRUE;		//turn on decimal point
		incr_alarm = 1;		//increment alarm time
		alarm_bound_12(); 	//bound the time for 12hr mark
		break;

	case 0x02://Set ALARM sound or RADIO sound
		sound = TRUE;//radio sound on	
		break;

	case 0x42://when setting alarm time with radio setting is set 
		sound = TRUE;
		incr_alarm = 1;
		alarm_bound_12();
		break;

	case 0x10: //setting radio
		radio = TRUE; 	//radio is on
		radio_on(); 	//set up radio 
		incr_freq = 20; // set increments for frequencies
		freq_bound();	//bound the frequencies
		lcd_radio();
		break;

	case 0x20: //disable alarm	
		disable_alarm();
		mode &= ~(0x20);
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
			//turn on the alarm after snooze time
			OCR3A = 5;
			TCCR1B |= 1 << CS10;
		}	
	}
		break;

	default:
		incr_time = 0; 	// encoders should not change time when clock is running
		incr_alarm = 0; // encoders should not change alarm time when clock is running 
		seconds_holder = 1; // increment seconds 
		OCR3A = volume;	// mute the sound of both alarm and radio
		radio = FALSE;	// radio is defaulted as off
		//if the radio is on and not in radio mode then turn off
		if(radio_status){
			radio_pwr_dwn();
			radio_status = FALSE;;
		} //power down the radio if it was previously on
		
		//clear the radio lcd
		for(i = 0; i < strlen(alarm_str); i++ ){
			lcd_temp_array[27 + i] = ' ';
		}

}

time_tracker_12(); //keep track of the time in 12 hr format 



PORTA = 0xFF;  //make PORTA an input port with pullups 	

//dont display volume when changing time or alarm
if(mode == 0x40){display_flag = FALSE;} 
if(mode == 0x08){display_flag = FALSE;}

//change LED to alarm time
if( ((alarm == TRUE) && (mode == 0x40)) | (mode == 0x42)){
	segsum(alarm_time);
}
//change LED to volume
else if( display_flag ){
	if( (display_flag) && (mode == 0x10)){segsum(volume);}
	if(display_flag){segsum(volume);}
}
//change LED to frequency
else if((radio == TRUE) && (mode != 0x08)){
	current_fm_freq = fm_freq;
	segsum(current_fm_freq/10);
}
//change LED to real clock
else{segsum(time);}

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

//if the NULL terminator is receive then start clearing the buffer
if(rx_char == '\0'){
	rcv_rdy = 1;
	uart_buf[--j] = (' ');
	uart_buf[j+1] = (' ');
	uart_buf[j+2] = (' ');
	j = 0;
}



}

