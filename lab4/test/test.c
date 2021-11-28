#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "hd44780.h"
#include <string.h>
ISR(TIMER1_COMPA_vect)
{
	PORTC ^= 1<<PC7;

}

//ISR(TIMER3_COMPA_VECT){
//if(alarm == TRUE){PORTE ^= 1 << PE3;}
//PORTE ^= 1 << PE3;
//}

void volume_init(void){
DDRE = 0xFF;
//PORTE = 1 << PE3;
//DDRE = 0xFF;
TCCR3A = (1 << COM3A1 | (1 << WGM31) ); //normal operation w/ OC3A is disconnected
TCCR3B |= (  (1 << CS30) | ( 1 << WGM32) | (1 << WGM33) ); //fast pwm 
//ETIMSK |= (1 << OCIE3A);
//TCCR3A = 0x00;
ICR3 = 10;
OCR3A = 5;
}


uint8_t main()
{
	volume_init();
	DDRB |= 0x07; // testing lcd

	SPCR = (1<<SPE) | (1<<MSTR);//initialize spi and mstr mode
	lcd_init();//initialize lcd
	clear_display(); // clear 
	uint2lcd(8);//display 8

	DDRC = 0xFF;
	TCCR1A = 0x00;
	TIMSK |= 1<<OCIE1A;
	//OCR1A = 3999; //a note, first note then trigger isr	
//	OCR1A = 2000;
	//c_note()
//b_note()
//trigger interrupt
	TCCR1B |= (1<<WGM12 | 1<<CS10);
	sei();
	while (1){}


}

