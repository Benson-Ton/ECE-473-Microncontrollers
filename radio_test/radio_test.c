#include <avr/io.h>
#include "si4734_driver/si4734.h"
#include "twi_master/twi_master.h"
#include <avr/interrupt.h>
#include <util/delay.h>


volatile uint8_t STC_interrupt;     //indicates tune or seek is done
enum radio_band{FM, AM, SW};
volatile enum radio_band current_radio_band;

volatile uint16_t  current_fm_freq; //0x2706, arg2, arg3; 99.9Mhz, 200khz steps


uint16_t eeprom_fm_freq;
uint16_t eeprom_am_freq;
uint16_t eeprom_sw_freq;
uint8_t  eeprom_volume;

//uint16_t current_fm_freq;
uint16_t current_am_freq;
uint16_t current_sw_freq;
uint8_t  current_volume;

uint8_t  si4734_wr_buf[9];
uint8_t  si4734_rd_buf[9];
uint8_t  si4734_tune_status_buf[8];

//write
void write_SPI(uint8_t value){

SPDR = value; // take in which mode it is currently on and display it
        while (bit_is_clear(SPSR,SPIF)) {} //wait till data is sent out
PORTD |= (1 << PD2); //SEND data to bargraph, rising edge
PORTD &= ~(1<<PD2); // falling edge
}


//ISR
ISR(INT7_vect){STC_interrupt = TRUE;

//	write_SPI(1 << 5);

}





//********************************************************************
//                            spi_init                               
//Initalizes the SPI port on the mega128. Does not do any further    
// external device specific initalizations.                          
//********************************************************************
void spi_init(void){
  DDRB |=  0x07;  //Turn on SS, MOSI, SCLK
  //mstr mode, sck=clk/2, cycle 1/2 phase, low polarity, MSB 1st, 
  //no interrupts, enable SPI, clk low , rising edge sample
  SPCR=(1<<SPE) | (1<<MSTR);
  //SPSR=(1<<SPI2X); //SPI at 2x speed (8 MHz)  
//DDRB |= 0xF0; // set PORTB 4-7 bits as outputs
//DDRA = 0x03; // set PORTA 6-7 bits as inputs
//DDRD = (1 << PD2); // set PD2 as output


}//spi_init


void volume_init(void){
//volume pin is PE3
TCCR3A = (1 << COM3A1 | (1 << WGM31) ); //normal operation w/ OC3A is disconnected
TCCR3B |= (  (1 << CS30) | ( 1 << WGM32) | (1 << WGM33) ); //fast pwm 
ICR3 = 10; // Set overflow top ; controls frequency 
OCR3A = 7; // set compare match; duty cycle

}




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




int main()
{
//spi_init();
init_twi();
volume_init();
uart_init();
sei();

//initializing timer/counter0 (TCNT0) 
//TIMSK |= (1<<TOIE0);    //enable interrupts
//TCCR0 |= (1 <<CS02) | (1<<CS00); //normal mode, prescale by 128


//interrupts configuration
EICRB |= (1 << ISC71) | (1 << ISC70);
EIMSK |= 1 << INT7;


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

//volatile uint16_t current_fm_freq =  9990; //0x2706, arg2, arg3; 99.9Mhz, 200khz steps


/*
volatile uint16_t  current_fm_freq =  9990; //0x2706, arg2, arg3; 99.9Mhz, 200khz steps
uint8_t  si4734_wr_buf[9];
uint8_t  si4734_rd_buf[9];
uint8_t  si4734_tune_status_buf[8];
*/
//volatile uint8_t STC_interrupt;     //indicates tune or seek is done


//configuring the Radio 
//PORTE |= 

radio_reset();


//	write_SPI(1 << 5);
//while(1){
//Once its setup, you can set the station and get the received signal strength.

fm_pwr_up();        //power up radio
_delay_ms(1);
while(twi_busy()){} //spin while TWI is busy 
current_fm_freq = 9990; //99.9 good
//current_fm_freq = 9910; //good
//current_fm_freq = 10053;

//current_fm_freq = 10075;//107.5
//current_fm_freq = 10053;
//current_fm_freq = 10063;

_delay_ms(.1);


fm_tune_freq();     //tune to frequency   

//	}

}
