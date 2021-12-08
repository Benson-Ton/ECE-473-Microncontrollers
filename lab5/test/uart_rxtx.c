//UART Example for inclass coding
//Roger Traylor 12.4.12
//Connect two mega128 boards via rs232 and they should end to each
//other a message and a sequence number.
//
//Change the message you send to your partner for checkoff.
//
//You can test this code by a "loopback" if you connect rx to tx
//on the DB9 connector.
//

#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include "uart_functions.h"
#include "hd44780.h"
#include <avr/interrupt.h>
#include <util/delay.h>

#include "twi_master.h"
#include "lm73_functions.h"

//Temperature variables
extern uint8_t lm73_rd_buf[2];
extern uint8_t lm73_wr_buf[2];
char            uart_buf[16];
volatile uint8_t  rcv_rdy;
char              rx_char;

//Remote temperature variables 
int             remote_temp = 10;
char temp_str[32] = {"temp_str" };


uint8_t           i;
volatile uint8_t  rcv_rdy;
char              rx_char; 
char              lcd_str_array[32];  //holds string to send to lcd
uint8_t           send_seq=0;         //transmit sequence number
char              lcd_string[3];      //holds value of sequence number

void spi_init(void){
  DDRB   = DDRB | 0x07;           //Turn on SS, MOSI, SCLK pins
  SPCR  |= (1<<SPE) | (1<<MSTR);  //set up SPI mode
  SPSR  |= (1<<SPI2X);            //run at double speed 
}//spi_init    

int main(){
  DDRF |= 0x08; //lcd strobe bit
  uart_init();  
  spi_init();
  lcd_init();
  init_twi();
  clear_display();
  cursor_home();
uint16_t lm73_temp;
  sei();
  
  while(1){
	  //temperature sensor
 twi_start_rd(LM73_ADDRESS, lm73_rd_buf, 2) ; //read temperature data from LM73 (2 bytes) 
  _delay_ms(2);    //wait for it to finish
  lm73_temp =  lm73_rd_buf[0];//save high temperature byte into lm73_temp
  lm73_temp = ( lm73_temp << 8); //shift it into upper byte 
  lm73_temp = lm73_temp | lm73_rd_buf[1]; //"OR" in the low temp byte to lm73_temp 
  lm73_temp = lm73_temp >> 7;
  itoa( lm73_temp, temp_str, 10); //convert to string in array with itoa() from avr-libc                           
	  //**************  start rcv portion ***************
      if(rcv_rdy==1){
        string2lcd(lcd_str_array);  //write out string if its ready
        rcv_rdy=0;
        cursor_home();
    }//if 
//**************  end rcv portion ***************

//**************  start tx portion ***************
//	uart_puts("hi");
      uart_puts(temp_str);   
	uart_putc('\0');
      /*
      uart_puts("Hi! Cruz: ");
    itoa(send_seq,lcd_string,10);
    uart_puts(lcd_string);
    uart_putc('\0');
    for(i=0;i<=9;i++){_delay_ms(100);}
    send_seq++;
   send_seq=(send_seq%20);
*/
      //**************  end tx portion ***************
  }//while
}//main

ISR(USART0_RX_vect){
static  uint8_t  i;
  rx_char = UDR0;              //get character
  lcd_str_array[i++]=rx_char;  //store in array 
 //if entire string has arrived, set flag, reset index
  if(rx_char == '\0'){
    rcv_rdy=1; 
    lcd_str_array[--i]  = (' ');     //clear the count field
    lcd_str_array[i+1]  = (' ');
    lcd_str_array[i+2]  = (' ');
    i=0;  
  }
}

