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

#define FALSE 0
#define  TRUE 1

#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include "uart_functions.h"
#include "hd44780.h"
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lm73_functions.h"
#include "twi_master.h"

//master will send "R" and compare it; no need for strcmp


extern uint8_t lm73_rd_buf[2];
extern uint8_t lm73_wr_buf[2];



char test[16]; 
int transmit = FALSE;
uint8_t           i;
volatile uint8_t  rcv_rdy;
char              rx_char; 
char              lcd_str_array[16];  //holds string to send to lcd
uint8_t           send_seq=0;         //transmit sequence number
char              lcd_string[3];      //holds value of sequence number
char 		  uart_buf[16];

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
  clear_display();
  cursor_home();
  init_twi();

  sei();

uint16_t lm73_temp;  //a place to assemble the temperature from the lm73


lm73_wr_buf[0] = LM73_PTR_TEMP; //load lm73_wr_buf[0] with temperature pointer address

twi_start_wr(LM73_ADDRESS, lm73_wr_buf, 1 ); //start the TWI write process
_delay_ms(2);    //wait for the xfer to finish

//clear_display(); //clean up the display



  while(1){
//test = uart_getc();
//if(test[0] == "R"){transmit = TRUE;}
//else{transmit = FALSE;}

 /************** reading temperature *************/
//  _delay_ms(100); //tenth second wait
 // clear_display(); //wipe the display
 twi_start_rd(LM73_ADDRESS, lm73_rd_buf, 2) ; //read temperature data from LM73 (2 bytes) 
  _delay_ms(2);    //wait for it to finish
  lm73_temp =  lm73_rd_buf[0];//save high temperature byte into lm73_temp
  lm73_temp = ( lm73_temp << 8); //shift it into upper byte 
  lm73_temp = lm73_temp | lm73_rd_buf[1]; //"OR" in the low temp byte to lm73_temp 
  lm73_temp = lm73_temp >> 7;
  itoa( lm73_temp, lcd_str_array, 10); //convert to string in array with itoa() from avr-libc                           
_delay_ms(100);
	  
	  
	  
	  
//**************  start rcv portion ****************
  //    if(rcv_rdy==1){
       // string2lcd(lcd_str_array);  //write out string if its ready
    //    rcv_rdy=0;
      //  cursor_home();
//test = uart_getc();
  //  }//if 
//**************  end rcv portion ***************

//reads in the temperature from TWI converts it to F then sends it to Master

//**************  start tx portion ***************


//if(transmit == TRUE){  
//  uart_puts("99");
   // itoa(send_seq,lcd_str_array,10);
   // uart_puts(lcd_str_array);
   // uart_putc('\0');
    //for(i=0;i<=9;i++){_delay_ms(100);}
//    send_seq++;
  //  send_seq=(send_seq%20);
//}
//**************  end tx portion ***************
  }//while
}//main

ISR(USART0_RX_vect){
//char temp = uart_getc();
rx_char = UDR0;              //get character
     uart_puts(lcd_str_array);
    uart_putc('\0');
 //   for(i=0;i<=9;i++){_delay_ms(100);}
/*
if(!rcv_rdy){
	static  uint8_t  i;
rx_char = UDR0;              //get character
  uart_buf[i++]=rx_char;  //store in array 
 //if entire string has arrived, set flag, reset index
  if(rx_char == '\0'){
    rcv_rdy=1; 
    uart_buf[--i]  = (' ');     //clear the count field
    uart_buf[i+1]  = (' ');
    uart_buf[i+2]  = (' ');
    i=0;  
  }
  else{
	i = 0;
	  while(uart_buf[i] != 0){
	test[i] = uart_buf[i];
      	rcv_rdy = 0;
	i++;	
	}
}	
}
*/
}
//************************************//

