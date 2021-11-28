// lab1_code.c 
// R. Traylor
// 7.13.20
//
// Modified for Lab1
// By: Benson Ton
// Date: 09/26/21

//This program increments a binary display of the number of button pushes on switch 
//S0 on the mega128 board.

#include <avr/io.h>
#include <util/delay.h>

//*******************************************************************************
//                            debounce_switch                                  
// Adapted from Ganssel's "Guide to Debouncing"            
// Checks the state of pushbutton S0 It shifts in ones till the button is pushed. 
// Function returns a 1 only once per debounced button push so a debounce and toggle 
// function can be implemented at the same time. Expects active low pushbutton on 
// Port D bit zero. Debounce time is determined by external loop delay times 12. 
//*******************************************************************************
int8_t debounce_switch() {
  static uint16_t state = 0; //holds present state
  state = (state << 1) | (! bit_is_clear(PIND, 0)) | 0xE000;
  if (state == 0xF000) return 1;
  return 0;
}

//*******************************************************************************
//			debounce_switch_dec
// Similar to the debounce_switch function except when the state of the pushbutton S1
//  Shifts current state by one then checks if S1 is cleared and if it is invert it
//  OR with 0xE00 to save the current state of other bits
//*******************************************************************************
int8_t debounce_switch_dec(){
 static uint16_t state = 0; //holds present state
 state = (state << 1) | (! bit_is_clear(PIND, 1)) | 0xE000;
 if(state == 0xF000) return 1;
 return 0;
}

//*******************************************************************************
// Check switch S0.  When found low for 12 passes of "debounce_switch(), increment
// PORTB. This will make an incrementing count on the port B LEDS. 
//*******************************************************************************
int main()
{
DDRB = 0xFF;  //set port B to all outputs
DDRD = 0x00; //set port D as input mode


while(1){     //do forever

 if(debounce_switch()) {PORTB++;};  //if switch true for 12 passes, increment port B
 if(debounce_switch_dec()) {PORTB--;}  //decrement port B and keep in loop for debounce 24ms

 _delay_ms(2);

} //while



} //main

//test save

