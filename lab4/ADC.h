#ifndef ADC_H
#define ADC_H


void ADC_init(void){

	ADMUX = (1 << REFS0); // refence to AVCC(5v)

	//ADC enable, Free running mode, interrupts enabled
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | ( 1 << ADPS1) | ( 1 << ADPS0) ; // still need to start it
//	ADCSRA |= (1 << ADSC); //start writing  	
}






#endif
