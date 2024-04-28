/*
 * ADC_private.h
 *
 * Created: 10/23/2023 9:16:54 AM
 *  Author: OMR
 */ 


#ifndef ADC_PRIVATE_H_
#define ADC_PRIVATE_H_

#include"STDTYPES.h"
/*ADC Registers*/ 


/*ADMUX bits*/
typedef enum{
	MUX0,MUX1,MUX2,MUX3,MUX4,ADLAR,REFS0,REFS1
}ADMUX_bits;
	
/*ADCSRA bits*/
enum{
	ADPS0,ADPS1,ADPS2,ADIE,ADIF,ADATE,ADSC,ADEN
}ADCSRA_bits;

/*SFIOR bits*/
enum{
	ADTS0 = 5, ADTS1=6, ADTS2=7
}SFIOR_bits;


#endif /* ADC_PRIVATE_H_ */
