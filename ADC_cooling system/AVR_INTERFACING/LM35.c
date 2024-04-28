/*
 * LM35.c
 *
 *  Created on: Nov 9, 2023
 *      Author: OMR
 */

#include"LM35.h"
#include"GPIO.h"
#include"ADC.h"


void LM35_init()
{
	ADC_CFG cfg = {AVCC, RIGHT_ADJUST, PRE_16, SINGLE_CONVERSION};
	ADC_init(&cfg);
}
u8 LM35_getTemperature(void)
{
	/*set lm35 pin as input*/
	DIO_voidSetPinDirection(PERIPHRAL_A, 2, INPUT);

	u16 digital_value;
	ADC_getDigitalValueSynchNonBlocking(CH_2, &digital_value);

	u8 temp = (u8)(((digital_value * REF_VOLT_ms)/MAX_DIGITAL_VALUE)/10);
	return temp;
}

