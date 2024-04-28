/*
 * LED.c
 *
 *  Created on: Nov 16, 2023
 *      Author: OMR
 */
#include"LED.h"
#include"GPIO.h"

void LED_on(GPIO_REGISTERS* reg, u8 pin_num)
{
	DIO_voidSetPinDirection(reg, pin_num, OUTPUT);
	DIO_voidSetPinValue(reg, pin_num, LOGIC_HIGH);
}
void LED_off(GPIO_REGISTERS* reg, u8 pin_num)
{
	DIO_voidSetPinDirection(reg, pin_num, OUTPUT);
	DIO_voidSetPinValue(reg, pin_num, LOGIC_LOW);

}
