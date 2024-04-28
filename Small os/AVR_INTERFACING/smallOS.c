/*
 * smallOS.c
 *
 *  Created on: Nov 17, 2023
 *      Author: OMR
 */
#include"smallOS.h"
#include"MCU_HW.h"
#include"PUSH_BUTTON.h"
#include"util/delay.h"
#include"GPIO.h"
void toggleBlueLed(GPIO_REGISTERS* ledPort, u8 ledPin)
{
	ledPort->PORT_R.ALL_BITS ^= 1<<ledPin;
}


void task1()
{

	/*blink red led*/
	//blinkLed(PERIPHRAL_B, 0);
	DIO_voidSetPinValue(PERIPHRAL_B, 0, 1);
	_delay_ms(200);
	DIO_voidSetPinValue(PERIPHRAL_B, 0, 0);
	_delay_ms(200);
}

void task2()
{
	/*toggle blue led*/
	if(DIO_voidGetPinValue(PERIPHRAL_C, 7) == PRESSED)
	{
		//	PERIPHRAL_C->PORT_R.bits.bit0 = 1;
		//	PERIPHRAL_C->PORT_R.
		/*toggle blue led*/
		PERIPHRAL_C->PORT_R.bits.bit0 ^= 1;
		/*if(PERIPHRAL_C->PORT_R.bits.bit0 == 1)
			PERIPHRAL_C->PORT_R.bits.bit0 = 0;
		else if(PERIPHRAL_C->PORT_R.bits.bit0 == 0)
			PERIPHRAL_C->PORT_R.bits.bit0=1;*/
	}
}


void task3()
{
	/*blink green led*/
	DIO_voidSetPinValue(PERIPHRAL_D, 0, 1);
	_delay_ms(200);
	DIO_voidSetPinValue(PERIPHRAL_D, 0, 0);
	_delay_ms(200);
}


static task_t tasks[3]={{60,task1},{30,task2},{180,task3}};

void sechulder(void)
{
	static volatile  u32 c=0;
	c++;
	for (int i=0;i<3;i++)
	{
		if (c%tasks[i].periodicity==0)
		{
			tasks[i].pfun();
		}
	}
	//DIO_voidSetPinValue(PERIPHRAL_B, 0, 1);

	if (c== 180) c=0;
}


