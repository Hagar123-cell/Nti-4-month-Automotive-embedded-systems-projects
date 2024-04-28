/*
 * PUSH_BUTTON.c
 *
 *  Created on: Nov 17, 2023
 *      Author: OMR
 */

#include"PUSH_BUTTON.h"
#include"MCU_HW.h"


PB_state PB_getButtonState(GPIO_REGISTERS* buttonPort, u8 buttonPin)
{
	u8 button;
	switch(buttonPin)
	{
	case 0:
		button=buttonPort->PIN_R.bits.bit0;
		break;

	case 1:
		button=buttonPort->PIN_R.bits.bit1;
		break;
	case 2:
		button=buttonPort->PIN_R.bits.bit2;
		break;
	case 3:
		button=buttonPort->PIN_R.bits.bit3;
		break;

	case 4:
		button=buttonPort->PIN_R.bits.bit4;
		break;
	case 5:
		button=buttonPort->PIN_R.bits.bit5;
		break;
	case 6:
		button=buttonPort->PIN_R.bits.bit6;
		break;
	case 7:
		button=buttonPort->PIN_R.bits.bit7;
		break;
	}
	return button;


}

