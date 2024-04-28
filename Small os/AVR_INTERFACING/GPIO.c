/*
 * GPIO.c
 *
 *  Created on: Oct 20, 2023
 *      Author: OMR
 */
#include"GPIO.h"
void DIO_voidSetPinDirection(GPIO_REGISTERS* ptr, u8 Copy_u8PinId, u8 Copy_u8Direction)
{
	switch(Copy_u8PinId)
	{
	case 0:
		ptr ->direction.bits.bit0 = Copy_u8Direction;
		break;
	case 1:
		ptr ->direction.bits.bit1 = Copy_u8Direction;
		break;
	case 2:
		ptr ->direction.bits.bit2 = Copy_u8Direction;
		break;
	case 3:
		ptr ->direction.bits.bit3 = Copy_u8Direction;
		break;
	case 4:
		ptr ->direction.bits.bit4 = Copy_u8Direction;
		break;
	case 5:
		ptr ->direction.bits.bit5 = Copy_u8Direction;
		break;
	case 6:
		ptr ->direction.bits.bit6 = Copy_u8Direction;
		break;
	case 7:
		ptr ->direction.bits.bit7 = Copy_u8Direction;
		break;
	}
}
void DIO_voidSetPinValue(GPIO_REGISTERS* ptr, u8 Copy_u8PinId, u8 Copy_u8Value)
{
	switch(Copy_u8PinId)
		{
		case 0:
			ptr ->PORT_R.bits.bit0 = Copy_u8Value;
			break;
		case 1:
			ptr ->PORT_R.bits.bit1 = Copy_u8Value;
			break;
		case 2:
			ptr ->PORT_R.bits.bit2 = Copy_u8Value;
			break;
		case 3:
			ptr ->PORT_R.bits.bit3 = Copy_u8Value;
			break;
		case 4:
			ptr ->PORT_R.bits.bit4 = Copy_u8Value;
			break;
		case 5:
			ptr ->PORT_R.bits.bit5 = Copy_u8Value;
			break;
		case 6:
			ptr ->PORT_R.bits.bit6 = Copy_u8Value;
			break;
		case 7:
			ptr ->PORT_R.bits.bit7 = Copy_u8Value;
			break;
		}
}
u8 DIO_voidGetPinValue(GPIO_REGISTERS* ptr, u8 Copy_u8PinId)
{
	u8 pinValue;
	switch(Copy_u8PinId)
		{
	case 0:
				pinValue =  ptr ->PIN_R.bits.bit0 ;
				break;
			case 1:
				pinValue =  ptr ->PIN_R.bits.bit1;
				break;
			case 2:
				return ptr ->PIN_R.bits.bit2;
				break;
			case 3:
				return ptr ->PIN_R.bits.bit3;
				break;
			case 4:
				return ptr ->PIN_R.bits.bit4;
				break;
			case 5:
				pinValue = ptr ->PIN_R.bits.bit5;
				break;
			case 6:
				pinValue = ptr ->PIN_R.bits.bit6;
				break;
			case 7:
				pinValue = ptr ->PIN_R.bits.bit7;
				break;
			}
		return pinValue;
}

void DIO_voidSetPortDirection(GPIO_REGISTERS* ptr, u8 Copy_u8Direction)
{
	ptr ->direction.ALL_BITS = Copy_u8Direction;
}
void DIO_voidSetPortValue(GPIO_REGISTERS* ptr, u8 Copy_u8Value)
{
	ptr ->PORT_R.ALL_BITS = Copy_u8Value;
}
u8 DIO_voidGetPortValue(GPIO_REGISTERS* ptr)
{
	return ptr -> PORT_R.ALL_BITS;
}
