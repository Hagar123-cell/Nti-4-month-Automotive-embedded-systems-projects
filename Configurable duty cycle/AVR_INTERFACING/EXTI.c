/*
 * EXTI.c
 *
 *  Created on: Oct 22, 2023
 *      Author: OMR
 */

#include "EXTI.h"
void(*ptr_to_callback_fns[3])(void) = {NULL, NULL, NULL};

void EXTI_enableInterrupt(INT_ID INT, triggerType trig)
{
	/*enable interrupt*/
	EXTI_R ->GICR.ALL_BITS |= (1<<INT);

	switch(INT)
	{
	case INT0:
		/*choose triggering type*/
		EXTI_R ->MCUCR.ALL_BITS |= trig;
		break;
	case INT1:
		/*choose triggering type*/
		EXTI_R ->MCUCR.ALL_BITS |= trig<<2;
		break;
	case INT2:
		/*choose triggering type*/
		EXTI_R ->MCUCSR.bits.bit6 = trig; /***************/
		break;
	}


}

void EXTI_disableInterrupt(INT_ID INT)
{
	EXTI_R ->GICR.ALL_BITS &= ~(1<<INT);
}
void EXTI_setCallBack(void(*ptr_to_fn)(void), INT_ID INT)
{
	ptr_to_callback_fns[INT-5] = ptr_to_fn;/*********************/
}
