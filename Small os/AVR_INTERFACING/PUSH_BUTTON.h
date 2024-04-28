/*
 * PUSH_BUTTON.h
 *
 *  Created on: Nov 17, 2023
 *      Author: OMR
 */

#ifndef PUSH_BUTTON_H_
#define PUSH_BUTTON_H_

#include"MCU_HW.h"

typedef enum
{
	 RELEASED, PRESSED
}PB_state;

PB_state PB_getButtonState(GPIO_REGISTERS* buttonPort, u8 buttonPin);
#endif /* PUSH_BUTTON_H_ */
