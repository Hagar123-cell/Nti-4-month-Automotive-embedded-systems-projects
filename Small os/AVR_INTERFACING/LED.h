/*
 * LED.h
 *
 *  Created on: Nov 16, 2023
 *      Author: OMR
 */

#ifndef LED_H_
#define LED_H_

#include"MCU_HW.h"


void LED_on(GPIO_REGISTERS* reg, u8 pin_num);
void LED_off(GPIO_REGISTERS* reg, u8 pin_num);

#endif /* LED_H_ */
