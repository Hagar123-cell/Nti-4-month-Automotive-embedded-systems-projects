/*
 * LCD_cfg.h
 *
 *  Created on: Oct 23, 2023
 *      Author: OMR
 */

#ifndef LCD_CFG_H_
#define LCD_CFG_H_

#include"MCU_HW.h"
#define LCD_MODE    _4_BIT_MODE


#define _4_BIT_MODE		4
#define FNC_SET_FOUR_BIT_MODE   0x28

#define _8_BIT_MODE		 8
#define FNC_SET_EIGHT_BIT_MODE   0x38



#define PIN0  0
#define PIN1  1
#define PIN2  2
#define PIN3  3
#define PIN4  4
#define PIN5  5
#define PIN6  6
#define PIN7  7

#define LCD_PIN4_PORT PERIPHRAL_A
#define LCD_PIN5_PORT PERIPHRAL_A
#define LCD_PIN6_PORT PERIPHRAL_A
#define LCD_PIN7_PORT PERIPHRAL_A



#define LCD_PIN4_PIN PIN3
#define LCD_PIN5_PIN PIN2
#define LCD_PIN6_PIN PIN1
#define LCD_PIN7_PIN PIN0

#define RS_PORT   PERIPHRAL_A
#define RS_PIN   PIN6

#define EN_PORT   PERIPHRAL_A
#define EN_PIN   PIN5

#endif /* LCD_CFG_H_ */
