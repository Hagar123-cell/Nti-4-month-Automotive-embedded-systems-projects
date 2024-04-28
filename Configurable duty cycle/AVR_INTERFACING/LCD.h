/*
 * LCD.h
 *
 *  Created on: Oct 23, 2023
 *      Author: OMR
 */

#ifndef LCD_H_
#define LCD_H_

#include"STDTYPES.h"

/*COMMANDS*/
#define LCD_CLEAR 				0x01
#define LCD_RETURN_H 			0x02
#define ENTRY_MODE				0X06
#define CURSOR_ON_DISPLAY_ON	0X0e

void LCD_void_Init(void);
void LCD_void_sendData(u8 copy_u8data );
void LCD_void_sendCommand(u8 copy_u8command);
void LCD_void_sendString(const s8 * pstr);
void LCD_void_sendIntNum(s32 copy_s32Num);
void LCD_void_gotoXY(u8 copy_u8Row,u8 copy_u8Col);
void LCD_void_creatCustomChar(const u8 * ArrPattern,u8 copy_u8charCode);
void LCD_void_displayCustomChar(u8 copy_u8charCode);
#endif /* LCD_H_ */
