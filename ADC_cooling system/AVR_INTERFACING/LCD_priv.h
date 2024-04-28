/*
 * LCD_priv.h
 *
 *  Created on: Oct 23, 2023
 *      Author: OMR
 */

#ifndef LCD_PRIV_H_
#define LCD_PRIV_H_

#define CHAR_CODES_OFFSET_IN_CGRAM    8
#define DB6                           6
#define TAKE_FIRST_3_BITS             0x07

#define LCD_CGRAM_ADDRESS             0x40

/*DDRAM_ADDRESSES*/
#define LCD_DDRAM_ADDRESS             0x80
#define ADDRESS_ROW_1			0X80
#define ADDRESS_ROW_2			0XC0
#define ADDRESS_ROW_3			0X94
#define ADDRESS_ROW_4			0XD4




static void H_LCD_void_latchByte(u8 copy_u8Byte);


#endif /* LCD_PRIV_H_ */
