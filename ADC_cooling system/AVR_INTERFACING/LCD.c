/*
 * LCD.c
 *
 *  Created on: Oct 23, 2023
 *      Author: OMR
 */

#include"LCD.h"
#include"LCD_priv.h"
#include"LCD_cfg.h"
#include"GPIO.h"
#include"util/delay.h"
#include<string.h>
#include"BIT_MATH.h"

void LCD_void_Init(void)
{
#if(LCD_MODE== _4_BIT_MODE)
	DIO_voidSetPinDirection(LCD_PIN4_PORT, LCD_PIN4_PIN, 1);
	DIO_voidSetPinDirection(LCD_PIN5_PORT, LCD_PIN5_PIN, 1);
	DIO_voidSetPinDirection(LCD_PIN6_PORT, LCD_PIN6_PIN, 1);
	DIO_voidSetPinDirection(LCD_PIN7_PORT, LCD_PIN7_PIN, 1);
	DIO_voidSetPinDirection(EN_PORT, EN_PIN, 1);
	DIO_voidSetPinDirection(RS_PORT, RS_PIN, 1);

	DIO_voidSetPinValue(RS_PORT, RS_PIN, 0);


	_delay_ms(100);
	LCD_void_sendCommand(LCD_RETURN_H);
	_delay_ms(10);
	LCD_void_sendCommand(FNC_SET_FOUR_BIT_MODE);
	_delay_ms(1);
	LCD_void_sendCommand(CURSOR_ON_DISPLAY_ON);
	_delay_ms(1);
	LCD_void_sendCommand(LCD_CLEAR);
	_delay_ms(10);
	LCD_void_sendCommand(ENTRY_MODE);
	_delay_ms(1);

#elif(LCD_MODE== _8_BIT_MODE)
	{
		_delay_ms(100);
		LCD_void_sendCommand(FNC_SET_EIGHT_BIT_MODE);
		_delay_ms(1);
		LCD_void_sendCommand(CURSOR_ON_DISPLAY_ON);
		_delay_ms(1);
		LCD_void_sendCommand(LCD_CLEAR);
		_delay_ms(10);
		LCD_void_sendCommand(ENTRY_MODE);
		_delay_ms(1);
	}
#endif
}



void LCD_void_sendData(u8 copy_u8data )
{
	DIO_voidSetPinValue(RS_PORT, RS_PIN, 1);
	H_LCD_void_latchByte(copy_u8data);
	_delay_ms(1);

}
void LCD_void_sendCommand(u8 copy_u8command)
{
	DIO_voidSetPinValue(RS_PORT, RS_PIN, 0);
	H_LCD_void_latchByte(copy_u8command);
	_delay_ms(1);

	/**********************/
}
void LCD_void_sendString(const s8 * pstr)
{

	int i=0;
	while(pstr[i] != '\0')
	{
		LCD_void_sendData(pstr[i]);
		i++;
	}
}

void LCD_void_sendIntNum(s32 copy_s32Num)
{

	if(copy_s32Num == 0)
	{
		LCD_void_sendData('0');
		return;
	}

	else if(copy_s32Num < 0)
	{
		LCD_void_sendData('-');
		copy_s32Num *= -1;
		_delay_ms(0.5);
	}

	u8 num_digits[10]={0};
	u8 i;

	for( i=0; (copy_s32Num)!=0;i++)
	{
		num_digits[i] = (copy_s32Num % 10);
		copy_s32Num /= 10;
	}
	int j;
	for (j=i-1;j>=0;j--)
	{
		LCD_void_sendData(num_digits[j]+'0');
	}



}

void LCD_void_gotoXY(u8 copy_u8Row,u8 copy_u8Col)
{
	u8 copy_u8command;

	if(copy_u8Row>4||copy_u8Row<1||copy_u8Col>20||copy_u8Col<1)
	{
		copy_u8command=0x80;
	}
	else if(copy_u8Row==1)
	{
		copy_u8command=ADDRESS_ROW_1+copy_u8Col-1 ;
	}
	else if (copy_u8Row==2)
	{
		copy_u8command=ADDRESS_ROW_2+copy_u8Col-1;
	}
	else if (copy_u8Row==3)
	{
		copy_u8command=ADDRESS_ROW_3+copy_u8Col-1;
	}
	else if (copy_u8Row==4)
	{
		copy_u8command=ADDRESS_ROW_4+copy_u8Col-1;
	}
	LCD_void_sendCommand(copy_u8command);
	_delay_ms(1);
}
void LCD_void_creatCustomChar(const u8 * ArrPattern,u8 copy_u8charCode)
{
	/*SET CGRAM ADDRESS BASED ON copy_u8charCode */
	u8 command = (((copy_u8charCode & (TAKE_FIRST_3_BITS))*CHAR_CODES_OFFSET_IN_CGRAM) + LCD_CGRAM_ADDRESS);
	LCD_void_sendCommand(command);

	/*WRITE DATA TO CG OR DD RAM*/
	for(int i=0; i<8; i++)
	{
		LCD_void_sendData(ArrPattern[i]);
	}
	LCD_void_sendCommand(LCD_DDRAM_ADDRESS );
	_delay_ms(1);

}
void LCD_void_displayCustomChar(u8 copy_u8charCode)
{
	/*EXECUTE COMMAND WRITE DATA TO CG OR DD RAM WITH CHAR CODE*/
	LCD_void_sendData(copy_u8charCode);
}

static void H_LCD_void_latchByte(u8 copy_u8Byte)
{
#if(LCD_MODE== _4_BIT_MODE)

	DIO_voidSetPinValue(LCD_PIN7_PORT,LCD_PIN7_PIN,READBIT(copy_u8Byte,7));
	DIO_voidSetPinValue(LCD_PIN6_PORT,LCD_PIN6_PIN,READBIT(copy_u8Byte,6));
	DIO_voidSetPinValue(LCD_PIN5_PORT,LCD_PIN5_PIN,READBIT(copy_u8Byte,5));
	DIO_voidSetPinValue(LCD_PIN4_PORT,LCD_PIN4_PIN,READBIT(copy_u8Byte,4));

	DIO_voidSetPinValue(EN_PORT,EN_PIN,1);
	_delay_ms(1);
	DIO_voidSetPinValue(EN_PORT,EN_PIN,0);
	_delay_ms(1);

	DIO_voidSetPinValue(LCD_PIN7_PORT,LCD_PIN7_PIN,READBIT(copy_u8Byte,3));
	DIO_voidSetPinValue(LCD_PIN6_PORT,LCD_PIN6_PIN,READBIT(copy_u8Byte,2));
	DIO_voidSetPinValue(LCD_PIN5_PORT,LCD_PIN5_PIN,READBIT(copy_u8Byte,1));
	DIO_voidSetPinValue(LCD_PIN4_PORT,LCD_PIN4_PIN,READBIT(copy_u8Byte,0));

	DIO_voidSetPinValue(EN_PORT,EN_PIN,1);
	_delay_ms(1);
	DIO_voidSetPinValue(EN_PORT,EN_PIN,0);
	_delay_ms(1);

#elif(LCD_MODE== _8_BIT_MODE)
	DIO_voidSetPinValue(LCD_PIN0_PORT,LCD_PIN0_PIN,READBIT(copy_u8Byte,0));
	DIO_voidSetPinValue(LCD_PIN1_PORT,LCD_PIN1_PIN,READBIT(copy_u8Byte,1));
	DIO_voidSetPinValue(LCD_PIN2_PORT,LCD_PIN2_PIN,READBIT(copy_u8Byte,2));
	DIO_voidSetPinValue(LCD_PIN3_PORT,LCD_PIN3_PIN,READBIT(copy_u8Byte,3));
	DIO_voidSetPinValue(LCD_PIN4_PORT,LCD_PIN4_PIN,READBIT(copy_u8Byte,4));
	DIO_voidSetPinValue(LCD_PIN5_PORT,LCD_PIN5_PIN,READBIT(copy_u8Byte,5));
	DIO_voidSetPinValue(LCD_PIN6_PORT,LCD_PIN6_PIN,READBIT(copy_u8Byte,6));
	DIO_voidSetPinValue(LCD_PIN7_PORT,LCD_PIN7_PIN,READBIT(copy_u8Byte,7));

	DIO_voidSetPinValue(EN_PORT,EN_PIN,1);
	_delay_ms(1);
	DIO_voidSetPinValue(EN_PORT,EN_PIN,0);
	_delay_ms(1);
#endif

}
