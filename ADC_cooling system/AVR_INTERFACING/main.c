/*
 * main.c
 *
 *  Created on: Nov 9, 2023
 *      Author: OMR
 */

#include"DC_MOTOR.h"
#include"LCD.h"
#include"LM35.h"
int main()
{

	DcMotor_Init();
	LM35_init();
	LCD_void_Init();
	LCD_void_sendCommand(CURSOR_OFF_DISPLAY_ON);


	u8 temp;
	while(1)
	{
		LCD_void_gotoXY(0, 0);
		temp=LM35_getTemperature();
		if( temp < 25)
		{
			DC_MOTOR_setSpeed(0, stop);
			LCD_void_sendString("temp = ");
			LCD_void_sendIntNum((u8)temp);
		}

		if( temp >= 25 && temp <= 35)
		{
			DC_MOTOR_setSpeed(50, clkWise);
			LCD_void_sendString("temp = ");
			LCD_void_sendIntNum((u8)temp);
		}

		if( temp > 35)
		{
			DC_MOTOR_setSpeed(80, clkWise);
			LCD_void_sendString("temp = ");
			LCD_void_sendIntNum((u8)temp);
		}

		LCD_void_sendData('c');

	}

}
