/*
 * main.c
 *
 *  Created on: Oct 20, 2023
 *      Author: hagar ahmed
 */


#include"LCD.h"
#include"KEYBAD.h"
#include"DC_MOTOR.h"
#include <util/delay.h>

int main()
{
	LCD_void_Init();
	Keybad_init();
	DcMotor_Init();

	u8 pressed_key, duty_cycle, i ;

	while(1)
	{
		LCD_void_sendString("enter duty cycle     ended with = : ");
		duty_cycle=0, i=0;

		while(i<3)
		{
			i++;
			pressed_key = Keybad_getValue();
			if(pressed_key == '=')
				break;
			duty_cycle = (duty_cycle*10) + (pressed_key);
		}


		LCD_void_sendIntNum(duty_cycle);
		DC_MOTOR_setSpeed(duty_cycle, clkWise);

		_delay_ms(3000);

		LCD_void_sendCommand(LCD_CLEAR);
		LCD_void_gotoXY(0, 0);

	}

}
