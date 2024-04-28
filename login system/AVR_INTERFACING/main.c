

#include"ULTRASONIC.h"
#include"RELAY.h"
#include"LCD.h"
#include"KEYBAD.h"
#include <util/delay.h>
#include"LED.h"
#include"TIMER1_int.h"
#include"TIMER1_priv.h"
#include"GIE_R.h"
#include"GPIO.h"
void led()
{
	DIO_voidSetPinDirection(PERIPHRAL_C, 0, 1);
	DIO_voidSetPinValue(PERIPHRAL_C, 0, 1);
}

void TEST()
{
	LCD_void_sendIntNum(TIMER1_GetInputCaptureValue());

	LCD_void_sendIntNum(TCNT1_REG);

}
int main()
{


	//RELAY_init();
	LCD_void_Init();
	//Keybad_init();

	/*init timer1*/
	//Timer1_cfg cfg = {T1_Normal, T1_PRE1024, T1_OC1_DISCON, T1_OC1_DISCON};
	//Timer1_Init(&cfg);

	//Timer1_EnableInt(T1_OVF);
	//Icu_ConfigType cfg={T1_PRE1024, RISING_EDGE_TRIGGER};
	//ICU_init(&cfg);

	TIMER1_INIT_CONFIG cfg = { TIMER1_COMPARE_OUTPUT_NON_PWM_MODE_OC1A_OC1B_DISCONNECTED, TIMER1_COMPARE_OUTPUT_FAST_PWM_MODE_OC1A_OC1B_DISCONNECTED, FCPU_DIVIDED_BY_1024, TIMER1_NORMAL_MODE,0, 0};

	TIMER1_Init(&cfg);
	/*setup icp pin as input*/
		DIO_voidSetPinDirection(PERIPHRAL_D, 6, INPUT);

	TIMER1_SetInputCaptureEdgeDetection(FALLING_EDGE);

	TIMER1_SetCallBack(TEST);
	TIMER1_IntEnable(&cfg);
	TIMER1_EnableICU();

	//Ultrasonic_init();
	GIE_enableGlobalInterrupt();

	u16 distance;
	while(1)
	{
		//u16 distance = Ultrasonic_readDistance();
		//TIMER1_ICU_setCallBack(led);
		//LCD_void_sendIntNum(5);

		/*distance = Ultrasonic_readDistance();
		LCD_void_sendIntNum(distance);
		LCD_void_sendIntNum(5);*/
	}
	/*_delay_ms(1000);
	LCD_void_sendIntNum(Timer1_GetCounts());

	_delay_ms(1000);
	LCD_void_sendIntNum(Timer1_GetCounts());*/
	//LCD_void_sendString("please enter your password: ");
	//Ultrasonic_Trigger();
	//u16 distance = Ultrasonic_readDistance();
	//LCD_void_sendIntNum(distance);

#if 0
	u16 distance, pass=0;
	u8 pressed_key, F_trial_counter=0;
	while(1)
	{
		distance = Ultrasonic_readDistance();

		if(distance >=0 &&  distance <=15)
		{

			LCD_void_sendCommand(LCD_CLEAR);
			LCD_void_sendString("please enter your password: ");
			for(int i=0; i<4; i++)
			{
				pressed_key=Keybad_getValue();
				pass=pass*10+pressed_key;
				LCD_void_sendData('*');
			}

			if(pass == 1234)
			{
				LCD_void_sendCommand(LCD_CLEAR);
				LCD_void_sendString("Welcome, hagar");
				RELAY_on();
				F_trial_counter=0;

				_delay_ms(20);

				while(1)
				{
					LCD_void_sendString("1_ leds ON");
					LCD_void_gotoXY(1, 0);
					LCD_void_sendString("2_ leds OFF");

					pressed_key = Keybad_getValue();
					if(pressed_key == 1)
					{
						LED_on(PERIPHRAL_B, 0);
						LED_on(PERIPHRAL_B, 1);
					}

					else if(pressed_key == 2)
					{
						LED_off(PERIPHRAL_B, 0);
						LED_off(PERIPHRAL_B, 1);
					}
					LCD_void_sendCommand(LCD_CLEAR);
				}


			}

			else
			{
				LCD_void_sendCommand(LCD_CLEAR);
				LCD_void_sendString("Wrong password Please try again");
				F_trial_counter++;
				if(F_trial_counter == 3)
				{
					LCD_void_sendCommand(LCD_CLEAR);
					LCD_void_sendString("Wrong password system will be locked for 2 minutes");
					RELAY_off();
					F_trial_counter=0;
					/*delay for 2 mins*/
					_delay_ms(120000);
				}


			}

			distance = Ultrasonic_readDistance();
		}

		/*if user is not within specified distance*/
		else
		{
			LCD_void_sendCommand(LCD_CLEAR);
			LCD_void_sendString("distance=");
			LCD_void_sendIntNum(distance);
			F_trial_counter=0;
		}


	}
#endif
}
