/*
 * ULTRASONIC.c
 *
 *  Created on: Nov 15, 2023
 *      Author: OMR
 */


#include"ULTRASONIC.h"
#include"ULTRASONIC_cfg.h"
#include"TIMER1_int.h"
#include"GPIO.h"
#include<util/delay.h>
#include"LCD.h"
#include"TIMER1_priv.h"
static u8 g_edgeCount = 0;
static u16 g_timeHigh = 0;
//static u8 g_distance = 0;


/*
 * Description :
 * Call-Back Function used to calculate g_timeHigh for the Ultrasonic pulse
 */
static void Ultrasonic_edgeProcessing(void)
{
	g_edgeCount++;
	if(g_edgeCount == 1)
	{
		LCD_void_sendIntNum(ICR1_REG);

		LCD_void_sendIntNum(TIMER1_GetInputCaptureValue());
		/*
		 * Clear the timer counter register to start measurements from the
		 * first detected rising edge
		 */
		TIMER1_ClearTimerValue();
		/* Detect falling edge */
		TIMER1_SetInputCaptureEdgeDetection(FALLING_EDGE);
	}
	else if(g_edgeCount == 2)
	{
		/* Store the High time value */
		g_timeHigh = TIMER1_GetInputCaptureValue();

		/* Detect rising edge */
		TIMER1_SetInputCaptureEdgeDetection(RISING_EDGE);
	}
}

void Ultrasonic_init(void)
{
	/*init timer1*/
	//Timer1_cfg cfg = {T1_Normal, T1_PRE1024, T1_OC1_DISCON, T1_OC1_DISCON};
	//Timer1_Init(&cfg);

	/*enable ICU interrupt*/
	//TIMER1_ICU_IntEnable();

	/*set ICU callback fn*/
		TIMER1_SetCallBack(Ultrasonic_edgeProcessing);

	/* Detect rising edge */
	TIMER1_SetInputCaptureEdgeDetection(RISING_EDGE);

	/*setup trigger pin as output*/
	DIO_voidSetPinDirection(ULTRASONIC_TRIGGER_PORT, ULTRASONIC_TRIGGER_PIN, OUTPUT);

	/*setup icp pin as input*/
	DIO_voidSetPinDirection(PERIPHRAL_D, 6, INPUT);

	/*enable ICU interrupt*/
	TIMER1_EnableICU();
}


/*
 * Description :
 * send the trigger pulse to the ultrasonic.
 */
void Ultrasonic_Trigger(void)
{
	DIO_voidSetPinValue(ULTRASONIC_TRIGGER_PORT, ULTRASONIC_TRIGGER_PIN, LOGIC_HIGH);
	_delay_us(10);
	DIO_voidSetPinValue(ULTRASONIC_TRIGGER_PORT, ULTRASONIC_TRIGGER_PIN, LOGIC_LOW);
}

/*
 * Description :
 * Send the trigger pulse by using Ultrasonic_Trigger function.
 * Start the measurements by the ICU from this moment.
 */
u16 Ultrasonic_readDistance(void)
{
	u16 g_distance=0;
	//TIMER1_ClearTimerValue();

	Ultrasonic_Trigger();
	LCD_void_sendIntNum(6);

	/* Wait until the ICU measures the pulse in the ECHO pin */
	while(g_edgeCount < 2);
	g_edgeCount = 0;
	/* Calculate the distance in Centimeter value */
	//LCD_void_sendIntNum((s32)g_timeHigh);
	return g_distance = g_timeHigh/58;
}
