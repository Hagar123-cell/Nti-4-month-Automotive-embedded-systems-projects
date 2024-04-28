/*
 * DC_MOTOR.c
 *
 *  Created on: Nov 9, 2023
 *      Author: OMR
 */


#include"DC_MOTOR.h"
#include"Timer0.h"
#include"GPIO.h"
#include"BIT_MATH.h"
void DcMotor_Init(void)
{
	/*SET IN1 & IN2 OF DC_MOTOR AS OUTPUT PINS FROM MC*/
	DIO_voidSetPinDirection(IN1_PORT_ID, 0, OUTPUT);
	DIO_voidSetPinDirection(IN2_PORT_ID, 1, OUTPUT);

	/*SET ENABLE PIN(OC0) AS OUTPUT*/
	DIO_voidSetPinDirection(PERIPHRAL_B, 3, OUTPUT);

	/*STOP DC_MOTOR INITIALY*/
	IN1_PORT_ID->PORT_R.bits.bit0=0;
	IN1_PORT_ID->PORT_R.bits.bit1=0;
	/*init timer0*/
	Timer0_cfg cfg = {PWM_fast, PRE8, NON_INVERTING};
	Timer0_Init(& cfg);
}
void DC_MOTOR_setSpeed(u8 duty, DcMotor_State state)
{
	/*adjust the state of the rotation of motor(clkwise/ anti clkwise/ stop)*/
		PERIPHRAL_B->PORT_R.ALL_BITS = (PERIPHRAL_B->PORT_R.ALL_BITS & 0xFC) | state;
	/*send the required speed to pwm to generate the wave on the enable pin of the motor*/
	setFastPWM( duty);

}
