/*
 * RELAY.c
 *
 *  Created on: Nov 15, 2023
 *      Author: OMR
 */


#include"RELAY.h"
#include"GPIO.h"
#include"RELAY.h"
#include"MCU_HW.h"
#include"RELAY_cfg.h"

/**
 * Initialize relay control pin.
 */
void RELAY_init ( )
{
	DIO_voidSetPinDirection(RELAY_PORT, RELAY_PIN, OUTPUT);

	/*turn off relay initialy*/
	RELAY_off();
}


/**
 * Turn on the relay.
 */
void RELAY_on ( )
{
	DIO_voidSetPinValue(RELAY_PORT, RELAY_PIN, LOGIC_HIGH);
}


/**
 * Turn off the relay.
 */
void RELAY_off ( )
{
	DIO_voidSetPinValue(RELAY_PORT, RELAY_PIN, LOGIC_LOW);
}
