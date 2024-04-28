/*
 * Keybad.c
 *
 *  Created on: Oct 21, 2023
 *      Author: OMR
 */

#include"KEYBAD.h"
#include"KEYBAD_cfg.h"
#include"GPIO.h"
#include<util/delay.h>



void Keybad_init()
{
	/*configure rows(PC2 -> PC5) as output*/
	PERIPHRAL_C ->direction.ALL_BITS |= 0b00111100;

	/*initialize all rows = 1*/
	PERIPHRAL_C ->PORT_R.ALL_BITS |= 0b00111100;

	/*configure cols(PD3, PD5, PD6, PD7) as input*/
	PERIPHRAL_D ->direction.ALL_BITS &= 0b00010111;

}
u8 Keybad_getValue()
{
	u8 key;
	u8 rows[4] = {2, 3, 4, 5};
	u8 cols[4] = {3, 5, 6, 7};
	while(1)
	{
	/*loop on rows*/
		for(int r=0; r<4; r++)
		{
			//DIO_voidSetPinDirection(PERIPHRAL_C, i+2, 1);
			DIO_voidSetPinValue(PERIPHRAL_C, rows[r], 0);
			/*loop on cols*/
			for(int c=0; c<4; c++)
			{
				if(DIO_voidGetPinValue(PERIPHRAL_D, cols[c]) == 0)
				{
					_delay_ms(30); /*handel debouncing*/
					if(DIO_voidGetPinValue(PERIPHRAL_D, cols[c]) == 0)
					{
						key = keybad_arr1[r][c];
						while(DIO_voidGetPinValue(PERIPHRAL_D, cols[c]) == 0);
						return key;
					}
				}
			}

			DIO_voidSetPinValue(PERIPHRAL_C, rows[r], 1);
		//	DIO_voidSetPinDirection(PERIPHRAL_C, i+2, 0);


			_delay_ms(20);
		}

	}
}
