/*
 * Timer.h
 *
 *  Created on: Oct 27, 2023
 *      Author: OMR
 */

#ifndef TIMER0_H_
#define TIMER0_H_

#include"Timer0_cfg.h"
#include"STDTYPES.h"
#define FCPU 16
#define OVF_Ticks  256
#define MAX_OCR0          255

typedef enum
{
	Normal_ID, CTC_ID
}modes_IDs;
void Timer0_Init(Timer0_cfg* cfg);
void Timer_start(TIMER0_PRESCALER pre);
void Timer_stop();
u16 Timer_GetCounts();
void Timer_setDelayTimeMilliSec(u32 delay_in_ms, Timer0_Mode mode);
void EnableInt(Timer0_Mode mode);
void DisableInt(Timer0_Mode mode);
void setCallBack(void (*ptr_to_callback)(void), modes_IDs mode_id);
void setFastPWM(s32 duty);
void setphaseCorrectPWM(s32 duty);
#endif /* TIMER0_H_ */
