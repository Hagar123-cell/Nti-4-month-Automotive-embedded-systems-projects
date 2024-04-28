/*
 * Timer_cfg.h
 *
 *  Created on: Oct 27, 2023
 *      Author: OMR
 */

#ifndef TIMER0_CFG_H_
#define TIMER0_CFG_H_

#define TIMER0_PWM_MODE   TIMER0_NON_INVERTING_PWM_MODE
typedef enum
{
 Normal, PWM_PhaseCorrect, T0_CTC, PWM_fast
}Timer0_Mode;

typedef enum
{
	No_clk_src, Nopre, PRE8, PRE64, PRE256, PRE1024,  Ext_clk_src_on_T0_on_falling_edge, Ext_clk_src_on_T0_on_rising_edge
}TIMER0_PRESCALER;

typedef enum
{
	OC0_DISCON, TOGGLE_OC0, NON_INVERTING, INVERTING
}CMP_OUTPUT_MODE;

typedef struct
{
	Timer0_Mode mode;
	TIMER0_PRESCALER pre;
	CMP_OUTPUT_MODE CMP_MODE;
}Timer0_cfg;
#endif /* TIMER0_CFG_H_ */
