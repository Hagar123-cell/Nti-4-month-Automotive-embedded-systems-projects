/*
 * TIMER1_prog.c
 *
 *  Created on: Oct 28, 2023
 *      Author: OMR
 */

#include "STDTYPES.h"
#include "TIMER1_config.h"
#include "TIMER1_int.h"
#include "TIMER1_priv.h"
#include"GPIO.h"
#include"EXTI.h"
#include"LCD.h"
s32 timer1OverFlowCounter = 0;
u16 timer1RemCounter = 0;

void(*timer1PtrCallBack)(void) = NULL;

void TIMER1_Init(const TIMER1_INIT_CONFIG* Config_Ptr)
{
	switch(Config_Ptr->Mode)
	{
	case TIMER1_NORMAL_MODE:
		TCCR1A_REG->bits.FOC1A_BIT = LOGIC_HIGH;
		TCCR1A_REG->bits.FOC1B_BIT = LOGIC_HIGH;
		TCCR1A_REG->bits.COM1A1_BIT = (Config_Ptr->Compare_output_Non_PWM>>1);
		TCCR1A_REG->bits.COM1A0_BIT = (Config_Ptr->Compare_output_Non_PWM>>0);
		TCCR1A_REG->bits.COM1B1_BIT = (Config_Ptr->Compare_output_Non_PWM>>1);
		TCCR1A_REG->bits.COM1B0_BIT = (Config_Ptr->Compare_output_Non_PWM>>0);
		OCR1A_REG = Config_Ptr->TIMER1_COMPARE_VALUE;
		OCR1B_REG = Config_Ptr->TIMER1_COMPARE_VALUE;
		break;
	case TIMER1_PHASE_CORRECT_PWM_8bIT_MODE:
		TCCR1A_REG->bits.FOC1A_BIT = LOGIC_LOW;
		TCCR1A_REG->bits.FOC1B_BIT = LOGIC_LOW;
		TCCR1A_REG->bits.COM1A1_BIT = (Config_Ptr->Compare_output_Phase_Correct_PWM>>1);
		TCCR1A_REG->bits.COM1A0_BIT = (Config_Ptr->Compare_output_Phase_Correct_PWM>>0);
		TCCR1A_REG->bits.COM1B1_BIT = (Config_Ptr->Compare_output_Phase_Correct_PWM>>1);
		TCCR1A_REG->bits.COM1B0_BIT = (Config_Ptr->Compare_output_Phase_Correct_PWM>>0);
		//		OCR1A_REG = Config_Ptr->TIMER1_COMPARE_VALUE;
		//		OCR1B_REG = Config_Ptr->TIMER1_COMPARE_VALUE;
		break;
	case TIMER1_PHASE_CORRECT_PWM_9bIT_MODE:
		TCCR1A_REG->bits.FOC1A_BIT = LOGIC_LOW;
		TCCR1A_REG->bits.FOC1B_BIT = LOGIC_LOW;
		TCCR1A_REG->bits.COM1A1_BIT = (Config_Ptr->Compare_output_Phase_Correct_PWM>>1);
		TCCR1A_REG->bits.COM1A0_BIT = (Config_Ptr->Compare_output_Phase_Correct_PWM>>0);
		TCCR1A_REG->bits.COM1B1_BIT = (Config_Ptr->Compare_output_Phase_Correct_PWM>>1);
		TCCR1A_REG->bits.COM1B0_BIT = (Config_Ptr->Compare_output_Phase_Correct_PWM>>0);
		OCR1A_REG = Config_Ptr->TIMER1_COMPARE_VALUE;
		OCR1B_REG = Config_Ptr->TIMER1_COMPARE_VALUE;
		break;
	case TIMER1_PHASE_CORRECT_PWM_10bIT_MODE:
		TCCR1A_REG->bits.FOC1A_BIT = LOGIC_LOW;
		TCCR1A_REG->bits.FOC1B_BIT = LOGIC_LOW;
		TCCR1A_REG->bits.COM1A1_BIT = (Config_Ptr->Compare_output_Phase_Correct_PWM>>1);
		TCCR1A_REG->bits.COM1A0_BIT = (Config_Ptr->Compare_output_Phase_Correct_PWM>>0);
		TCCR1A_REG->bits.COM1B1_BIT = (Config_Ptr->Compare_output_Phase_Correct_PWM>>1);
		TCCR1A_REG->bits.COM1B0_BIT = (Config_Ptr->Compare_output_Phase_Correct_PWM>>0);
		OCR1A_REG = Config_Ptr->TIMER1_COMPARE_VALUE;
		OCR1B_REG = Config_Ptr->TIMER1_COMPARE_VALUE;
		break;
	case TIMER1_CTC_OCR1A_MODE:
		TCCR1A_REG->bits.FOC1A_BIT = LOGIC_LOW;
		TCCR1A_REG->bits.FOC1B_BIT = LOGIC_LOW;
		TCCR1A_REG->bits.COM1A1_BIT = (Config_Ptr->Compare_output_Non_PWM>>1);
		TCCR1A_REG->bits.COM1A0_BIT = (Config_Ptr->Compare_output_Non_PWM>>0);
		TCCR1A_REG->bits.COM1B1_BIT = (Config_Ptr->Compare_output_Non_PWM>>1);
		TCCR1A_REG->bits.COM1B0_BIT = (Config_Ptr->Compare_output_Non_PWM>>0);
		OCR1A_REG = Config_Ptr->TIMER1_COMPARE_VALUE;
		OCR1B_REG = Config_Ptr->TIMER1_COMPARE_VALUE;
		break;
	case TIMER1_FAST_PWM_8bIT_MODE:
		TCCR1A_REG->bits.FOC1A_BIT = LOGIC_LOW;
		TCCR1A_REG->bits.FOC1B_BIT = LOGIC_LOW;
		TCCR1A_REG->bits.COM1A1_BIT = (Config_Ptr->Compare_output_Fast_PWM>>1);
		TCCR1A_REG->bits.COM1A0_BIT = (Config_Ptr->Compare_output_Fast_PWM>>0);
		TCCR1A_REG->bits.COM1B1_BIT = (Config_Ptr->Compare_output_Fast_PWM>>1);
		TCCR1A_REG->bits.COM1B0_BIT = (Config_Ptr->Compare_output_Fast_PWM>>0);
		OCR1A_REG = Config_Ptr->TIMER1_COMPARE_VALUE;
		OCR1B_REG = Config_Ptr->TIMER1_COMPARE_VALUE;
		break;
	case TIMER1_FAST_PWM_9bIT_MODE:
		TCCR1A_REG->bits.FOC1A_BIT = LOGIC_LOW;
		TCCR1A_REG->bits.FOC1B_BIT = LOGIC_LOW;
		TCCR1A_REG->bits.COM1A1_BIT = (Config_Ptr->Compare_output_Fast_PWM>>1);
		TCCR1A_REG->bits.COM1A0_BIT = (Config_Ptr->Compare_output_Fast_PWM>>0);
		TCCR1A_REG->bits.COM1B1_BIT = (Config_Ptr->Compare_output_Fast_PWM>>1);
		TCCR1A_REG->bits.COM1B0_BIT = (Config_Ptr->Compare_output_Fast_PWM>>0);
		OCR1A_REG = Config_Ptr->TIMER1_COMPARE_VALUE;
		OCR1B_REG = Config_Ptr->TIMER1_COMPARE_VALUE;
		break;
	case TIMER1_FAST_PWM_10bIT_MODE:
		TCCR1A_REG->bits.FOC1A_BIT = LOGIC_LOW;
		TCCR1A_REG->bits.FOC1B_BIT = LOGIC_LOW;
		TCCR1A_REG->bits.COM1A1_BIT = (Config_Ptr->Compare_output_Fast_PWM>>1);
		TCCR1A_REG->bits.COM1A0_BIT = (Config_Ptr->Compare_output_Fast_PWM>>0);
		TCCR1A_REG->bits.COM1B1_BIT = (Config_Ptr->Compare_output_Fast_PWM>>1);
		TCCR1A_REG->bits.COM1B0_BIT = (Config_Ptr->Compare_output_Fast_PWM>>0);
		OCR1A_REG = Config_Ptr->TIMER1_COMPARE_VALUE;
		OCR1B_REG = Config_Ptr->TIMER1_COMPARE_VALUE;
		break;
	case TIMER1_PHASE_FREQUENCY_CORRECT_PWM_ICR1_MODE:
		TCCR1A_REG->bits.FOC1A_BIT = LOGIC_LOW;
		TCCR1A_REG->bits.FOC1B_BIT = LOGIC_LOW;
		TCCR1A_REG->bits.COM1A1_BIT = (Config_Ptr->Compare_output_Phase_Correct_PWM>>1);
		TCCR1A_REG->bits.COM1A0_BIT = (Config_Ptr->Compare_output_Phase_Correct_PWM>>0);
		TCCR1A_REG->bits.COM1B1_BIT = (Config_Ptr->Compare_output_Phase_Correct_PWM>>1);
		TCCR1A_REG->bits.COM1B0_BIT = (Config_Ptr->Compare_output_Phase_Correct_PWM>>0);
		OCR1A_REG = Config_Ptr->TIMER1_COMPARE_VALUE;
		OCR1B_REG = Config_Ptr->TIMER1_COMPARE_VALUE;
		break;
	case TIMER1_PHASE_FREQUENCY_CORRECT_PWM_OCR1A_MODE:
		TCCR1A_REG->bits.FOC1A_BIT = LOGIC_LOW;
		TCCR1A_REG->bits.FOC1B_BIT = LOGIC_LOW;
		TCCR1A_REG->bits.COM1A1_BIT = (Config_Ptr->Compare_output_Phase_Correct_PWM>>1);
		TCCR1A_REG->bits.COM1A0_BIT = (Config_Ptr->Compare_output_Phase_Correct_PWM>>0);
		TCCR1A_REG->bits.COM1B1_BIT = (Config_Ptr->Compare_output_Phase_Correct_PWM>>1);
		TCCR1A_REG->bits.COM1B0_BIT = (Config_Ptr->Compare_output_Phase_Correct_PWM>>0);
		OCR1A_REG = Config_Ptr->TIMER1_COMPARE_VALUE;
		OCR1B_REG = Config_Ptr->TIMER1_COMPARE_VALUE;
		break;
	case TIMER1_PHASE_CORRECT_PWM_ICR1_MODE:
		TCCR1A_REG->bits.FOC1A_BIT = LOGIC_LOW;
		TCCR1A_REG->bits.FOC1B_BIT = LOGIC_LOW;
		TCCR1A_REG->bits.COM1A1_BIT = (Config_Ptr->Compare_output_Phase_Correct_PWM>>1);
		TCCR1A_REG->bits.COM1A0_BIT = (Config_Ptr->Compare_output_Phase_Correct_PWM>>0);
		TCCR1A_REG->bits.COM1B1_BIT = (Config_Ptr->Compare_output_Phase_Correct_PWM>>1);
		TCCR1A_REG->bits.COM1B0_BIT = (Config_Ptr->Compare_output_Phase_Correct_PWM>>0);
		OCR1A_REG = Config_Ptr->TIMER1_COMPARE_VALUE;
		OCR1B_REG = Config_Ptr->TIMER1_COMPARE_VALUE;
		break;
	case TIMER1_PHASE_CORRECT_PWM_OCR1A_MODE:
		TCCR1A_REG->bits.FOC1A_BIT = LOGIC_LOW;
		TCCR1A_REG->bits.FOC1B_BIT = LOGIC_LOW;
		TCCR1A_REG->bits.COM1A1_BIT = (Config_Ptr->Compare_output_Phase_Correct_PWM>>1);
		TCCR1A_REG->bits.COM1A0_BIT = (Config_Ptr->Compare_output_Phase_Correct_PWM>>0);
		TCCR1A_REG->bits.COM1B1_BIT = (Config_Ptr->Compare_output_Phase_Correct_PWM>>1);
		TCCR1A_REG->bits.COM1B0_BIT = (Config_Ptr->Compare_output_Phase_Correct_PWM>>0);
		OCR1A_REG = Config_Ptr->TIMER1_COMPARE_VALUE;
		OCR1B_REG = Config_Ptr->TIMER1_COMPARE_VALUE;
		break;
	case TIMER1_CTC_ICR1_MODE:
		TCCR1A_REG->bits.FOC1A_BIT = LOGIC_LOW;
		TCCR1A_REG->bits.FOC1B_BIT = LOGIC_LOW;
		TCCR1A_REG->bits.COM1A1_BIT = (Config_Ptr->Compare_output_Non_PWM>>1);
		TCCR1A_REG->bits.COM1A0_BIT = (Config_Ptr->Compare_output_Non_PWM>>0);
		TCCR1A_REG->bits.COM1B1_BIT = (Config_Ptr->Compare_output_Non_PWM>>1);
		TCCR1A_REG->bits.COM1B0_BIT = (Config_Ptr->Compare_output_Non_PWM>>0);
		OCR1A_REG = Config_Ptr->TIMER1_COMPARE_VALUE;
		OCR1B_REG = Config_Ptr->TIMER1_COMPARE_VALUE;
		break;
	case TIMER1_FAST_PWM_ICR1_MODE:
		TCCR1A_REG->bits.FOC1A_BIT = LOGIC_LOW;
		TCCR1A_REG->bits.FOC1B_BIT = LOGIC_LOW;
		TCCR1A_REG->bits.COM1A1_BIT = (Config_Ptr->Compare_output_Fast_PWM>>1);
		TCCR1A_REG->bits.COM1A0_BIT = (Config_Ptr->Compare_output_Fast_PWM>>0);
		TCCR1A_REG->bits.COM1B1_BIT = (Config_Ptr->Compare_output_Fast_PWM>>1);
		TCCR1A_REG->bits.COM1B0_BIT = (Config_Ptr->Compare_output_Fast_PWM>>0);
		OCR1A_REG = Config_Ptr->TIMER1_COMPARE_VALUE;
		OCR1B_REG = Config_Ptr->TIMER1_COMPARE_VALUE;
		break;
	case TIMER1_FAST_PWM_OCR1A_MODE:
		TCCR1A_REG->bits.FOC1A_BIT = LOGIC_LOW;
		TCCR1A_REG->bits.FOC1B_BIT = LOGIC_LOW;
		TCCR1A_REG->bits.COM1A1_BIT = (Config_Ptr->Compare_output_Fast_PWM>>1);
		TCCR1A_REG->bits.COM1A0_BIT = (Config_Ptr->Compare_output_Fast_PWM>>0);
		TCCR1A_REG->bits.COM1B1_BIT = (Config_Ptr->Compare_output_Fast_PWM>>1);
		TCCR1A_REG->bits.COM1B0_BIT = (Config_Ptr->Compare_output_Fast_PWM>>0);
		OCR1A_REG = Config_Ptr->TIMER1_COMPARE_VALUE;
		OCR1B_REG = Config_Ptr->TIMER1_COMPARE_VALUE;
		break;
	}
	TCNT1_REG = (Config_Ptr->TIMER1_INITIAL_VALUE);
	TCCR1A_REG->bits.WGM10_BIT = (Config_Ptr->Mode);
	TCCR1A_REG->bits.WGM11_BIT = (Config_Ptr->Mode>> 1);
	TCCR1B_REG->bits.WGM12_BIT = (Config_Ptr->Mode >> 2);
	TCCR1B_REG->bits.WGM13_BIT = (Config_Ptr->Mode >> 3);
	//TCCR0_REG->FULL_REG = ((TCCR0_REG->FULL_REG & CLOCK_SELECT_MASK) | (Config_Ptr->Clock_Select));
}
void TIMER1_Start(const TIMER1_INIT_CONFIG* Config_Ptr)
{
	TCCR1B_REG->FULL_REG = ((TCCR1B_REG->FULL_REG & CLOCK_SELECT_MASK) | ((Config_Ptr->Clock_Select & 0x07)));
}
void TIMER1_Stop(const TIMER1_INIT_CONFIG* Config_Ptr)
{
	TCCR1B_REG->FULL_REG = ((TCCR1B_REG->FULL_REG & CLOCK_SELECT_MASK) | (NO_CLOCK));
}
u16 TIMER1_GetCounts(void)
{
	return TCNT1_REG;
}
void TIMER1_SetDelayTimeMilliSec(const TIMER1_INIT_CONFIG* Config_Ptr, u32 copy_u32TimeMS)
{
	switch(Config_Ptr->Mode)
	{
	case TIMER1_NORMAL_MODE:
		timer1OverFlowCounter = (((copy_u32TimeMS * 1000)/TICK_TIME) / TOP_VALUE);
		timer1RemCounter = (u16)(((copy_u32TimeMS * 1000)/TICK_TIME) % TOP_VALUE);
		TIMER1_IntEnable(Config_Ptr);
		TIMER1_Start(Config_Ptr);
		break;
	}
}
void TIMER1_IntEnable(const TIMER1_INIT_CONFIG* Config_Ptr)
{
	if(Config_Ptr->Mode == TIMER1_NORMAL_MODE)
	{
		TIMSK_REG->bits.TOIE1_BIT = LOGIC_HIGH;
	}
	/*else if (Config_Ptr->Mode == TIMER1_CTC_MODE)
	{
		TIMSK_REG->bits.OCIE0_BIT = LOGIC_HIGH;
	}
	else
	{
		//return error;
	}
	 */
}
void TIMER1_IntDisable(const TIMER1_INIT_CONFIG* Config_Ptr)
{
	if(Config_Ptr->Mode == TIMER1_NORMAL_MODE)
	{
		TIMSK_REG->bits.TOIE1_BIT = LOGIC_HIGH;
	}
	/*	else if (Config_Ptr->Mode == TIMER1_CTC_MODE)
	{
		TIMSK_REG->bits.OCIE0_BIT = LOGIC_LOW;
	}
	else
	{
		//return error;
	}
	 */
}

void TIMER1_SetCallBack(void(*ptrfn)(void))
{
	timer1PtrCallBack = ptrfn;
}
void TIMER1_SetFastPWM(const TIMER1_INIT_CONFIG* Config_Ptr, u16 duty)
{
	/* pwm freq = FCPU/(prescaler*256) */

	/* duty cycle */
	u16 dutyval = 0;
	//	dutyval = (u16)((duty*TOP_VALUE)/100);
	dutyval = (u16)((duty*256)/100);

	OCR1A_REG = dutyval;

	OCR1B_REG = dutyval;

}
void TIMER1_SetPhaseCorrectPWM(const TIMER1_INIT_CONFIG* Config_Ptr, u16 duty);

void TIMER1_GetEventDuration(void)
{
	ICR1_REG = 0;
	TIMSK_REG->bits.TICIE1_BIT = LOGIC_HIGH;
	TCCR1B_REG->bits.ICNC1_BIT = LOGIC_LOW;
	TCCR1B_REG->bits.ICES1_BIT = RISING_EDGE;
}
void TIMER1_GetSignalFrequency(void)
{
	ICR1_REG = 0;
	TIMSK_REG->bits.TICIE1_BIT = LOGIC_HIGH;
	TCCR1B_REG->bits.ICNC1_BIT = LOGIC_LOW;
	TCCR1B_REG->bits.ICES1_BIT = RISING_EDGE;
}
void TIMER1_GetSignalDutyCycle(void)
{
	ICR1_REG = 0;
	TIMSK_REG->bits.TICIE1_BIT = LOGIC_HIGH;
	TCCR1B_REG->bits.ICNC1_BIT = LOGIC_LOW;
	TCCR1B_REG->bits.ICES1_BIT = RISING_EDGE;
}

u16 TIMER1_GetInputCaptureValue(void)
{
	return ICR1_REG;
}

void TIMER1_SetInputCaptureEdgeDetection(u8 trigger)
{
	TCCR1B_REG->bits.ICES1_BIT = trigger;
}

void TIMER1_ClearTimerValue(void)
{
	TCNT1_REG = 0;
}

void TIMER1_DisableICU(void)
{
	TIMSK_REG->bits.TICIE1_BIT = 0;
}

void TIMER1_EnableICU(void)
{
	TIMSK_REG->bits.TICIE1_BIT = 1;
}
ISR(TIMER1_OVF_vect)
{
	/* For the delay func */

	timer1OverFlowCounter--;

	if(timer1OverFlowCounter == 0)
	{
		TCNT1_REG = (TOP_VALUE - timer1RemCounter);
	}

	if((timer1PtrCallBack != NULL) && (timer1OverFlowCounter == -1))
	{
		(*timer1PtrCallBack)();
	}
}

ISR(TIMER1_ICU_vect)
{
	if(timer1PtrCallBack != NULL)
	{
		(*timer1PtrCallBack)();
	}
}

ISR(TIMER1_OCA_vect)
{
	if(timer1PtrCallBack != NULL)
	{
		(*timer1PtrCallBack)();
	}
}


ISR(TIMER1_OCB_vect)
{
	if(timer1PtrCallBack != NULL)
	{
		(*timer1PtrCallBack)();
	}
}
