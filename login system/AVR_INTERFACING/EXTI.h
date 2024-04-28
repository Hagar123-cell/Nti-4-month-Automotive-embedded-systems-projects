/*
 * EXTI.c
 *
 *  Created on: Oct 22, 2023
 *      Author: OMR
 */

#ifndef EXTI_H_
#define EXTI_H_

#include "MCU_HW.h"

/* Interrupt vectors */
/* External Interrupt Request 0 */
#define INT0_vect			__vector_1
/* External Interrupt Request 1 */
#define INT1_vect			__vector_2
/* External Interrupt Request 2 */
#define INT2_vect			__vector_3
/* Timer/Counter2 Compare Match */
#define TIMER2_COMP_vect		__vector_4
/* Timer/Counter2 Overflow */
#define TIMER2_OVF_vect			__vector_5
/* Timer/Counter1 Capture Event */
#define TIMER1_ICU_vect		__vector_6
/* Timer/Counter1 Compare Match A */
#define TIMER1_OCA_vect		__vector_7
/* Timer/Counter1 Compare Match B */
#define TIMER1_OCB_vect		__vector_8
/* Timer/Counter1 Overflow */
#define TIMER1_OVF_vect			__vector_9
/* Timer/Counter0 Compare Match */
#define TIMER0_OC_vect		__vector_10
/* Timer/Counter0 Overflow */
#define TIMER0_OVF_vect			__vector_11
/* Serial Transfer Complete */
#define SPI_STC_vect			__vector_12
/* USART, Rx Complete */
#define UART_RX_vect			__vector_13
/* USART Data Register Empty */
#define UART_UDRE_vect			__vector_14
/* USART, Tx Complete */
#define UART_TX_vect			__vector_15
/* ADC Conversion Complete */
#define ADC_vect			   __vector_16
/* EEPROM Ready */
#define EE_RDY_vect			   __vector_17
/* Analog Comparator */
#define ANA_COMP_vect			__vector_18
/* 2-wire Serial Interface */
#define TWI_vect			    __vector_19
/* Store Program Memory Ready */
#define SPM_RDY_vect			__vector_20

#  define BAD_vect        __vector_default



#  define ISR_NOBLOCK    __attribute__((interrupt))
#  define ISR_NAKED      __attribute__((naked))


#  define ISR(vector,...)            \
void vector (void) __attribute__ ((signal))__VA_ARGS__ ; \
void vector (void)



typedef enum
{
	INT2 = 5,
	INT0,
	INT1
}INT_ID;

typedef enum
{
EXTI_LOW_LEVEL ,
EXTI_ANY_CHANGE ,
EXTI_FALL_EDGE  ,  //   10 (take first bit for INT2)for INT2 0 activates falling edge
EXTI_RISE_EDGE    //   11 (take first bit for INT2)for INT2 1 activates rising edge
}triggerType;

void EXTI_enableInterrupt(INT_ID INT, triggerType trig);
void EXTI_disableInterrupt(INT_ID INT);
void EXTI_setCallBack(void(*ptr_to_fn)(void), INT_ID INT);

#endif /* EXTI_H_ */





