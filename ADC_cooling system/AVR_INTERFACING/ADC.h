/*
 * ADC_interface.h
 *
 * Created: 10/23/2023 9:16:40 AM
 *  Author: OMR
 */ 


#ifndef ADC_H_
#define ADC_H_

#include "STDTYPES.h"
#include"ADC_cfg.h"

#define ADC_TIME_LIMIT   50000

#define INTERRUPT_ENABLE 1

void ADC_init(ADC_CFG* cfg);

void ADC_getDigitalValueSynchNonBlocking (CHANNEL_NUMBER ch_num, u16* ptr);

void ADC_readChannel(u8 CH_num, u16* ptr);

void ADC_autotrigger_init(void);

void ADC_autotrigger_readChannel(u8 CH_num, u16* ptr);



#endif /* ADC_H_ */
