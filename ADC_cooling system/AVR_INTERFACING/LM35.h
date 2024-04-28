/*
 * LM35.h
 *
 *  Created on: Nov 9, 2023
 *      Author: OMR
 */

#ifndef LM35_H_
#define LM35_H_

#include"STDTYPES.h"

#define REF_VOLT_ms   5000UL
#define MAX_DIGITAL_VALUE 1023

void LM35_init();
u8 LM35_getTemperature(void);

#endif /* LM35_H_ */
