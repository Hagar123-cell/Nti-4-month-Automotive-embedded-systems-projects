/*
 * smallOS.h
 *
 *  Created on: Nov 17, 2023
 *      Author: OMR
 */

#ifndef SMALLOS_H_
#define SMALLOS_H_

#include"STDTYPES.h"
typedef struct {
	u32 periodicity;
	void (*pfun) (void);
}task_t;


 void sechulder(void);
#endif /* SMALLOS_H_ */
