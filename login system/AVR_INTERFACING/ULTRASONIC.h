/*
 * ULTRASONIC.h
 *
 *  Created on: Nov 15, 2023
 *      Author: OMR
 */

#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

#include"STDTYPES.h"




/*
 * Description :
 * initialize the ultrasonic Driver.
 */
void Ultrasonic_init(void);


/*
 * Description :
 * send the trigger pulse to the ultrasonic.
 */
void Ultrasonic_Trigger(void);


/*
 * Description :
 * Send the trigger pulse by using Ultrasonic_Trigger function.
 * Start the measurements by the ICU from this moment.
 */
u16 Ultrasonic_readDistance(void);


#endif /* ULTRASONIC_H_ */
