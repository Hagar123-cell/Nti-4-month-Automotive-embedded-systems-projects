/*
 * BIT_MATH.h
 *
 *  Created on: Oct 30, 2023
 *      Author: OMR
 */

#ifndef BIT_MATH_H_
#define BIT_MATH_H_

#define READBIT(REG, BIT)  ((REG >> BIT) & 1)
#define SETBIT(REG, BIT)   (REG |= 1<<BIT)
#define CLEARBIT(REG, BIT) (REG &= ~(1<<BIT))
#define BIT_IS_CLEAR(REG, BIT) ( !(REG & (1<<BIT)) )

#endif /* BIT_MATH_H_ */
