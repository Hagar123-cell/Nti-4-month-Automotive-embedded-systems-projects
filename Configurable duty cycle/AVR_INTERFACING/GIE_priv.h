/*
 * GIE_priv.h
 *
 *  Created on: Oct 22, 2023
 *      Author: OMR
 */

#ifndef GIE_PRIV_H_
#define GIE_PRIV_H_

# define sei()  __asm__ __volatile__ ("sei" ::)
# define cli()  __asm__ __volatile__ ("cli" ::)

#endif /* GIE_PRIV_H_ */
