/*
 * cordic_math.h
 *
 *  Created on: Aug 28, 2022
 *      Author: ayeiser
 */

#ifndef INC_CORDIC_MATH_H_
#define INC_CORDIC_MATH_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void cordic_sincos(uint32_t, float*, float*);
float cordic_modulus(float, float);
void cordic_modphase(float, float, float*, uint32_t*);
float cordic_sqrt(float, float);


#ifdef __cplusplus
}
#endif

#endif /* INC_CORDIC_MATH_H_ */
