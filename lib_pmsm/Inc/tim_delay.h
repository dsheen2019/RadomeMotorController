/*
 * tim_delay.h
 *
 *  Created on: Sep 4, 2022
 *      Author: ayeiser
 */

#ifndef INC_TIM_DELAY_H_
#define INC_TIM_DELAY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

void start_delay_timer(TIM_HandleTypeDef*);
void delay_ticks(uint16_t);

#ifdef __cplusplus
}
#endif

#endif /* INC_TIM_DELAY_H_ */
