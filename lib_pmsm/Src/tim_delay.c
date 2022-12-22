/*
 * tim_delay.c
 *
 *  Created on: Sep 4, 2022
 *      Author: ayeiser
 */

#include "tim_delay.h"

TIM_TypeDef* delay_timer; // usually TIM6

void start_delay_timer(TIM_HandleTypeDef* htim) {
	delay_timer = htim->Instance;
	HAL_TIM_Base_Start(htim);
}

void delay_ticks(uint16_t delay) {
	delay_timer->CNT = 0;
	while (delay_timer->CNT < delay);
}
