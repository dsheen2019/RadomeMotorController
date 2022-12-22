/*
 * ResolverPositionSensor.h
 *
 *  Created on: Aug 29, 2022
 *      Author: ayeiser
 */

#ifndef INC_RESOLVERPOSITIONSENSOR_H_
#define INC_RESOLVERPOSITIONSENSOR_H_

#include <PositionSensor.h>
#include "tim_delay.h"

class ResolverPositionSensor : public PositionSensor {
protected:
	SPI_HandleTypeDef* hspi;
public:
	ResolverPositionSensor(SPI_HandleTypeDef *_hspi,
			float _dt,
			uint32_t _pole_count,
			uint32_t _elec_offset = 0u,
			uint32_t _scale = 1u) :
				PositionSensor(_dt, _pole_count, _elec_offset, _scale),
				hspi(_hspi) {}
	bool update();
	void setup() {
		HAL_GPIO_WritePin(RESOLVER_RESET_GPIO_Port, RESOLVER_RESET_Pin, GPIO_PIN_RESET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(RESOLVER_RESET_GPIO_Port, RESOLVER_RESET_Pin, GPIO_PIN_SET);
		HAL_Delay(20);
	}
};

#endif /* INC_RESOLVERPOSITIONSENSOR_H_ */
