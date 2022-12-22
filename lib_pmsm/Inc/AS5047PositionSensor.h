/*
 * AS5047PositionSensor.h
 *
 *  Created on: Aug 29, 2022
 *      Author: ayeiser
 */

#ifndef INC_AS5047POSITIONSENSOR_H_
#define INC_AS5047POSITIONSENSOR_H_

#include <PositionSensor.h>
//#include "stm32g4xx_ll_spi.h"

class AS5047PositionSensor : public PositionSensor {
protected:
	SPI_TypeDef* hspi;
public:
	AS5047PositionSensor(SPI_TypeDef* _hspi,
			float _dt,
			uint32_t _cnt,
			uint32_t _off = 0u,
			uint32_t _scale = 1u) :
				PositionSensor(_dt, _cnt, _off, _scale), hspi(_hspi) {};
	bool update();
	uint32_t getElecAngle();
	uint32_t getMechAngle();
	float getElecVelocity();
	float getMechVelocity();
};

#endif /* INC_AS5047POSITIONSENSOR_H_ */
