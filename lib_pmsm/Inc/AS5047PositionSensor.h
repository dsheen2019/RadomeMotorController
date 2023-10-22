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

class BiQuadLPF {
protected:
	float wpast1, wpast2;
	float a1, a2, b0, b1, b2;
	float filtered;
public:
	BiQuadLPF(): wpast1(0.0f), wpast2(0.0f) {}
	BiQuadLPF(float _a1, float _a2, float _b0, float _b1, float _b2) :
		wpast1(0.0f), wpast2(0.0f),
		a1(_a1), a2(_a2), b0(_b0), b1(_b1), b2(_b2), filtered(0.0f){}
	void setButter2(float dt, float omega) {
		float T = 1.0f / (dt * omega);
		float T2 = T*T;
		float damping = 0.70710678f; // sqrt 0.5

		float a0 = 1.0f + 4.0f*T*damping + 4*T2;
		a1 = 2.0f*(1.0f - 4.0f*T2) / a0;
		a2 = (1.0f - 4.0f*T*damping + 4.0f*T2) / a0;
		b0 = 1.0f / a0;
		b1 = 2.0f / a0;
		b2 = 1.0f / a0;
	}
	BiQuadLPF(float dt, float omega) : BiQuadLPF() {
		setButter2(dt, omega);
	}

	float update(float x) {
		float w = x - wpast1 * a1 - wpast2 * a2;
		filtered = b0 * w + b1 * wpast1 + b2 * wpast2;
		wpast2 = wpast1;
		wpast1 = w;
		return filtered;
	}

	float get() const { return filtered; }

};

class AS5047PositionSensor : public PositionSensor {
protected:
	SPI_HandleTypeDef* hspi;
	BiQuadLPF lpf;
public:
	AS5047PositionSensor(SPI_HandleTypeDef* _hspi,
			float _dt,
			uint32_t _cnt,
			uint32_t _off = 0u,
			uint32_t _scale = 1u) :
				PositionSensor(_dt, _cnt, _off, 1.0f, _scale), hspi(_hspi), lpf(_dt, 1000.0f) {};
	void setup();
	bool update();
};



#endif /* INC_AS5047POSITIONSENSOR_H_ */
