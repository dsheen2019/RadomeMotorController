/*
 * pwm.h
 *
 *  Created on: Aug 15, 2022
 *      Author: ayeiser
 */

#ifndef INC_PWM_H_
#define INC_PWM_H_

#include "stm32g4xx_hal.h"
#include "angle.h"
#include "main.h"
#include "constants.h"

class PWMGenerator {
protected:
	TIM_HandleTypeDef *htim;
	TrigAngle angle;
	vect_dq vdq;
	vect_uvw vuvw;
	int16_t u, v, w;
public:
	PWMGenerator(TIM_HandleTypeDef *_htim):
		htim(_htim), angle(),
		vdq{0.0f, 0.0f}, vuvw{0.0f, 0.0f, 0.0f},
		u(0), v(0), w(0) {}
	void startTiming();
	void setTargets(vect_dq& _vdq) {vdq = _vdq;}
	void setAngle(uint32_t);
	void update(float);
	virtual void writePWM() = 0;
	void setPeriod(uint16_t period) {__HAL_TIM_SET_AUTORELOAD(htim, period); }
	void startSwitching();
	void stopSwitching();
	vect_dq getVdq() {return vdq;}
	vect_uvw getVuvw() {return vuvw;}
};

class PWMGenerator_UVW : public PWMGenerator {
public:
	PWMGenerator_UVW(TIM_HandleTypeDef *_htim) :
		PWMGenerator(_htim) {}

	void writePWM() {
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, u);
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, v);
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, w);
	}
};

class PWMGenerator_WVU : public PWMGenerator {
public:
	PWMGenerator_WVU(TIM_HandleTypeDef *_htim) :
		PWMGenerator(_htim) {}

	void writePWM() {
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, u);
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, v);
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, w);
	}
};

#endif /* INC_PWM_H_ */
