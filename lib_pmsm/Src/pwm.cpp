/*
 * pwm.cpp
 *
 *  Created on: Aug 15, 2022
 *      Author: ayeiser
 */

#include <pwm.h>

void PWMGenerator::startTiming() {
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_4);
	HAL_TIM_Base_Start_IT(htim);
}

void PWMGenerator::startSwitching() {
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);

	HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_3);
}

void PWMGenerator::stopSwitching() {
	HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_3);

	HAL_TIMEx_PWMN_Stop(htim, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(htim, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(htim, TIM_CHANNEL_3);
}
// Low side connected to non-inverting input, PWM mode 1
void PWMGenerator::update(uint32_t _angle, float vbus) {
	const uint16_t period = __HAL_TIM_GET_AUTORELOAD(htim);
	const float volt_to_cmd = -float(period) / vbus; // inversion necessary

	angle.setAngle_int(_angle);
	vuvw = angle.dq_to_uvw(vdq);

	int16_t u_cmd = int16_t(volt_to_cmd * vuvw.u);
	int16_t v_cmd = int16_t(volt_to_cmd * vuvw.v);
	int16_t w_cmd = int16_t(volt_to_cmd * vuvw.w);

	int16_t max_cmd = MAX3(u_cmd, v_cmd, w_cmd);
	int16_t min_cmd = MIN3(u_cmd, v_cmd, w_cmd);

	const int16_t adc_offset = PWM_ADC_TRIG + ADC_SW_OFFSET; // set this a bit nicer

	int16_t midrange = (period - max_cmd - min_cmd + adc_offset) >> 1;

	u = CLIP(u_cmd + midrange, adc_offset, period);
	v = CLIP(v_cmd + midrange, adc_offset, period);
	w = CLIP(w_cmd + midrange, adc_offset, period);
	writePWM();
}

