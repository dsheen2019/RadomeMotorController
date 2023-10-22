/*
 * CurrentSensor.h
 *
 *  Created on: Aug 14, 2022
 *      Author: ayeiser
 */

#ifndef SRC_CURRENTSENSOR_H_
#define SRC_CURRENTSENSOR_H_

#include "main.h"
#include "angle.h"
#include "string.h"

class ADCReader {
protected:
	static const int samples;
	static const int reps;
	static ADC_HandleTypeDef *hadc_ch1;
	static ADC_HandleTypeDef *hadc_ch2;
	static uint16_t adc_buf1[64];
	static uint16_t adc_buf2[64];
	static bool ready1;
	static bool ready2;

	uint16_t* adc_val_ptr;
	const float scale;
	const int16_t offset;
public:
	ADCReader() : adc_val_ptr(nullptr), scale(1.0f), offset(0) {};
	ADCReader(ADC_HandleTypeDef *_hadc, int _seq, float _scale, int16_t _offset = 0);

	static void start() {
		HAL_Delay(100);

		HAL_ADCEx_Calibration_Start(hadc_ch1, ADC_SINGLE_ENDED);
		HAL_ADCEx_Calibration_Start(hadc_ch2, ADC_SINGLE_ENDED);
		HAL_Delay(100);

		HAL_ADC_Start_DMA(hadc_ch1, (uint32_t*) adc_buf1, samples*reps);
		HAL_ADC_Start_DMA(hadc_ch2, (uint32_t*) adc_buf2, samples*reps);
	}

	static void callback(ADC_HandleTypeDef *hadc) {
		if (hadc == hadc_ch1) {
			hadc->Instance->CR |= ADC_CR_ADSTART;
			ready1 = true;
			return;
		}
		if (hadc == hadc_ch2) {
			hadc->Instance->CR |= ADC_CR_ADSTART;
			ready2 = true;
			return;
		}
	}

	static bool isReady() {
		return ready1 && ready2;
	}

	static void clearReadyFlag() {
		ready1 = ready2 = false;
	}

	static bool waitUntilReady(TIM_HandleTypeDef *htim, uint32_t timeout) {
		while (isReady() && (htim->Instance->CNT < timeout));
		return isReady();
	}

	float computeAverage() const;
//	float computeSlope();
};

class CurrentSensor {
protected:
	ADCReader &phaseU, &phaseV, &phaseW, &vbusMeas;

	vect_uvw currents;
	vect_uvw slopes;
	float vbus;

public:
	CurrentSensor(ADCReader &_phaseU, ADCReader &_phaseV, ADCReader &_phaseW, ADCReader &_vbusMeas) :
		phaseU(_phaseU), phaseV(_phaseV), phaseW(_phaseW), vbusMeas(_vbusMeas),
		currents{0.0f, 0.0f, 0.0f}, slopes{0.0f, 0.0f, 0.0f}, vbus(0.0f) {}

	bool update(TIM_HandleTypeDef *htim, uint32_t timeout = 0xff) {
		bool dataGood = ADCReader::waitUntilReady(htim, timeout);
		currents.u = phaseU.computeAverage();
		currents.v = phaseV.computeAverage();
		currents.w = phaseW.computeAverage();

		vbus = vbusMeas.computeAverage();

		ADCReader::clearReadyFlag();
		return dataGood;
	}
	vect_uvw getCurrents() const {return currents;}
	vect_uvw getCurrentSlopes() const {return slopes;}
	float getVBUS() const {return vbus;}
};


#endif /* SRC_CURRENTSENSOR_H_ */
