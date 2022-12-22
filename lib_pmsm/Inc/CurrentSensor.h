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

class CurrentSensor {
protected:
	ADC_HandleTypeDef *hadc1;
	const float current_scale;
	const float vbus_scale;
	uint16_t* adc_buf1;
	const uint16_t seqlen;
	const uint16_t reps;
	const uint16_t buflen;

	volatile bool adc_ready1;
	bool dataGood;

	vect_uvw currents;
	vect_uvw slopes;
	float vbus;

public:
	CurrentSensor(ADC_HandleTypeDef *_hadc1,
			float _current_scale, float _vbus_scale,
			uint16_t _seqlen, uint16_t _reps = 1) :
				hadc1(_hadc1), current_scale(_current_scale),
				vbus_scale(_vbus_scale), seqlen(_seqlen),
				reps(_reps), buflen(_seqlen * _reps) {
		adc_buf1 = new uint16_t[buflen];
		memset(adc_buf1, 0, sizeof(uint16_t)*buflen);

	}
	~CurrentSensor();

	void setup() {
		HAL_ADC_Start_DMA(hadc1, (uint32_t*) adc_buf1, buflen);
	}

	void retriggerADC(ADC_HandleTypeDef *hadc) {
		if (hadc->Instance == ADC1) {
			hadc->Instance->CR |= ADC_CR_ADSTART;
			adc_ready1 = true;
		}
	}

	bool adcReady() {return adc_ready1;}
	void ADCFlagReset() {adc_ready1 = false;}
	void waitForADC(uint32_t ticks = 0xffffffff) {
		uint32_t tick = 0;
		while (!adcReady() && ++tick < ticks);
		dataGood = adcReady();
	}

	// Unique for every current sensor
	// no good way of characterizing ADC mapping
	virtual void mapBufValues() = 0;

	bool update(uint32_t ticks = 0xff) {
		waitForADC(ticks);
		mapBufValues();
		ADCFlagReset();
		return dataGood;
	}
	vect_uvw getCurrents() {return currents;}
	vect_uvw getCurrentSlopes() {return slopes;}
	float getVBUS() {return vbus;}
};

class CurrentSensorDualADC : public CurrentSensor {
protected:
	ADC_HandleTypeDef *hadc2;
	uint16_t* adc_buf2;
	volatile bool adc_ready2;
public:
	CurrentSensorDualADC(ADC_HandleTypeDef *_hadc1, ADC_HandleTypeDef *_hadc2,
			float _current_scale, float _vbus_scale,
			uint16_t _seqlen, uint16_t _reps = 1) :
				CurrentSensor(_hadc1, _current_scale, _vbus_scale, _seqlen, _reps),
				hadc2(_hadc2) {
		adc_buf2 = new uint16_t[buflen];
		memset(adc_buf2, 0, sizeof(uint16_t)*buflen);
	}
	~CurrentSensorDualADC();
	void setup() {
		HAL_ADC_Start_DMA(hadc1, (uint32_t*) adc_buf1, buflen);
		HAL_ADC_Start_DMA(hadc2, (uint32_t*) adc_buf2, buflen);
	}

	void retriggerADC(ADC_HandleTypeDef *hadc) {
		if (hadc->Instance == ADC1) {
			hadc->Instance->CR |= ADC_CR_ADSTART;
			adc_ready1 = true;
		}
		else if (hadc->Instance == ADC2) {
			hadc->Instance->CR |= ADC_CR_ADSTART;
			adc_ready2 = true;
		}
	}

	bool adcReady() {return adc_ready1 && adc_ready2;}
	void ADCFlagReset() {adc_ready1 = adc_ready2 = false;}
};

class CurrentSensorDualADC_BigDyno : public CurrentSensorDualADC {
protected:
	const float vlogic_scale;
	const float temp_scale;
	float vlogic;
	float temp;
public:
	CurrentSensorDualADC_BigDyno(ADC_HandleTypeDef *_hadc1, ADC_HandleTypeDef *_hadc2,
			float _current_scale, float _vbus_scale, float _vlogic_scale, float _temp_scale) :
				CurrentSensorDualADC(_hadc1, _hadc2, _current_scale, _vbus_scale, 3, 1),
				vlogic_scale(_vlogic_scale), temp_scale(_temp_scale){}
	void mapBufValues();
	float getVlogic() {return vlogic;}
	float getTemp() {return temp;}
};

class CurrentSensorDualADC_BackEMF : public CurrentSensorDualADC {
public:
	CurrentSensorDualADC_BackEMF(ADC_HandleTypeDef *_hadc1, ADC_HandleTypeDef *_hadc2,
			float _current_scale, float _vbus_scale, float _reps) :
				CurrentSensorDualADC(_hadc1, _hadc2, _current_scale, _vbus_scale, 2, _reps) {}
	void mapBufValues();
};


#endif /* SRC_CURRENTSENSOR_H_ */
