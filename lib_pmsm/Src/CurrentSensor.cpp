/*
 * CurrentSensor.cpp
 *
 *  Created on: Aug 14, 2022
 *      Author: ayeiser
 */

#include <CurrentSensor.h>

bool ADCReader::ready1 = false;
bool ADCReader::ready2 = false;

uint16_t ADCReader::adc_buf1[64];
uint16_t ADCReader::adc_buf2[64];

extern ADC_HandleTypeDef hadc1, hadc2;

ADC_HandleTypeDef* ADCReader::hadc_ch1 = &hadc1;
ADC_HandleTypeDef* ADCReader::hadc_ch2 = &hadc2;

const int ADCReader::samples = 2;
const int ADCReader::reps = ADC_SAMPLES;

ADCReader::ADCReader(ADC_HandleTypeDef *_hadc, int _seq, float _scale, int16_t _offset) :
scale(_scale), offset(_offset)
{
	if (( _seq >= 0 ) && ( _seq < samples)) {
		if (_hadc == hadc_ch1) {
			adc_val_ptr = adc_buf1 + _seq;
		} else if (_hadc == hadc_ch2) {
			adc_val_ptr = adc_buf2 + _seq;
		} else {
			adc_val_ptr = nullptr;
		}
	} else {
		adc_val_ptr = nullptr;
	}
}

float ADCReader::computeAverage() const {
	int32_t mean_acc = 0;
	for (uint16_t* ptr = adc_val_ptr; ptr < adc_val_ptr + samples*reps; ptr += samples) {
		mean_acc += (*ptr - offset);
	}
	return float(mean_acc) * (scale / float(reps));
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	ADCReader::callback(hadc);
}
