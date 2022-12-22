/*
 * CurrentSensor.cpp
 *
 *  Created on: Aug 14, 2022
 *      Author: ayeiser
 */

#include <CurrentSensor.h>

CurrentSensor::~CurrentSensor() {
	HAL_ADCEx_MultiModeStop_DMA(hadc1);
	delete [] adc_buf1;
}

CurrentSensorDualADC::~CurrentSensorDualADC() {
	HAL_ADCEx_MultiModeStop_DMA(hadc2);
	delete [] adc_buf2;
}

void CurrentSensorDualADC_BigDyno::mapBufValues() {
	currents.u = current_scale * float(adc_buf1[0] - 2048);
	currents.v = current_scale * float(adc_buf2[0] - 2048);
	currents.w = current_scale * float(adc_buf2[1] - 2048);

	vbus = vbus_scale * float(adc_buf2[2]); // TODO check scaling here!
	vlogic = vlogic_scale * float(adc_buf1[1]);
	temp = temp_scale * float(adc_buf1[2]);
}

void CurrentSensorDualADC_BackEMF::mapBufValues() {
	int32_t uvw_mean_acc[3] = {0, 0, 0};
	int32_t vbus_mean_acc = 0;
	int32_t uvw_slope_acc[3] = {0, 0, 0};
	int32_t x_acc = 0;
	int32_t x = 1 - reps;
	int16_t *buf_ptr1 = (int16_t*) adc_buf1;
	int16_t *buf_ptr2 = (int16_t*) adc_buf2;
	while (x < reps) {
		x_acc += x*x;
		uvw_mean_acc[0] += (buf_ptr1[0] - 2048);
		uvw_slope_acc[0] += (buf_ptr1[0] - 2048) * x;

		uvw_mean_acc[1] += (buf_ptr2[1] - 2048);
		uvw_slope_acc[1] += (buf_ptr2[1] - 2048) * x;

		uvw_mean_acc[2] += (buf_ptr2[0] - 2048);
		uvw_slope_acc[2] += (buf_ptr2[0] - 2048) * x;

		vbus_mean_acc += buf_ptr1[1];

		buf_ptr1 += 2;
		buf_ptr2 += 2;
//		uvw_mean_acc[0] += (*buf_ptr1 - 2048);
//		uvw_slope_acc[0] += (*(buf_ptr1++) - 2048) * x;

//		uvw_mean_acc[1] += (*buf_ptr1 - 2048);
//		uvw_slope_acc[1] += (*(buf_ptr1++) - 2048) * x;

//		uvw_mean_acc[2] += (*buf_ptr2 - 2048);
//		uvw_slope_acc[2] += (*(buf_ptr2++) - 2048) * x;

//		vbus_mean_acc += *(buf_ptr2++);
		x += 2;
	}

	float mean_scale = current_scale / float(reps);
	float slope_scale = 2.0f * current_scale *
			(float(CLOCK_FREQ) / (float(ADC_CLK_SCALE) * float(ADC_SEQ_LEN))) /
					float(x_acc);

	currents.u = -float(uvw_mean_acc[0]) * mean_scale;
	currents.v = -float(uvw_mean_acc[1]) * mean_scale;
	currents.w = -float(uvw_mean_acc[2]) * mean_scale;
	vbus = float(vbus_mean_acc) * vbus_scale / float(reps);

	slopes.u = -float(uvw_slope_acc[0]) * slope_scale;
	slopes.v = -float(uvw_slope_acc[1]) * slope_scale;
	slopes.w = -float(uvw_slope_acc[2]) * slope_scale;
}
