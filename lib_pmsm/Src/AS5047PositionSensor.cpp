/*
 * AS5047PositionSensor.cpp
 *
 *  Created on: Aug 29, 2022
 *      Author: ayeiser
 */

#include <AS5047PositionSensor.h>

void AS5047PositionSensor::setup() {
	update();
	elec_vel = mech_vel = 0.0f;
}

bool AS5047PositionSensor::update() {
	uint8_t tx_buf[2] = {0xff, 0xff};
	uint8_t rx_buf[2];
    CLEAR_BIT(hspi->Instance->CR2, SPI_RXFIFO_THRESHOLD);
	__HAL_SPI_ENABLE(hspi);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	for (volatile int i = 0; i < 10; i++);
	int tx_len = 1;
	int rx_len = 1;
	while (tx_len || rx_len) {

		if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE) && tx_len > 0) {
			hspi->Instance->DR = *((uint16_t *) tx_buf);
			tx_len--;
		}
		if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE) && rx_len > 0) {
			*((uint16_t*) rx_buf) = hspi->Instance->DR;
			rx_len--;
		}

	}
	__HAL_SPI_DISABLE(hspi);

//	HAL_SPI_TransmitReceive(hspi, tx_buf, rx_buf, 2, 1);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

//	asm("nop"); asm("nop"); asm("nop");
//
//	CLEAR_BIT(hspi->Instance->CR2, SPI_RXFIFO_THRESHOLD);
//	__HAL_SPI_ENABLE(hspi);
//	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
//	for (volatile int i = 0; i < 10; i++);
//	tx_len = 1;
//	rx_len = 1;
//	while (tx_len || rx_len) {
//
//		if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE) && tx_len > 0) {
//			hspi->Instance->DR = *((uint16_t *) tx_buf);
//			tx_len--;
//		}
//		if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE) && rx_len > 0) {
//			*((uint16_t*) rx_buf) = hspi->Instance->DR;
//			rx_len--;
//		}
//
//	}
//	__HAL_SPI_DISABLE(hspi);

	//	HAL_SPI_TransmitReceive(hspi, tx_buf, rx_buf, 2, 1);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
//
//	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
//	asm("nop"); asm("nop"); asm("nop");
//	HAL_SPI_TransmitReceive(hspi, tx_buf, rx_buf, 2, 1);
//	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	uint32_t new_angle = (rx_buf[0] << 8 | rx_buf[1]) << 18;

	int32_t vel_diff = new_angle - mech_angle;
	tracking = true;

//	const float beta = MIN(0.1f, 500.0f * dt);   // 2 ms convergence time
//	mech_vel *= (1.0f - beta);
//	mech_vel += calcVelocity(vel_diff) * beta;
	mech_vel = lpf.update(calcVelocity(vel_diff));
	elec_vel = mech_vel * float(pole_count);

	mech_angle = new_angle;
	elec_angle = mech_angle * pole_count - elec_offset;
	return true;
}
