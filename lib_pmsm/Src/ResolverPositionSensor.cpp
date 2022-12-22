/*
 * ResolverPositionSensor.cpp
 *
 *  Created on: Aug 29, 2022
 *      Author: ayeiser
 */

#include <ResolverPositionSensor.h>

//ResolverPositionSensor::~ResolverPositionSensor() {
//	// TODO Auto-generated destructor stub
//}

bool ResolverPositionSensor::update() {
	RESOLVER_SAMPLE_GPIO_Port->BRR = RESOLVER_SAMPLE_Pin; // set sample low to start read operation
	RESOLVER_CS_GPIO_Port->BRR = RESOLVER_CS_Pin; // set CS pin to low
	RESOLVER_RDVEL_GPIO_Port->BSRR = RESOLVER_RDVEL_Pin; // set RDVEL high to read position
	RESOLVER_RD_GPIO_Port->BSRR = RESOLVER_RD_Pin; // set RD high

	delay_ticks(170); // 1 us delay

	// CPOL = CPHA = 0

	RESOLVER_RD_GPIO_Port->BRR = RESOLVER_RD_Pin; // set RD low, start SPI transaction
	uint16_t pos;
	HAL_SPI_Receive(&hspi, (uint8_t*) &pos, 1, 1);
	RESOLVER_RD_GPIO_Port->BSRR = RESOLVER_RD_Pin; // set RD high, end SPI transaction
	RESOLVER_RDVEL_GPIO_Port->BRR = RESOLVER_RDVEL_Pin; // set RDVEL low to read velocity

	delay_ticks(5); // 30 ns delay

	RESOLVER_RD_GPIO_Port->BRR = RESOLVER_RD_Pin; // set RD low, start SPI transaction
	int16_t vel;
	HAL_SPI_Receive(&hspi, (uint8_t*) &vel, 1, 1);
	RESOLVER_RD_GPIO_Port->BSRR = RESOLVER_RD_Pin; // set RD high, end SPI transaction
	RESOLVER_SAMPLE_GPIO_Port->BSRR = RESOLVER_SAMPLE_Pin; // set sample high
	RESOLVER_CS_GPIO_Port->BSRR = RESOLVER_CS_Pin; // set CS pin high


	elec_angle = ((pos & 0xfff0) << 16) + elec_offset;
	elec_vel = float( int16_t(vel & 0xfff0)) * (4.656612873077393e-2f);  // I *think* this is the correct velocity scale
	mech_vel = elec_vel / pole_count;

	const uint16_t FAULT_MASK = 0x0006;

	if (pos & FAULT_MASK || vel & FAULT_MASK) {
		tracking = false;
	} else {
	// TODO check parity bit (odd parity)
		tracking = true;
	}
	return tracking;
}

