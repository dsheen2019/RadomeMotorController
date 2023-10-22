/*
 * ZettlexIncoder.cpp
 *
 *  Created on: Sep 24, 2023
 *      Author: ayeiser
 */

#include <ZettlexIncoder.h>
#include "string.h"

void ZettlexIncoder::update() {

	zpd = spi_rx_buf[1] & 0x01; // zero point default (1 is default)
	pv = spi_rx_buf[2] & 0x80; // position valid (1 is valid)
	ps = spi_rx_buf[2] & 0x40; // Position synchronized flag (1 is synchronized)
	uint32_t pos_buf = (uint32_t(spi_rx_buf[2] & 0x3F) << 16
			| (uint32_t(spi_rx_buf[3]) << 8)
			| uint32_t(spi_rx_buf[4])) << 14;
	sd = spi_rx_buf[5] & 0x80; // Stale data flag (1 is stale)
	uint32_t crc_data = (((uint32_t)spi_rx_buf[5] >> 7) |
			((uint32_t)spi_rx_buf[4] << 1) |
			(uint32_t)spi_rx_buf[3] << 9 |
			(uint32_t)spi_rx_buf[2] << 17 |
			(((uint32_t)spi_rx_buf[1] & 0x01) << 25)
	);
	crc = check_crc(crc_data, spi_rx_buf[5] & 0x7F);

	if (crc && pv) {
		position = pos_buf;
	}
//	memset(spi_rx_buf, 0, 6);


	HAL_SPI_Receive_DMA(hspi, spi_rx_buf, 6);

}

bool ZettlexIncoder::check_crc(uint32_t value, uint8_t crc) const {
	uint32_t poly = 0xDB000000;
	for(int i = 0; i < 31; i++){
		if(value & (0x80000000)){
			value = value ^ poly;
		}
		value = value << 1;
	}
	if(value & (0x80000000)){
		value = value ^ poly;
	}
	return (value >> 24) == crc;
}
