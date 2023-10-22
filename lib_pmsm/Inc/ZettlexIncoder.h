/*
 * ZettlexIncoder.h
 *
 *  Created on: Sep 24, 2023
 *      Author: ayeiser
 */

#ifndef INC_ZETTLEXINCODER_H_
#define INC_ZETTLEXINCODER_H_

#include "main.h"

#define ZETTLEX_ZPD_FLAG (1 << 2)
#define ZETTLEX_PS_FLAG (1 << 1)
#define ZETTLEX_SD_FLAG (1 << 0)

#define ZETTLEX_PV_FLAG (1 << 1)
#define ZETTLEX_CRC_FLAG (1 << 0)

class ZettlexIncoder {
protected:
	SPI_HandleTypeDef* hspi;
	uint8_t spi_rx_buf[6];

	bool zpd, pv, ps, sd, crc;

	uint32_t position;


public:
	ZettlexIncoder(SPI_HandleTypeDef* _hspi) :
		hspi(_hspi),
		zpd(0), pv(0), ps(0), sd(0), crc(0), position(0) {}

	void update();
	bool isValid() const { return crc && pv; }
	bool check_crc(uint32_t, uint8_t) const;

	uint32_t getAngle() const { return position; }


};

#endif /* INC_ZETTLEXINCODER_H_ */
