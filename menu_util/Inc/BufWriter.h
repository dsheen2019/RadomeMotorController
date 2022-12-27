/*
 * BufWriter.h
 *
 *  Created on: Sep 21, 2022
 *      Author: ayeiser
 */

#ifndef BUFWRITER_H_
#define BUFWRITER_H_

#include "fonts.h"
#include "stdint.h"
#include "string.h"
#include "stdio.h"

class BufWriter {
private:
	uint8_t* buf; // 128x64 screen-->8 rows, 128 columns
	const uint16_t rows;
	const uint16_t cols;
	uint16_t row;
	uint16_t col;
public:
	BufWriter(uint8_t* _buf, uint16_t _rows, uint16_t _cols):
		buf(_buf), rows(_rows), cols(_cols) {}

	void clear();
	bool write(uint16_t, uint16_t, uint8_t);
	void writeChar6x8(char);
	void writeChar8x16(char);
	void writeStr6x8(char*, uint8_t);
	void writeStr8x16(char*, uint8_t);
	void writeLine6x8(char* _str, uint8_t _len=32) {
		writeStr6x8(_str, _len); row++; col=0;
	}
	void writeLine8x16(char* _str, uint8_t _len=32) {
		writeStr8x16(_str, _len); row+=2; col=0;
	}
	void newline() {row++; col=0;}
};

#endif /* BUFWRITER_H_ */
