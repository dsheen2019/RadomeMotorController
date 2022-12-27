/*
 * BufWriter.cpp
 *
 *  Created on: Sep 21, 2022
 *      Author: ayeiser
 */

#include "BufWriter.h"

extern unsigned char console_font_6x8[];
extern unsigned char console_font_8x16[];


void BufWriter::clear() {
	row = col = 0;
	memset(buf, 0, rows*cols);
}

bool BufWriter::write(uint16_t _row, uint16_t _col, uint8_t _data) {
	if(_row < rows && _col < cols) {
		buf[_row*cols + _col] = _data;
		return true;
	} else {
		return false;
	}
}

void BufWriter::writeChar6x8(char _c) {
	uint16_t offset = (_c - FONT_OFFSET)*FONT_6x8_GLYPHLEN;
	for (uint8_t i = 0; i < FONT_6x8_GLYPHLEN; i++, col++) {
		write(row, col, console_font_6x8[offset+i]);
	}
}

void BufWriter::writeChar8x16(char _c) {
	uint16_t offset = (_c - FONT_OFFSET)*FONT_8x16_GLYPHLEN;
	for (uint8_t i = 0; i < 8; i+=1, col++) {
		write(row, col, console_font_8x16[offset+i]);
		write(row+1, col, console_font_8x16[offset+i+8]);
	}
}

void BufWriter::writeStr6x8(char* str, uint8_t _len = 22) {
	for (uint8_t i = 0; i < _len; i++) {
		if(str[i] == 0) {
			break;
		}
		writeChar6x8(str[i]);
	}
}

void BufWriter::writeStr8x16(char* str, uint8_t _len = 11) {
	for (uint8_t i = 0; i < _len; i++) {
		if(str[i] == 0) {
			break;
		}
		writeChar8x16(str[i]);
	}
}

