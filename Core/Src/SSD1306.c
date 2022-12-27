#include <SSD1306.h>

// I2C is stupid. We can only transfer upt to 255 bytes at a time, but we need header shit
// so we will transfer the data to the screen one row at a time
// luckily there are only 8 rows
// this is OK because it keeps the frame buf short anyways
// and makes less code per interrupt

uint8_t framebuf[COLUMNS+FRAME_BUF_OFFSET];   // add control commands. Only a single row is buffered at a time.
int timeout_cnt;
uint8_t _char_width;
uint8_t _font;

SPI_HandleTypeDef *ssd1306_spi;

HAL_StatusTypeDef SSD1306_InitScreen(SPI_HandleTypeDef *hspi) {   // init screen

	ssd1306_spi = hspi;

	int _vccstate = 0;

	_vccstate = SSD1306_SWITCHCAPVCC;
	//GPIOA->ODR |= GPIO_ODR_8;
//	SSD1306_command1(0xA5);


	// Init sequence
	HAL_StatusTypeDef retval = SSD1306_command1(SSD1306_DISPLAYOFF);
	if (retval == HAL_OK) {                   // 0xAE
		// if first command doesn't work, give up

		SSD1306_command1(SSD1306_SETDISPLAYCLOCKDIV);            // 0xD5
		SSD1306_command1(0x80);                                  // the suggested ratio 0x80

		SSD1306_command1(SSD1306_SETMULTIPLEX);                  // 0xA8
		SSD1306_command1(SSD1306_LCDHEIGHT - 1);

		SSD1306_command1(SSD1306_SETDISPLAYOFFSET);              // 0xD3
		SSD1306_command1(0x0);                                   // no offset
		SSD1306_command1(SSD1306_SETSTARTLINE | 0x0);            // line #0
		SSD1306_command1(SSD1306_CHARGEPUMP);                    // 0x8D

		if (_vccstate == SSD1306_EXTERNALVCC){
			SSD1306_command1(0x10);
		} else {
			SSD1306_command1(0x14);
		}

		SSD1306_command1(SSD1306_MEMORYMODE);                    // 0x20
		SSD1306_command1(0x00);                                  // 0x0 act like ks0108
		SSD1306_command1(SSD1306_SEGREMAP | 0x1);
		SSD1306_command1(SSD1306_COMSCANDEC);

		SSD1306_command1(SSD1306_SETCOMPINS);                    // 0xDA
		SSD1306_command1(0x12);
		SSD1306_command1(SSD1306_SETCONTRAST);                   // 0x81

		if (_vccstate == SSD1306_EXTERNALVCC){
			SSD1306_command1(0x9F);
		} else {
			SSD1306_command1(0xCF);
		}

		SSD1306_command1(SSD1306_SETPRECHARGE);                  // 0xd9

		if (_vccstate == SSD1306_EXTERNALVCC) {
			SSD1306_command1(0x22);
		} else {
			SSD1306_command1(0xF1);
		}

		SSD1306_command1(SSD1306_SETVCOMDETECT);                 // 0xDB
		SSD1306_command1(0x40);
		SSD1306_command1(SSD1306_DISPLAYALLON_RESUME);           // 0xA4
		SSD1306_command1(SSD1306_NORMALDISPLAY);                 // 0xA6

		SSD1306_command1(SSD1306_DEACTIVATE_SCROLL);

		SSD1306_command1(SSD1306_DISPLAYON);//--turn on oled panel
		HAL_Delay(10);


		SSD1306_clearDisplay();
	}
	return retval;

//	SSD1306_SetupFrameBuf();   // write initial conditions

//	_char_width = FONT8x8_WIDTH;
//	_font = FONT_8x8;
//	timeout_cnt = TIMEOUT_INIT;
}

// I2C stuff

HAL_StatusTypeDef SSD1306_spiWrite( uint8_t *buf, uint16_t num_bytes) {
	return HAL_SPI_Transmit( ssd1306_spi, buf, num_bytes, HAL_MAX_DELAY );
}

//HAL_StatusTypeDef SSD1306_i2cWrite( uint8_t *buf, uint16_t num_bytes) {
//	return HAL_I2C_Master_Transmit(ssd1306_i2c, SSD1306_SLAVE_ADDR,
//			buf, num_bytes, HAL_MAX_DELAY);
//}


//HAL_StatusTypeDef SSD1306_DMAi2cWrite( uint8_t *buf, uint16_t num_bytes) {
//	return HAL_I2C_Master_Transmit_DMA(ssd1306_i2c, SSD1306_SLAVE_ADDR,
//			buf, num_bytes);
//
//}

HAL_StatusTypeDef SSD1306_command1(uint8_t c) {
	HAL_GPIO_WritePin(SCREEN_DATASEL_GPIO_Port, SCREEN_DATASEL_Pin, GPIO_PIN_RESET);
	return SSD1306_spiWrite(&c, 1);
}

HAL_StatusTypeDef SSD1306_sendCommand(uint8_t command, uint8_t param1, uint8_t param2) {
	HAL_GPIO_WritePin(SCREEN_DATASEL_GPIO_Port, SCREEN_DATASEL_Pin, GPIO_PIN_RESET);
	//  Note continuationbit is set, so COMMAND_MODE must be
	//  repeated before each databyte that serves as parameter!

	uint8_t databytes[3];

	databytes[0] = command;
	databytes[1] = param1;
	databytes[2] = param2;
	return SSD1306_spiWrite(databytes, 3);    // Write command
}

HAL_StatusTypeDef SSD1306_sendDataByte(uint8_t data){
	HAL_GPIO_WritePin(SCREEN_DATASEL_GPIO_Port, SCREEN_DATASEL_Pin, GPIO_PIN_SET);
	return SSD1306_spiWrite(&data, 1);
}

HAL_StatusTypeDef SSD1306_sendData(uint8_t* data, uint16_t len){
	HAL_GPIO_WritePin(SCREEN_DATASEL_GPIO_Port, SCREEN_DATASEL_Pin, GPIO_PIN_SET);
	return SSD1306_spiWrite(data, len);
}

HAL_StatusTypeDef SSD1306_setPageAddress(uint8_t start, uint8_t end) {
	return SSD1306_sendCommand(SET_PAGE_ADDRESS, start, end);
}
HAL_StatusTypeDef SSD1306_setColumnAddress(uint8_t start, uint8_t end) {
	return SSD1306_sendCommand(SET_COLUMN_ADDRESS, start, end);
}
HAL_StatusTypeDef SSD1306_writeBuf(uint8_t* buf) {
	SSD1306_setPageAddress(0, MAX_PAGE);
	SSD1306_setColumnAddress(0, MAX_COL);
	HAL_GPIO_WritePin(SCREEN_DATASEL_GPIO_Port, SCREEN_DATASEL_Pin, GPIO_PIN_SET);
	return SSD1306_spiWrite(buf, PAGES*COLUMNS);

}
// Standard version
void SSD1306_clearDisplay() {

	//setDisplayOff();
	SSD1306_setPageAddress(0, MAX_PAGE);  // all pages
	SSD1306_setColumnAddress(0, MAX_COL); // all columns

	for (uint8_t page = 0; page < PAGES; page++) {
		for (uint8_t col = 0; col < COLUMNS; col++) {
			SSD1306_sendDataByte(0x00);
		}
	}

}
