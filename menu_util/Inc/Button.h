/*
 * Button.h
 *
 *  Created on: Dec 27, 2022
 *      Author: ayeiser
 */

#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_

#include "main.h"
#define SETTLING_TIME 120
#define REPEAT_TIME 12000
#define REPEAT_INTERVAL 1200

class Button {
private:
	GPIO_TypeDef* port;
	uint16_t pin;
	uint16_t ctr, repeat_ctr;
	bool state, rising_flag, falling_flag;
public:
	Button(GPIO_TypeDef* _port, uint16_t _pin) :
		port(_port), pin(_pin), ctr(0), repeat_ctr(0),
		state(false), rising_flag(false), falling_flag(false) {};

	bool update();
	bool getRisingEdge();
	bool getFallingEdge();
};

#endif /* INC_BUTTON_H_ */
