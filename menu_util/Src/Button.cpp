/*
 * Button.cpp
 *
 *  Created on: Dec 27, 2022
 *      Author: ayeiser
 */

#include <Button.h>

bool Button::update() {
	bool recent = !HAL_GPIO_ReadPin(port, pin);

	if (state == recent) {
		ctr = 0;
		if (++repeat_ctr > REPEAT_TIME) {
			repeat_ctr = REPEAT_TIME - REPEAT_INTERVAL;
			if (state) {
				rising_flag = true;
			} else {
				falling_flag = true;
			}
		}
	} else {
		repeat_ctr = 0;
		if (++ctr > SETTLING_TIME) {
			state = recent;
			if (state) {
				rising_flag = true;
			} else {
				falling_flag = true;
			}
		}
	}

	return state;
}

bool Button::getRisingEdge() {
	bool flag = rising_flag;
	rising_flag = false;
	return flag;
}

bool Button::getFallingEdge() {
	bool flag = falling_flag;
	falling_flag = false;
	return flag;
}
