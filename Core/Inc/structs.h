/*
 * structs.h
 *
 *  Created on: Oct 21, 2023
 *      Author: ayeiser
 */

#ifndef INC_STRUCTS_H_
#define INC_STRUCTS_H_

#include "main.h"

struct motor_command_t {
	uint32_t position;
	float velocity;
	uint32_t last_tick;
};

struct motor_reply_t {
	uint32_t position;
	float velocity;
	float current;
	float vbus;
};


#endif /* INC_STRUCTS_H_ */
