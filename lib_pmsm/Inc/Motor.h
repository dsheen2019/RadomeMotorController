/*
 * Motor.h
 *
 *  Created on: Aug 14, 2022
 *      Author: ayeiser
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "PositionSensor.h"
#include "CurrentSensor.h"
#include "pwm.h"

class Motor {
protected:
	PositionSensor &position;
	CurrentSensor &current;
	PWMGenerator &pwm;
public:
	Motor(PositionSensor &_pos, CurrentSensor &_cur, PWMGenerator &_pwm) :
		position(_pos), current(_cur), pwm(_pwm){};
	virtual ~Motor();
	void setup();
	void update();
};

#endif /* MOTOR_H_ */
