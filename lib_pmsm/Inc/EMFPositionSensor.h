/*
 * EMFPositionSensor.h
 *
 *  Created on: Sep 7, 2022
 *      Author: ayeiser
 */

#ifndef INC_EMFPOSITIONSENSOR_H_
#define INC_EMFPOSITIONSENSOR_H_

#include <PositionSensor.h>
#include "CurrentSensor.h"
#include "pwm.h"

class EMFPositionSensor: public PositionSensor {
protected:
	CurrentSensor &cur;
	PWMGenerator &pwm;

	TrigAngle angle_est;
	vect_dq Ldq;
	vect_dq emf;
	float R;
	float emf_mag;
	bool reverse;
public:
	EMFPositionSensor(CurrentSensor &_cur,
			PWMGenerator &_pwm,
			float _dt,
			uint32_t _cnt,
			vect_dq _Ldq,
			float _R) :
		PositionSensor(_dt, _cnt, 0, -0.5f), cur(_cur), pwm(_pwm), angle_est(),
		Ldq(_Ldq), R(_R) {}

	void setup() {}
	bool update();
	float getMag() const {return emf_mag;}
	void getEMF(vect_dq *_emf) const {*_emf = emf;}

};

#endif /* INC_EMFPOSITIONSENSOR_H_ */
