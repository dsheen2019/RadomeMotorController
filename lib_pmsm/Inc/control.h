/*
 * control.h
 *
 *  Created on: Aug 15, 2022
 *      Author: ayeiser
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#include "main.h"
#include "angle.h"

#include "PositionSensor.h"
#include "CurrentSensor.h"
#include "pwm.h"

/*
 * Implements a discrete PI controller y = KP * (1 + KI/s)) * x
 * Canonical form: y[n+1] = KP*(x[n] + KI*dt*XX[n]), XX[n+1] = XX[n] + x[n]
 * Our form: y[n+1] = y[n] + KP*((x[n] - x[n-1]) + KI*dt*x[n])
 * This form has the advantage of having no integrator to saturate.
 */
class PIController {
protected:
	float KP;
	float KI;
	float limit;
	float dt;
	float output;
	float last_err;
public:
	PIController(float _dt, float _limit, float _KP = 1.0f, float _KI = 0.0f) :
		KP(_KP), KI(_KI), limit(_limit), dt(_dt), output(0.0f), last_err(0.0f) {}

	float update(float state, float setpoint) {return update(setpoint - state);}
	float update(float err);

	void setLimit(float _limit) {limit = _limit;}
	void setGains(float _KP, float _KI) {KP = _KP; KI = _KI;}
	void setDT(float _dt) {dt = _dt;}
	void reset() {output = 0.0f; last_err = 0.0f;}
	void setOutput(float _out) {output = _out;}
	float getOutput() {return output;}
};

/*
 * Same as the PI controller, but
 * y[n+1] = y[n] + KD * ((x[n] - x[n-1]) + KI*dt*x[n] + KD/dt*(x[n] - 2*x[n-1] + x[n-2]))
 */
class PIDController : public PIController {
protected:
	float KD;
	float last_diff;
public:
	PIDController(float _dt, float _limit, float _KP = 1.0f, float _KI = 0.0f, float _KD = 0.0f) :
		PIController(_dt, _limit, _KP, _KI), KD(_KD), last_diff(0.0f) {}

	float update(float state, float setpoint) {return update(setpoint - state);}
	float update(float err);
	void setGains(float _KP, float _KI, float _KD) {
		KP = _KP; KI = _KI; KD = _KD;
	}
	void reset() {output = 0.0f; last_err = 0.0f; last_diff = 0.0f;}
};

class CurrentDQController {
private:
	const float scale;
	float dt;
	float limit;
	vect_dq Ldq;
	PIController idc;
	PIController iqc;
	vect_dq vphase;
	vect_dq ilast;
	bool overmod;
	float vphase_mag;
public:
	CurrentDQController(float _dt, float _limit, vect_dq _Ldq, float _scale = 0.3f) :
		scale(_scale), dt(_dt), limit(_limit), Ldq(_Ldq),
		idc(_dt, 2.0f * _limit, Ldq.d * scale / dt, scale * 0.5f / dt),
		iqc(_dt, 2.0f * _limit, Ldq.q * scale / dt, scale * 0.5f / dt),
		vphase{0.0f, 0.0f}, ilast{0.0f, 0.0f}, overmod(false), vphase_mag(0.0f) {}

	vect_dq update(vect_dq&, vect_dq&, float);
	void reset() {
		vphase.d = vphase.q = 0.0f;
		ilast.d = ilast.q = 0.0f;
		idc.reset();
		iqc.reset();
	}
	void setLimit(float _limit) {
		limit = _limit;
		idc.setLimit(2.0f*_limit);
		iqc.setLimit(2.0f*_limit);
	}
	void setDT(float _dt) {
		dt = _dt;
		idc.setDT(_dt);
		iqc.setDT(_dt);
	}
	bool isOvermod() {return overmod;}
	vect_dq getOutput() {return vphase;}
	float getMag() {return vphase_mag;}
};

class GenericController {
protected:
	PositionSensor &pos;
	CurrentSensor &current;
	PWMGenerator &pwm;
	CurrentDQController &idqc;
	TrigAngle i_phase;
	TrigAngle v_phase;
	vect_dq idq, vdq;
	float dt;
	float delay_corr;
public:
	GenericController(float _dt,
			PositionSensor &_pos,
			CurrentSensor &_current,
			PWMGenerator &_pwm,
			CurrentDQController &_idqc,
			float _delay = 0.0f) :
				pos(_pos), current(_current), pwm(_pwm), idqc(_idqc),
				i_phase(), v_phase(), dt(_dt), delay_corr(_delay) {};
	void update(vect_dq &target);
	void setDT(float _dt) {
		dt = _dt;
		pos.setDT(dt);
		idqc.setDT(dt);
	}
	vect_dq getVdq() {return vdq;}
	vect_dq getIdq() {return idq;}
};

class LowPassFilter {
protected:
	float dt;
	float w;
	float var;
public:
	LowPassFilter(float _dt, float _w) : dt(_dt), w(_w), var(0.0f){}
	void setFreq(float _w) {w = _w;}
	float update(float _var) {
		float frac = w * dt;
		var = var*(1.0f - frac) + frac*_var;
		return var;
	}
	float getValue() {return var;}
};

#endif /* INC_CONTROL_H_ */
