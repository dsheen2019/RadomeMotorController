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
#include <cmath>

#include "PositionSensor.h"
#include "CurrentSensor.h"
#include "pwm.h"
#include "stdio.h"
#include "ZettlexIncoder.h"

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

	virtual float update(float state, float setpoint) {return update(setpoint - state);}
	virtual float update(float err);

	void setLimit(float _limit) {limit = _limit;}
	void setGains(float _KP, float _KI) {KP = _KP; KI = _KI;}
	void setDT(float _dt) {dt = _dt;}
	void reset() {output = 0.0f; last_err = 0.0f;}
	void setOutput(float _out) {output = _out;}
	float getOutput() {return output;}
};

class PIControllerSlewLimited : public PIController {
protected:
	float slew_limit;
public:
	PIControllerSlewLimited(float _dt, float _limit, float _KP = 1.0f, float _KI = 0.0f) :
		PIController(_dt, _limit, _KP, _KI), slew_limit(0.05f) {}

	void setSlewLimit(float _lim) {slew_limit = _lim;}

	virtual float update(float err) {
		float old_out = output;
		PIController::update(err);
		output = old_out + fminf(fmaxf(output - old_out, -slew_limit*dt), slew_limit*dt);
		return output;
	}
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
		idc(_dt, 2.0f * _limit, Ldq.d * scale / dt, scale * 0.25f / dt),
		iqc(_dt, 2.0f * _limit, Ldq.q * scale / dt, scale * 0.25f / dt),
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

class ParentController {
protected:
	static CurrentSensor &current;
	static PWMGenerator &pwm;
	static PositionSensor &pos;
	static float dt;

	static uint32_t angle;
	static float omega;

	static vect_uvw iuvw;
	static vect_dq idq;
	static vect_dq vdq;
	static float vbus;

	static bool enable;
	static bool shutdown;

	const static float undervoltage_threshold;
	const static float overvoltage_threshold;  // TODO not used
	const static float enable_voltage_threshold;

	const static float overcurrent_threshold;

public:
	ParentController() {}
	virtual void print() {

		float mech_vel = pos.getMechVelocity() * 60.0f / (2.0f * PI);

		printf("Id =% 6.2fA Iq =% 6.2fA     Vd =% 6.2fV Vq = % 6.2fV   pos = 0x%08x   Vel = % 6.0fRPM   Vbus = % 6.2fV\n",
						idq.d, idq.q, vdq.d, vdq.q, pos.getPhaseCorrMechAngle(), mech_vel, vbus);

//		printf("% 7.3f % 7.3f\n", idq.d, idq.q);
	}

	virtual void exec() {
		print();
		HAL_Delay(50);
	}

	static uint32_t getAngle() {return angle;}

	static vect_dq getCurrents() { return idq; }
	static float getVbus() { return vbus; }

	virtual void update_downcount() {}
	virtual void update_upcount() {}


	void update(TIM_HandleTypeDef* htim) {
		angle += int32_t(lroundf(0.5f * omega * dt * ANGLE_CONV));
		if (htim->Instance->CR1 & TIM_CR1_DIR) {
			ADCReader::clearReadyFlag();
			pos.update();
			update_downcount();
		} else {
			current.update(htim, 1700);
			iuvw = current.getCurrents();
			vbus = current.getVBUS();
			TrigAngle current_angle = angle;
			idq = current_angle.uvw_to_dq(iuvw);

			if (idq.d * idq.d + idq.q * idq.q > overcurrent_threshold*overcurrent_threshold) {
				shutdown = true;
			}

			update_upcount();

			vbus = fmaxf(undervoltage_threshold, vbus);

			if (vbus < undervoltage_threshold) {
				enable = false;
			}
			if (vbus > enable_voltage_threshold) {
				enable = true;
			}

			if (!enable || shutdown) {
				pwm.stopSwitching();
			} else {
				pwm.startSwitching();
			}

			pwm.setTargets(vdq);
		}
		uint32_t v_angle = angle + int32_t(lroundf(0.75f * dt * omega * ANGLE_CONV));
		pwm.setAngle(v_angle);
		pwm.update(vbus);
	}
};

class DummyController : public ParentController {
public:
	DummyController() : ParentController() {}
	void update_downcount() {
		omega = 2.0f * PI * 7.0f * 0.1f;
	}

	void update_upcount() {
		vdq.d = 1.5f;
		vdq.q = 1.5f;
	}
};

class TorqueController : public ParentController {
protected:
	CurrentDQController &idqc;
	float torque;
public:
	TorqueController(CurrentDQController& _idqc) : ParentController(), idqc(_idqc), torque(0.0f) {}

	void setTorque(float t) { torque = t; }

	virtual void update_downcount() {
		if (pos.isTracking()) {
			angle = pos.getPhaseCorrElecAngle(); // TODO this is for back emf sensor, add phase correction
			omega = pos.getElecVelocity();
		} else {
			omega = 0.0f;
			angle = 0.0f;
		}
	}

	virtual void update_upcount() {
		idqc.setLimit(current.getVBUS() * 0.7f); // lol overmod
		vect_dq idq_target;
		if (pos.isTracking()) {
			idq_target.d = 0.0f;
			idq_target.q = torque;
		} else {
			idq_target.d = idq_target.q = 0.0f;
		}
		vdq = idqc.update(idq_target, idq, 0.0f);
	}
};

class PDController : public TorqueController {
protected:
	const float current_limit_max;
	float current_limit, kp, kd;
	uint32_t pos_target;
	float vel_target;
public:
	PDController(CurrentDQController& _idqc, float _current_limit) :
		TorqueController(_idqc),
		current_limit_max(_current_limit),
		current_limit(0.0f),
		kp(0.0f), kd(0.0f),
		pos_target(0), vel_target(0.0f) {}

	void setGains(float _kp, float _kd, float _lim) {
		kp = _kp; kd = _kd; current_limit = fminf(fmaxf(_lim, 0.0f), current_limit_max);
	}

	void setTargets(uint32_t pos, float vel) {
		pos_target = pos;
		vel_target = vel;
	}

	virtual void update_upcount() {
		float pos_error = float(int32_t(pos_target - pos.getPhaseCorrMechAngle())) / ANGLE_CONV;
		float vel_error = vel_target - pos.getMechVelocity();
		torque = fminf(fmaxf(kp * pos_error + kd * vel_error, -current_limit), current_limit);
		TorqueController::update_upcount();
	}
};

class PDVelocityController : public PDController {
protected:
	float velocity_lag_hpf;
	float velocity_slew_limit = MAX_MOTOR_SLEW; // rad/s^2, motor-side
	float vel_target_slew_limited = 0.0f;
public:
	PDVelocityController(CurrentDQController& _idqc, float _current_limit) :
		PDController(_idqc, _current_limit), velocity_lag_hpf(1.0f) {}

	void set_hpf(float _hpf) { velocity_lag_hpf = _hpf; }

	void setVelTarget(float vel) {
		vel_target = vel;
	}

	virtual void update_upcount() {
		vel_target_slew_limited += fminf(fmaxf(vel_target - vel_target_slew_limited, -velocity_slew_limit * dt), velocity_slew_limit * dt);
		float pos_error = float(int32_t(pos_target - pos.getPhaseCorrMechAngle())) / ANGLE_CONV;
		float pos_delta = (1.0f - dt * velocity_lag_hpf) * dt * vel_target_slew_limited - dt * velocity_lag_hpf * pos_error;
		pos_target += int32_t(lroundf(pos_delta * ANGLE_CONV));
		float vel_error = vel_target_slew_limited - pos.getMechVelocity();
		torque = fminf(fmaxf(kp * pos_error + kd * vel_error, -current_limit), current_limit);
		TorqueController::update_upcount();
	}
};

class IncoderPositionController : public PDVelocityController {
protected:
	ZettlexIncoder& zett;
	uint32_t zett_pos_target = 0;
	float zett_vel_target = 0;
	float pos_kvel = 0;
public:
	IncoderPositionController(CurrentDQController& _idqc, ZettlexIncoder& _inc, float _current_limit) :
		PDVelocityController(_idqc, _current_limit), zett(_inc) {}

	void setPositionGains(float _kp) { pos_kvel = _kp; }

	void setTrajectory(uint32_t _pos, float _vel) { zett_pos_target = _pos; zett_vel_target = _vel; }

	virtual void update_upcount() {
		float zett_pos_err = float(int32_t(zett_pos_target - zett.getAngle())) / ANGLE_CONV;
		zett_pos_target += int32_t(lroundf(zett_vel_target * dt * ANGLE_CONV));

		float vel_err_command = zett_pos_err * pos_kvel;
		float vel_err_max = 0.7f * sqrtf(2.0f * fabsf(zett_pos_err) * MAX_MOTOR_SLEW / GEAR_REDUCTION) + 0.02f;


		vel_err_command = fminf(fmaxf(vel_err_command, -vel_err_max), vel_err_max);

#ifdef AZIMUTH
		vel_target = (zett_vel_target + vel_err_command) * GEAR_REDUCTION * VEL_FUDGE_FACTOR;
#endif

#ifdef ELEVATION
		float zett_upper_space = fmaxf(float(int32_t(EL_UPPER_HARDSTOP - zett.getAngle())) / ANGLE_CONV, 0.0f);
		float zett_lower_space = fmaxf(float(int32_t(zett.getAngle() - EL_LOWER_HARDSTOP)) / ANGLE_CONV, 0.0f);

		float vel_tot_max = 0.7f * sqrtf(2.0f * zett_upper_space * MAX_MOTOR_SLEW / GEAR_REDUCTION);
		float vel_tot_min = -0.7f * sqrtf(2.0f * zett_lower_space * MAX_MOTOR_SLEW / GEAR_REDUCTION);

		vel_target = fminf(fmaxf((zett_vel_target + vel_err_command), vel_tot_min), vel_tot_max) * GEAR_REDUCTION * VEL_FUDGE_FACTOR;
//		vel_target = (zett_vel_target + vel_err_command) * GEAR_REDUCTION * VEL_FUDGE_FACTOR;

#endif

		vel_target_slew_limited += fminf(fmaxf(vel_target - vel_target_slew_limited, -velocity_slew_limit * dt), velocity_slew_limit * dt);
		float pos_error = float(int32_t(pos_target - pos.getPhaseCorrMechAngle())) / ANGLE_CONV;
		float pos_delta = (1.0f - dt * velocity_lag_hpf) * dt * vel_target_slew_limited - dt * velocity_lag_hpf * pos_error;
		pos_target += int32_t(lroundf(pos_delta * ANGLE_CONV));
		float vel_error = vel_target_slew_limited - pos.getMechVelocity();
		torque = fminf(fmaxf(kp * pos_error + kd * vel_error, -current_limit), current_limit);
		TorqueController::update_upcount();
	}
};

class MotorParamFinder : public ParentController {
protected:
	float max_current;

	vect_dq accum;
	int count;

	int state;

public:
	MotorParamFinder(float _max_current) :
		max_current(_max_current),
		accum{0.0f, 0.0f}, count(0), state(-1)
	{}

	void exec() {
		printf("Entering motor parameter search mode!\n");
		state = 0;
		omega = 0.0f;
		vdq.d = vdq.q = 0.0f;

		accum.d = accum.q = 0.0f;

		while (state == 0) {
			print();
			HAL_Delay(50);
		}

		if (state != 1) {
			omega = 0.0f;
			state = -1;
			printf("Something failed...\n");
			return;
		}

		const int num_incrs = 5;
		float omegas[num_incrs] = {0.0f, 1e3f, 2e3f, 4e3f, 8e3f };

		for (int i = 0; i < num_incrs; i++) {
			omega = omegas[i];

			accum.d = accum.q = 0.0f;

			HAL_Delay(100);

			count = 4096;
			while (count > 0) {
				print();
				HAL_Delay(50);
			}

			accum.d /= 4096.0f;
			accum.q /= 4096.0f;

			vect_dq Z;
			float mag = accum.d*accum.d + accum.q*accum.q;

			Z.d = accum.d * vdq.d / mag;
			Z.q = -accum.q * vdq.d / mag;

			if (omega > 1.0f) {
				Z.q /= omega;
			}

			printf("w = %f rad/s Id = %fA Iq = %fA Vd = %fV Vq = %fV, R = %f ohm, L = %f uH\n",
					omega, accum.d, accum.q, vdq.d, vdq.q, Z.d, Z.q * 1e6f);
		}

		omega = 0.0f;
		state = -1;

		return;
	}

	void update_downcount() {
	}

	void update_upcount() {
		switch (state) {
		case 0:
			vdq.d += 10.0f*dt;

			if (fabsf(idq.d) > max_current) {
				state = 1;
			}

			if (fabsf(vdq.d) > current.getVBUS() * 0.25f) {
				state = -1;
			}
			break;
		case 1:
			if (count > 0) {
				accum.d += idq.d;
				accum.q += idq.q;
				count--;
			}
			break;
		default:
		{
			vdq.d = vdq.q = 0.0f;
		}
			break;
		}
	}
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
