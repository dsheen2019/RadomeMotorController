/*
 * PositionSensor.h
 *
 *  Created on: Aug 14, 2022
 *      Author: ayeiser
 */

#ifndef POSITIONSENSOR_H_
#define POSITIONSENSOR_H_


#include "stm32g4xx_hal.h"
#include "angle.h"

class PositionSensor {
protected:
	float dt;					// seconds
	uint32_t elec_angle;		// 1.31q from -pi to pi
	const uint32_t pole_count;	// Number of poles per mechanical revolution
	const uint32_t elec_offset;	// offset between encoder and field
	uint32_t mech_angle;		// fixed point, depends on mech_angle_scale
	const uint32_t mech_scale;	// mech_angle_scale . (32 - mech_angle_scale) q format
	float elec_vel;				// radians/second
	float mech_vel;				// radians/second
	const float delay_corr;			// steps
	bool tracking;
public:
	PositionSensor(float _dt, uint32_t _cnt, uint32_t _off = 0u, float _delay_corr=0.0f, uint32_t _scale = 1u) :
		dt(_dt), pole_count(_cnt), elec_offset(_off), mech_scale(_scale), delay_corr(_delay_corr) {};
	virtual void setup() = 0;
	virtual bool update() = 0;
	uint32_t getElecAngle() const {return elec_angle;}
	uint32_t getMechAngle() const {return mech_angle;}
	float getElecVelocity() const {return elec_vel;}
	float getMechVelocity() const {return mech_vel;}
	uint32_t calcAngleIncrement(float steps = 1.0f) const {
		return int32_t(steps * dt * elec_vel * ANGLE_CONV);
	}
	uint32_t calcPhaseAdjAngle(float steps = 1.0f) const {
		return elec_angle + calcAngleIncrement(steps);
	}
	uint32_t getPhaseCorrElecAngle() const {
		return elec_angle + int32_t(delay_corr * dt * elec_vel * ANGLE_CONV);
	}
	uint32_t getPhaseCorrMechAngle() const {
		return mech_angle + int32_t(delay_corr * dt * mech_vel * ANGLE_CONV);
	}
	float calcVelocity(int32_t step) const {
		return float(step) / (ANGLE_CONV * dt);
	}
	void setDT(float _dt) {
		dt = _dt;
	}
	bool isTracking() const { return tracking;}
	virtual float getMag() const { return 0.0f;}

};

class TestPositionSensor : public PositionSensor {
public:
	TestPositionSensor(float _dt, uint32_t _cnt, uint32_t _off = 0u) : PositionSensor(_dt, _cnt, _off) {};
	void setup() {};
	bool update() {return false;};
	void setElecAngle(uint32_t _angle) { elec_angle = _angle; };
	void setElecVelocity(float _vel) { elec_vel = _vel; };
	void advanceAngle(float steps = 1.0f) { elec_angle += calcAngleIncrement(steps); };
};

#endif /* POSITIONSENSOR_H_ */
