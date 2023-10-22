/*
 * EMFPositionSensor.cpp
 *
 *  Created on: Sep 7, 2022
 *      Author: ayeiser
 */

#include <EMFPositionSensor.h>

bool EMFPositionSensor::update() {
	vect_uvw current = cur.getCurrents();
	vect_uvw pwm_uvw = pwm.getVuvw();

//	vect_uvw slope = cur.getCurrentSlopes();

	angle_est.setAngle_int(elec_angle);
	vect_dq current_dq = angle_est.uvw_to_dq(current);
	vect_dq pwm_dq = angle_est.uvw_to_dq(pwm_uvw);
	vect_dq emf_inst;

//	vect_dq slope_dq = angle_est.uvw_to_dq(slope);

//	emf.d = -(slope_dq.d * Ldq.d + current_dq.d * R);
//	emf.q = -(slope_dq.q * Ldq.q + current_dq.q * R);
	emf_inst.d = pwm_dq.d - current_dq.d * R + elec_vel*Ldq.q*current_dq.q;
	emf_inst.q = pwm_dq.q - current_dq.q * R + elec_vel*Ldq.d*current_dq.d;

	const float upper_thresh = 1.0f;
	const float lower_thresh = 0.5f;
	const float alpha = MIN(0.1f, 1000.0f * dt); // 1 ms convergence time
	const float beta = MIN(0.02f, 200.0f * dt);   // 5 ms convergence time


	emf.d = INTERP(0.0f, emf_inst.d, alpha);
	if (reverse) {
		emf.q = INTERP(-emf_mag, emf_inst.q, alpha);
	} else {
		emf.q = INTERP(emf_mag, emf_inst.q, alpha);
	}

	uint32_t angle_err;
	const uint32_t PI_OVER_2 = 0x40000000;

//	cordic_modphase(emf_inst.d, emf_inst.q, &emf_inst_mag, &angle_err);


	cordic_modphase(emf.d, emf.q, &emf_mag, &angle_err);

	if (reverse) {
		angle_err += PI_OVER_2;
	} else {
		angle_err -= PI_OVER_2;
	}

	elec_angle = calcPhaseAdjAngle(1.0f); // increment by 1 time step
	elec_angle += angle_err;
	elec_vel += beta * calcVelocity(angle_err);

	reverse = (elec_vel < 0.0f);


	if (emf_mag > upper_thresh) {
		tracking = true;
	}
	if (emf_mag < lower_thresh) {
		tracking = false;
		elec_vel = 0;
		elec_angle = 0;
	}

	mech_vel = elec_vel / pole_count;
	return tracking;
}
