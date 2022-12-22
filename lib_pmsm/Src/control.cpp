/*
 * control.cpp
 *
 *  Created on: Aug 15, 2022
 *      Author: ayeiser
 */

#include <control.h>

float PIController::update(float err) {
	output += KP * ( (err - last_err) + err*dt*KI);
	last_err = err;
	output = CLIP(output, -limit, limit);
	return output;
}

float PIDController::update(float err) {
	float diff = err - last_err;
	output += KP * ( diff + err*dt*KI + (diff - last_diff)*KD/dt);
	last_err = err;
	last_diff = diff;
	output = CLIP(output, -limit, limit);
	return output;
}

vect_dq CurrentDQController::update(vect_dq &target, vect_dq &current, float omega) {
	vphase.d = idc.update(target.d - current.d);
	vphase.q = iqc.update(target.q - current.q);

	vphase.d -= 0.5f * omega * Ldq.q * (current.q - ilast.q);
	vphase.q += 0.5f * omega * Ldq.d * (current.d - ilast.d);

	ilast.d = current.d;
	ilast.q = current.q;

	vphase_mag = cordic_modulus(vphase.d, vphase.q);
	overmod = (vphase_mag > limit);
	if (overmod) {
		vphase.d *= (limit / vphase_mag);
		vphase.q *= (limit / vphase_mag);
	}

	idc.setOutput(vphase.d);
	iqc.setOutput(vphase.q);

	return vphase;
};

void GenericController::update(vect_dq &target) {
	float omega = pos.getElecVelocity();
	i_phase.setAngle_int(pos.calcPhaseAdjAngle(delay_corr));

	vect_uvw iuvw = current.getCurrents();
	idq = i_phase.uvw_to_dq(iuvw);

	if (!pos.isTracking()) {
		target.d = 0.0f;
		target.q = 0.0f;
		omega = 0.0f;
	}

	idqc.setLimit(current.getVBUS() * 0.7f); // lol overmod
	vdq = idqc.update(target, idq, omega);

	pwm.setTargets(vdq);
	pwm.update(pos.calcPhaseAdjAngle(0.75f + delay_corr), current.getVBUS());
}
