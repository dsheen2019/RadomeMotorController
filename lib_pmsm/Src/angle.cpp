/*
 * angle.cpp
 *
 *  Created on: Aug 20, 2022
 *      Author: ayeiser
 */

#include <angle.h>


TrigAngle::~TrigAngle() {
	// TODO Auto-generated destructor stub
}

void TrigAngle::setAngle_int(uint32_t _angle) {
	angle = _angle;
	cordic_sincos(_angle, &cos, &sin);
}

vect_dq TrigAngle::uvw_to_dq(vect_uvw &uvw) {
	float a = TWO_THIRDS*(uvw.u - 0.5f * (uvw.v + uvw.w));
	float b = TWO_THIRDS*(SQRT_3_4 * (uvw.v - uvw.w));
	vect_dq dq;
	dq.d =  cos * a + sin * b;
	dq.q = -sin * a + cos * b;
	return dq;
}

vect_uvw TrigAngle::dq_to_uvw(vect_dq &dq) {
	float a = cos * dq.d - sin * dq.q;
	float b = sin * dq.d + cos * dq.q;

	vect_uvw uvw;
	uvw.u = a;
	a *= -0.5f;
	b *= SQRT_3_4;

	uvw.v = a + b; // -0.5 * a + 0.866 * b
	uvw.w = a - b; // -0.5 * a - 0.866 * b
	return uvw;
}
