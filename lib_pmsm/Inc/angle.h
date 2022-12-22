/*
 * angle.h
 *
 *  Created on: Aug 20, 2022
 *      Author: ayeiser
 */

#ifndef INC_ANGLE_H_
#define INC_ANGLE_H_

#include "main.h"
#include "cordic_math.h"

#define ANGLE_CONV (0x1p31f / PI)

struct vect_dq {
	float d, q;
};

struct vect_uvw {
	float u, v, w;
};

class TrigAngle {
private:
	uint32_t angle;
	float sin, cos;
public:
	TrigAngle() : angle(0), sin(0.0f), cos(1.0f) {};
	TrigAngle(uint32_t _angle) {
		setAngle_int(_angle);
	}
	virtual ~TrigAngle();

	inline void setAngle_float(float _angle) {setAngle_int(int32_t(_angle*ANGLE_CONV));};
	void setAngle_int(uint32_t);

	float getAngle_float() {return angle/ANGLE_CONV;}
	uint32_t getAngle_int() {return angle;}

	vect_dq getSinCos() { vect_dq sc = {cos, sin}; return sc; }

	vect_dq uvw_to_dq(vect_uvw&);
	vect_uvw dq_to_uvw(vect_dq&);
};

#endif /* INC_ANGLE_H_ */
