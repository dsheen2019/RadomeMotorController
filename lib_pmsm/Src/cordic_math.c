/*
 * cordic_math.c
 *
 *  Created on: Aug 28, 2022
 *      Author: ayeiser
 */

#include "cordic_math.h"

static inline void cordic_sincos_int(uint32_t angle, int32_t* cos, int32_t* sin) {
	/*
	 * Sine and cosine are actually implemented as functions with 2 arguments: modulus and phase
	 * so the return is modulus * sin(theta), modulus * cos(theta)
	 *
	 * Even when sine and cosine are set to 1 arg functions, they still multiply by the second argument
	 * This is due to a "feature" in the CORDIC implementation:
	 * If you set NARGS to zero, and are using 32 bit precision, the second argument
	 * just stays there, so you "don't have to write it again", even if CSR is changed
	 *
	 * So when you use the modulo operator,
	 * the second argument stays put unless you explicitly reset it
	 */

	WRITE_REG(CORDIC->CSR, CORDIC_FUNCTION_COSINE
			| CORDIC_PRECISION_6CYCLES
			| CORDIC_CSR_NARGS		// 2 args :(
			| CORDIC_CSR_NRES);		// 2 results
	WRITE_REG(CORDIC->WDATA, angle);
	WRITE_REG(CORDIC->WDATA, 0x7fffffff); // pain and suffering
	*cos = READ_REG(CORDIC->RDATA);
	*sin = READ_REG(CORDIC->RDATA);
}

void cordic_sincos(uint32_t angle, float* sin, float* cos) {
	int32_t sin_int, cos_int;
	cordic_sincos_int(angle, &sin_int, &cos_int);
	*sin = 0x1p-31f * (float) sin_int;
	*cos = 0x1p-31f * (float) cos_int;
}

static inline int32_t cordic_modulus_int(int32_t x, int32_t y) {
	WRITE_REG(CORDIC->CSR, CORDIC_FUNCTION_MODULUS
			| CORDIC_PRECISION_6CYCLES
			| CORDIC_CSR_NARGS);	// 2 args (1 result)

	WRITE_REG(CORDIC->WDATA, x);
	WRITE_REG(CORDIC->WDATA, y);
	return READ_REG(CORDIC->RDATA);
}

float cordic_modulus(float x, float y) {
	const float max_mag = 1.5f*(MAX(x, -x)+MAX(y, -y));
	const float scale_factor = max_mag * 0x1p-31f;
	int32_t x_int = (int32_t) (x / scale_factor);
	int32_t y_int = (int32_t) (y / scale_factor);
	int32_t mag_int = cordic_modulus_int(x_int, y_int);
	return scale_factor * ((float) mag_int);
}

static inline void cordic_modphase_int(int32_t x, int32_t y, int32_t* mag, uint32_t* phase) {
	WRITE_REG(CORDIC->CSR, CORDIC_FUNCTION_MODULUS
			| CORDIC_PRECISION_6CYCLES
			| CORDIC_CSR_NARGS		// 2 args
			| CORDIC_CSR_NRES);		// 2 results

	WRITE_REG(CORDIC->WDATA, x);
	WRITE_REG(CORDIC->WDATA, y);
	*mag = READ_REG(CORDIC->RDATA);
	*phase = READ_REG(CORDIC->RDATA);
}

void cordic_modphase(float x, float y, float* mag, uint32_t* angle) {
	int32_t mag_int;
	uint32_t phase_int;

	const float max_mag = 1.5f*(MAX(x, -x) + MAX(y, -y));
	const float scale_factor = max_mag * 0x1p-31f;
	int32_t x_int = (int32_t) (x / scale_factor);
	int32_t y_int = (int32_t) (y / scale_factor);
	cordic_modphase_int(x_int, y_int, &mag_int, &phase_int);
	*mag = scale_factor * ((float) mag_int);
	*angle = phase_int;
}

static inline uint32_t cordic_sqrt_int(uint32_t x) {
	WRITE_REG(CORDIC->CSR, CORDIC_FUNCTION_SQUAREROOT
			| CORDIC_PRECISION_3CYCLES);	// 1 arg, 1 result
	WRITE_REG(CORDIC->WDATA, x);
	return READ_REG(CORDIC->RDATA);
}

float cordic_sqrt(float x, float max_sqrt_x) {
	uint32_t x_int = (uint32_t) (x / (0x1p-31f * max_sqrt_x * max_sqrt_x));
	uint32_t sqrt_x_int = cordic_sqrt_int(x_int);
	return 0x1p-31f * max_sqrt_x * (float) (sqrt_x_int);
}


