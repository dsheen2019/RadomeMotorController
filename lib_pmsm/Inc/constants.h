/*
 * constants.h
 *
 *  Created on: Aug 14, 2022
 *      Author: ayeiser
 */

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#define PI 3.141592653589793f
#define TWO_PI (2.0f * PI)
#define SQRT_3_4 0.8660254037844386f
#define TWO_THIRDS 0.6666666666666666f
#define SQRT_1_3 0.577350269189626f

#define MAX(x, y) (((x)>(y))?(x):(y))
#define MIN(x, y) (((x)<(y))?(x):(y))
#define MAX3(x, y, z) (MAX(MAX((x), (y)), (z)))
#define MIN3(x, y, z) (MIN(MIN((x), (y)), (z)))
#define CLIP(x, l, u) (MAX(MIN((x), (u)), (l)))

#define INTERP(x1, x2, coeff) ((x1)*(1.0f - (coeff)) + (x2)*(coeff))



#endif /* CONSTANTS_H_ */
