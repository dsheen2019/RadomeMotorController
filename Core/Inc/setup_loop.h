/*
 * control_loop.h
 *
 *  Created on: Aug 14, 2022
 *      Author: ayeiser
 */

#ifndef SETUP_LOOP_H_
#define SETUP_LOOP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void setup();
void loop();
void motor_control_interrupt();

#ifdef __cplusplus
}
#endif

#endif /* SETUP_LOOP_H_ */
