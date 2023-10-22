/*
 * retarget.h
 *
 *  Created on: Jan 27, 2023
 *      Author: ayeiser
 */

#ifndef INC_RETARGET_H_
#define INC_RETARGET_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "main.h"

#define UART_RX_BUF_SIZE 256


void RetargetInit(UART_HandleTypeDef* huart);
int rxDataReady();

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

#ifdef __GNUC__
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif

#ifdef __cplusplus
}
#endif

#endif /* INC_RETARGET_H_ */
