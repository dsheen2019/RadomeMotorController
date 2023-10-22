/*
 * retarget.c
 *
 *  Created on: Jan 27, 2023
 *      Author: ayeiser
 */


#include <retarget.h>

UART_HandleTypeDef* gHuart;
uint8_t uart_rx_buf[UART_RX_BUF_SIZE];
int uart_rx_ptr = 0;
void RetargetInit(UART_HandleTypeDef* huart) {
	gHuart = huart;
	HAL_UART_Receive_DMA(gHuart, uart_rx_buf, UART_RX_BUF_SIZE);
	setvbuf(stdout, NULL, _IONBF, 0);
	setvbuf(stdin, NULL, _IONBF, 0);
}

int rxDataReady() {
	return (2*UART_RX_BUF_SIZE - (gHuart->hdmarx->Instance->CNDTR + uart_rx_ptr)) % UART_RX_BUF_SIZE;
}

PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(gHuart, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}

GETCHAR_PROTOTYPE
{
  uint8_t ch = 0;

  /* Clear the Overrun flag just before receiving the first character */
  __HAL_UART_CLEAR_OREFLAG(gHuart);

  /* Wait for reception of a character on the USART RX line and echo this
   * character on console */
//  HAL_UART_Receive(gHuart, (uint8_t *)&ch, 1, HAL_MAX_DELAY);

  while (rxDataReady() == 0);
  ch = uart_rx_buf[uart_rx_ptr];
  uart_rx_ptr++;
  uart_rx_ptr %= UART_RX_BUF_SIZE;
//  HAL_UART_Transmit(gHuart, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
