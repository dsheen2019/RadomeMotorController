/*
 * CANHandler.h
 *
 *  Created on: Feb 9, 2023
 *      Author: ayeiser
 */

#ifndef INC_CANHANDLER_H_
#define INC_CANHANDLER_H_

#include "main.h"
#include "structs.h"

class CANHandler {
protected:
	FDCAN_HandleTypeDef *hfdcan;

	FDCAN_TxHeaderTypeDef TxHeader;
	uint8_t TxData[8];

	FDCAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[8];
public:
	CANHandler(FDCAN_HandleTypeDef *_hfdcan):
		hfdcan(_hfdcan) {};
	void start();

	bool setupTxPacket(uint32_t id, uint32_t len);
	HAL_StatusTypeDef pushTx();
	HAL_StatusTypeDef pushTx(uint32_t id, uint32_t len);
	uint32_t getRxFifoFillLevel(uint32_t fifo = FDCAN_RX_FIFO0);
	HAL_StatusTypeDef popRx(uint32_t fifo = FDCAN_RX_FIFO0);

	void printRxCAN(void);

	void processRxCAN(motor_command_t*, motor_reply_t*);
	void createReplyPacket(motor_reply_t*);
};

#endif /* INC_CANHANDLER_H_ */
