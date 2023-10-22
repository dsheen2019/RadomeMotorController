/*
 * CANHandler.cpp
 *
 *  Created on: Feb 9, 2023
 *      Author: ayeiser
 */

#include <CANHandler.h>
#include <stdio.h>

void CANHandler::start() {
	if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
		Error_Handler();
	}

	TxHeader.Identifier = 0x69;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;
}

bool CANHandler::setupTxPacket(uint32_t id, uint32_t len) {
	TxHeader.Identifier = id;
	if (len <= 8) {
		TxHeader.DataLength = FDCAN_DLC_BYTES_1*len;
		return true;
	} else {
		return false;
	}
}

HAL_StatusTypeDef CANHandler::pushTx() {
	return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, TxData);
}

HAL_StatusTypeDef CANHandler::pushTx(uint32_t id, uint32_t len) {
	if (!setupTxPacket(id, len)) {
		return HAL_ERROR;
	}
	return pushTx();
}

uint32_t CANHandler::getRxFifoFillLevel(uint32_t fifo) {
	return HAL_FDCAN_GetRxFifoFillLevel(hfdcan, fifo);
}

HAL_StatusTypeDef CANHandler::popRx(uint32_t fifo) {
	return HAL_FDCAN_GetRxMessage(hfdcan, fifo, &RxHeader, RxData);
}

void CANHandler::printRxCAN(void) {
	uint32_t len = RxHeader.DataLength / FDCAN_DLC_BYTES_1;
	uint32_t addr = RxHeader.Identifier;

	if (len > 0 && len <= 8) {
		printf("Rx CAN id: %08x data:", addr);
		for (int i = 0; i < len; i++) {
			printf(" %02x", RxData[i]);
		}
		printf("\n");
	}
}

void CANHandler::processRxCAN(motor_command_t* _cmd, motor_reply_t* _reply) {
	uint32_t len = RxHeader.DataLength / FDCAN_DLC_BYTES_1;
	uint32_t addr = RxHeader.Identifier;

	if (addr == ID) {
		if (len == 8) {
			_cmd->position = *(uint32_t*) (RxData);
			_cmd->velocity = *(float*) (RxData + 4);
			_cmd->last_tick = HAL_GetTick();

//			printf("pos=0x%04x, vel=% 6.2f\n", _cmd->position >> 16, _cmd->velocity);

			setupTxPacket(0x100 + ID, 8);
			*(uint32_t*) (TxData) = _reply->position;
			*(float*) (TxData + 4) = _reply->velocity;

			pushTx();
		}
	}
}



