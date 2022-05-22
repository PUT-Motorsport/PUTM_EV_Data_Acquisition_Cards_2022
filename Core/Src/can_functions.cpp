/*
 * can_functions.cpp
 *
 *  Created on: 22 maj 2022
 *      Author: molso
 */

#include "can_functions.hpp"

void init_can_filter(CAN_FilterTypeDef sFilterConfig) {
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}
}

void init_can_config(void) {

	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_CAN_ActivateNotification(&hcan1,
			CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK) {
		Error_Handler();
	}
}

