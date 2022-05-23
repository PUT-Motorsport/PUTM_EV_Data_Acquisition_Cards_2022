/*
 * can_functions.cpp
 *
 *  Created on: 22 maj 2022
 *      Author: molso
 */

#include "can_functions.hpp"
#include <can_interface.hpp>
extern CAN_HandleTypeDef hcan1;

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

void can_main_frame_send(uint16_t adc_susp_right, uint16_t adc_susp_left, uint8_t brake_pressure_front, uint8_t brake_pressure_back){

	AQ_main aq_frame{
		adc_susp_right,
		adc_susp_left, // i brake balance
		brake_pressure_front, // pressure of braking lquid front in %
		brake_pressure_back, // pressure of braking lquid back in %
	};

	auto aq_main_frame = PUTM_CAN::Can_tx_message<AQ_main>(aq_frame, can_tx_header_AQ_MAIN);
	auto status = aq_main_frame.send(hcan1);
	 if (HAL_StatusTypeDef::HAL_OK != status)
	  {
	    Error_Handler();
	  };

}

