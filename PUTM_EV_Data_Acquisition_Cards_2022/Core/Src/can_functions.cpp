/*
 * can_functions.cpp
 *
 *  Created on: 22 maj 2022
 *      Author: molso
 */

#include "PUTM_EV_CAN_LIBRARY/lib/can_interface.hpp"
#include <LEDs.hpp>
#include <global_variables.hpp>

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
		Set_CAN_Warning();
		Set_Warning();
		Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan1,
	CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK) {
		Set_CAN_Warning();
		Set_Warning();
		Error_Handler();
	}
}

void can_main_frame_send(uint16_t adc_susp_right, uint16_t adc_susp_left,
		uint16_t brake_pressure_front, Status status) {

//	PUTM_CAN::AQ_main aq_frame { adc_susp_right, adc_susp_left,
//			brake_pressure_front, // pressure of braking liquid front in %
//			brake_pressure_back, // pressure of braking liquid back in %
//	};
	PUTM_CAN::AQ_main aq_frame{};
	aq_frame.suspension_left = adc_susp_left;
	aq_frame.suspension_right = adc_susp_right;
	aq_frame.brake_pressure_front = brake_pressure_front;
	aq_frame.device_state = (PUTM_CAN:: AQ_states) status;


	auto aq_main_frame = PUTM_CAN::Can_tx_message<PUTM_CAN::AQ_main>(aq_frame,
			PUTM_CAN::can_tx_header_AQ_MAIN);
	auto can_status = aq_main_frame.send(hcan1);
	if (HAL_StatusTypeDef::HAL_OK != can_status) {
		Set_CAN_Warning();
		Error_Handler();
	};
}
void can_acceleraton_frame_send(int16_t acc_x, int16_t acc_y, int16_t acc_z) {
	PUTM_CAN::AQ_acceleration aq_frame { acc_x, acc_y, acc_z, };
	auto aq_acc_frame = PUTM_CAN::Can_tx_message<PUTM_CAN::AQ_acceleration>(aq_frame,
			PUTM_CAN::can_tx_header_AQ_ACCELERATION);
	auto status = aq_acc_frame.send(hcan1);
	if (HAL_StatusTypeDef::HAL_OK != status) {
		Set_CAN_Warning();
		Error_Handler();
	};
}
void can_gyroscope_frame_send(int16_t speed_x, int16_t speed_y,
		int16_t speed_z) {
	PUTM_CAN::AQ_gyroscope aq_frame { speed_x, speed_y, speed_z, };
	auto aq_gyr_frame = PUTM_CAN::Can_tx_message<PUTM_CAN::AQ_gyroscope>(aq_frame,
			PUTM_CAN::can_tx_header_AQ_GYROSCOPE);
	auto status = aq_gyr_frame.send(hcan1);
	if (HAL_StatusTypeDef::HAL_OK != status) {
		Set_CAN_Warning();
		Error_Handler();
	};
}

void can_ts_button_frame_send(){
	PUTM_CAN::AQ_ts_button aq_frame{
		.placeholder{1}
	};
	auto aq_ts_frame = PUTM_CAN::Can_tx_message<PUTM_CAN::AQ_ts_button>(aq_frame, PUTM_CAN::can_tx_header_AQ_TS_BUTTON);
	auto status = aq_ts_frame.send(hcan1);
	if (HAL_StatusTypeDef::HAL_OK != status) {
		Set_CAN_Warning();
		Error_Handler();
	};
}

