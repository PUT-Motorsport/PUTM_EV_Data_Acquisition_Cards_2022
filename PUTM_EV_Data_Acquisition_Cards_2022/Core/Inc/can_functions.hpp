/*
 * can_functions.hpp
 *
 *  Created on: 22 maj 2022
 *      Author: molso
 */

#ifndef INC_CAN_FUNCTIONS_HPP_
#define INC_CAN_FUNCTIONS_HPP_
#include <main.h>
#include <cmsis_os.h>
#include <global_variables.hpp>


void init_can_filter(CAN_FilterTypeDef sFilterConfig);
void init_can_config(void);
void can_main_frame_send(uint16_t adc_susp_right, uint16_t adc_susp_left, uint16_t brake_pressure_front, uint16_t brake_pressure_back ,Status status );
void can_acceleraton_frame_send(int16_t acc_x, int16_t acc_y,
		int16_t acc_z);
void can_gyroscope_frame_send(int16_t speed_x, int16_t speed_y,
		int16_t speed_z);
void can_ts_button_frame_send();
#endif /* INC_CAN_FUNCTIONS_HPP_ */
