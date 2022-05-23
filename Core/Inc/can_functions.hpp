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


void init_can_filter(CAN_FilterTypeDef sFilterConfig);
void init_can_config(void);
void can_main_frame_send(uint16_t adc_susp_right, uint16_t adc_susp_left, uint8_t brake_pressure_front, uint8_t brake_pressure_back);

#endif /* INC_CAN_FUNCTIONS_HPP_ */
