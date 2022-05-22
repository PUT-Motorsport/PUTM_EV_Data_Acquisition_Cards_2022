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
#include "can_interface.hpp"

CAN_HandleTypeDef hcan1;
void init_can_filter(CAN_FilterTypeDef sFilterConfig);
void init_can_config(void);

#endif /* INC_CAN_FUNCTIONS_HPP_ */
