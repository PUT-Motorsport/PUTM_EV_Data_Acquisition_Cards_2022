/*
 * digital_inputs.cpp
 *
 *  Created on: May 27, 2022
 *      Author: molso
 */


#include "digital_inputs.hpp"


uint8_t scan_digital_inputs(void)
{
	uint8_t result = 0;
	for (int var = 0; var < sizeof(digital_pin); ++var) {
		if (HAL_GPIO_ReadPin(GPIOB, digital_pin[var])!=1)
		{
			result |= (1<<var);
		}

	}
	return result;
}
