/*
 * meansure_task_.hpp
 *
 *  Created on: 23 maj 2022
 *      Author: molso
 */

#ifndef INC_MEANSURE_TASK_HPP_
#define INC_MEANSURE_TASK_HPP_

#include <main.h>
#include <cmsis_os.h>

uint16_t braking_pressure_bar_calculation(uint16_t adc_value);
uint8_t braking_pressure_percent_calculation(uint16_t pressure_in_bar);




#endif /* INC_MEANSURE_TASK_HPP_ */
