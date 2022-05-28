/*
 * meansure_task_.cpp
 *
 *  Created on: 23 maj 2022
 *      Author: molso
 */

#include <meansure_task.hpp>

uint16_t braking_pressure_bar_calculation(uint16_t adc_value) {
	/* 0 --- 5V
	 * 0 --- 250bar
	 */
	uint16_t val = adc_value / 819;
	return val;

}

uint8_t braking_pressure_percent_calculation(uint16_t adc_value) {
	uint16_t val = (adc_value * 100) / 4095;
	return (uint8_t) (val <= 100) ? val : 100;

}
