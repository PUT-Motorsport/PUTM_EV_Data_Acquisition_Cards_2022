/*
 * meansure_task_.cpp
 *
 *  Created on: 23 maj 2022
 *      Author: molso
 */

#include <meansure_task.hpp>


uint16_t braking_pressure_bar_calculation(uint16_t adc_value){
/* 0 --- 5V
 * 0 --- 250bar
 * f1 = 50
 * 0 --- 5v
 * 0 --- 4095
 * f2 = 13 107
 * f3 = f2/f1==262
 */
	uint16_t val = adc_value / 819;
	return val;

}

uint8_t braking_pressure_percent_calculation(uint16_t adc_value){
	uint16_t val =  (adc_value * 100) / 4095;
	return (uint8_t)val;

}
