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
 * 0 --- 65 535
 * f2 = 13 107
 * f3 = f2/f1==262
 */
	return adc_value / 262;

}

uint8_t braking_pressure_percent_calculation(uint16_t pressure_in_bar){
	return (pressure_in_bar * 100) / 655535;
}
