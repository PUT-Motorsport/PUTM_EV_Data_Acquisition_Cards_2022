/*
 * adc_data.cpp
 *
 *  Created on: May 23, 2022
 *      Author: molso
 */

#include "adc_data.hpp"

adc_data::adc_data() {
	// TODO Auto-generated constructor stub
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) data.begin(), (uint32_t) data.size());

}
uint16_t adc_data::operator[](Analog_channel channel) {
	return data[static_cast<size_t>(channel)];
}

uint16_t adc_data::braking_pressure_bar_calculation(uint16_t adc_value) {
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

uint8_t adc_data::braking_pressure_percent_calculation(uint16_t pressure_in_bar) {
	return (pressure_in_bar * 100) / 655535;
}

