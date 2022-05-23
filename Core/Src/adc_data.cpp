/*
 * adc_data.cpp
 *
 *  Created on: May 23, 2022
 *      Author: molso
 */

#include "adc_data.hpp"
#include <array>

adc_data::adc_data() {
	// TODO Auto-generated constructor stub
	HAL_ADC_Start_DMA(&hadc1, data.begin(), data.size());

}
uint16_t operator[](Analog_channel channel) {
	return data[static_cast<size_t>(channel)];
}

