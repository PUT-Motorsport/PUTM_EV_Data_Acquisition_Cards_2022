/*
 * adc_data.cpp
 *
 *  Created on: May 23, 2022
 *      Author: molso
 */

#include "adc_data.hpp"
std::array<uint16_t, 6> adc_data::data { };
adc_data::adc_data() {
	// TODO Auto-generated constructor stub
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) data.begin(), (uint32_t) data.size());

}
uint16_t adc_data::operator[](Analog_channel channel) {
	return data[static_cast<size_t>(channel)];
}



