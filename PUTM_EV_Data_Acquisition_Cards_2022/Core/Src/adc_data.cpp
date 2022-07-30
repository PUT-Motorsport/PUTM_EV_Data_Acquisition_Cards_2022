/*
 * adc_data.cpp
 *
 *  Created on: May 23, 2022
 *      Author: molso
 */

#include "adc_data.hpp"
std::array<uint16_t, 120> adc_data::data { };
extern adc_data adc_data;
adc_data::adc_data() {
	//HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) data.begin(), (uint32_t) data.size());
}

uint16_t adc_data::operator[](Analog_channel channel)
{
	uint32_t sum = 0;
	uint32_t k = 0;
	for(int i = uint8_t(channel); i<(sizeof(data)/sizeof(uint16_t)); i = i+6)
	{
		sum = sum + data[static_cast<size_t>(i)];
		k++;
	}
	uint32_t s = (sum/(k));
	return (sum/(k));
}
uint16_t adc_data::smooth_value(Analog_channel channel)
{

}
