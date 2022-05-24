/*
 * adc_data.hpp
 *
 *  Created on: May 23, 2022
 *      Author: molso
 */

#ifndef SRC_ADC_DATA_HPP_
#define SRC_ADC_DATA_HPP_
#include "main.h"
#include <array>
enum struct Analog_channel {
	BRAKE_FRONT, BRAKE_BACK, SUSP_R, ANALOG_1, ANALOG_2, SUSP_L
};
extern ADC_HandleTypeDef hadc1;
class adc_data {
public:
	std::array<uint16_t, 6> data { };
	adc_data();
	uint16_t operator[](Analog_channel channel);
	uint16_t braking_pressure_bar_calculation(uint16_t adc_value);
	uint8_t braking_pressure_percent_calculation(uint16_t pressure_in_bar);

};

#endif /* SRC_ADC_DATA_HPP_ */
