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
extern ADC_HandleTypeDef hadc1;
enum struct Analog_channel {
	BRAKE_FRONT, BRAKE_BACK, SUSP_R, ANALOG_1, ANALOG_2, SUSP_L
};
class adc_data {
public:
	static std::array<uint16_t, 120> data;
	adc_data();
	uint16_t operator[](Analog_channel channel);
	uint16_t smooth_value(Analog_channel channel);
};

#endif /* SRC_ADC_DATA_HPP_ */
