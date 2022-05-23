/*
 * adc_data.hpp
 *
 *  Created on: May 23, 2022
 *      Author: molso
 */

#ifndef SRC_ADC_DATA_HPP_
#define SRC_ADC_DATA_HPP_
#include "main.h"

enum struct adc_channel{
	BRAKE_FRONT,
	BRAKE_BACK,
	SUSP_R,
	ANALOG_1,
	ANALOG_2,
	SUSP_L
};
class adc_data {
public:
	std::array<uint16_t, 6> data{};
	adc_data();
};

#endif /* SRC_ADC_DATA_HPP_ */
