/*
 * digital_inputs.hpp
 *
 *  Created on: May 27, 2022
 *      Author: molso
 */

#ifndef INC_DIGITAL_INPUTS_HPP_
#define INC_DIGITAL_INPUTS_HPP_

#include "main.h"
#include <array>
//// PB0---PB4
//enum struct Digital_channel {
//	kill_driver = kill_driver_input_Pin,
//	kill_right = kill_right_input_Pin,
//	kill_left = kill_left_input_Pin,
//	inertia = inertia_input_Pin,
//	overtravel = overtravel_input_Pin,
//};

uint8_t digital_pin[5] = {kill_driver_input_Pin, kill_right_input_Pin, kill_left_input_Pin, inertia_input_Pin, overtravel_input_Pin};

uint8_t scan_digital_inputs(void);



#endif /* INC_DIGITAL_INPUTS_HPP_ */
