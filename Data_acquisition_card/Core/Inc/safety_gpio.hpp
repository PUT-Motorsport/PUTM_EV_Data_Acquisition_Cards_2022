#pragma once

#include "main.h"
#include "common_defs.hpp"
#include <array>
#include <span>

namespace {
std::array<bool, SAFETY_ARRAY_SIZE> safety_state;
}

[[nodiscard]] inline std::span<bool> get_safety_state() {
	/*
	 * Safety has inverted logic
	 * */
	safety_state[static_cast<std::size_t>(Safety::Driver_kill)] = 	not HAL_GPIO_ReadPin(Kill_driver_sense_GPIO_Port, Kill_driver_sense_Pin);
	safety_state[static_cast<std::size_t>(Safety::Right_kill)] = 	not HAL_GPIO_ReadPin(Kill_right_sense_GPIO_Port, Kill_right_sense_Pin);
	safety_state[static_cast<std::size_t>(Safety::Left_kill)] = 	not HAL_GPIO_ReadPin(Kill_left_sense_GPIO_Port, Kill_left_sense_Pin);
	safety_state[static_cast<std::size_t>(Safety::BSPD)] = 			not HAL_GPIO_ReadPin(BSPD_Sense_GPIO_Port, BSPD_Sense_Pin);
	safety_state[static_cast<std::size_t>(Safety::Overtravel)] = 	not HAL_GPIO_ReadPin(Overtravel_sense_GPIO_Port, Overtravel_sense_Pin);

	return std::span{safety_state};		//return a non owning reference to the safety state
}
