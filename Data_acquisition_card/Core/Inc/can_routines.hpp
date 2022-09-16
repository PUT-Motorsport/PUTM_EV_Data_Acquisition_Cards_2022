#pragma once

#include "safety_gpio.hpp"

bool send_main_frame(volatile uint16_t * adc1_buff, volatile uint16_t * adc2_buff) {
	auto safety = get_safety_state();

}

bool send_gyroscope_frame(uint16_t x, uint16_t y, uint16_t z) {

}

bool send_acc_frame(uint16_t x, uint16_t y, uint16_t z) {

}
