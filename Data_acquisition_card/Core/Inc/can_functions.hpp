#pragma once

#include "safety_gpio.hpp"
#include "debugIO.hpp"

namespace Canbus {

//will initialize the can with a blank filter
//will not return on fail
void initialize(CAN_HandleTypeDef * instance);

void send_main_frame(ADC1_Data volatile const * const adc1_data, ADC2_Data volatile const * const adc2_data);

void send_gyroscope_frame(int16_t x, int16_t y, int16_t z);

void send_acc_frame(int16_t x, int16_t y, int16_t z);

void send_rtd_frame();

}	//namespace Canbus
