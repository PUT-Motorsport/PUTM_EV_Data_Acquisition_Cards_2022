#include "main.h"

#include "common_defs.hpp"

namespace {
uint32_t lastRTDsignal{};
}
extern bool send_rtd_signal_flag;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == RTD_input_Pin) {
		if ((lastRTDsignal + RTD_SIGNAL_COOLDOWN) > HAL_GetTick()) {
			return;
		}
		send_rtd_signal_flag = true;
		lastRTDsignal = HAL_GetTick();
	}
}
