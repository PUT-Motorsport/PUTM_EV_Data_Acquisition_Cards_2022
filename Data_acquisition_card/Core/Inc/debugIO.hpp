#pragma once

#include "main.h"
#include <cstdint>

enum class State : uint8_t {
    OK = 15,
    CANMissedMsgWarning = 1,
	BadSensorRead,
	AssertionFailed,
	IOSPIError,
	HALError,
	HALAssertionFailed,
	BadInitSeq,
	CANError,
};

namespace {
State state_ = State::OK;
}

namespace Device {

inline void setState(State state) {

    if (static_cast<uint8_t>(state) <= static_cast<uint8_t>(state_)) { //status level must persist unless a more important error has appeared
        return;
    }
    state_ = state;

    //display a 0-15 number using 4 available leds
    uint8_t state_value{static_cast<uint8_t>(state_)};

    if (state_value &= (1u << 0)) {
    	HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
    }
    if (state_value &= (1u << 1)) {
    	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    }
    if (state_value &= (1u << 2)) {
    	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    }
    if (state_value &= (1u << 3)) {
    	HAL_GPIO_WritePin(LED3_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    }
}

} //namespace Device

__attribute__((noreturn)) inline void unrecoverableError(State errorType) {
	Device::setState(errorType);

	__disable_irq();

	while(true);
}

#ifndef NDEBUG
#define RUNTIME_ASSERT(statement) {if (not (statement)) unrecoverableError(State::AssertionFailed);}
#else
#define RUNTIME_ASSERT(statement) ;
#endif
