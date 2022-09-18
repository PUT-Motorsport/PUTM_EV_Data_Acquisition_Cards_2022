#pragma once

#include <cstdint>
#include "debugIO.hpp"

namespace BrakePressure {

[[nodiscard]] constexpr inline uint16_t normalize_to_kpa(uint16_t arg) {
	/* Sensor max pressure: 5 bar = 500 kPa
	 * Assuming max voltage @ 5 bar
	 * Assuming the pressure-voltage curve is linear
	 * The raw ADC reading can be divided by 8.19 to get the pressure in kPa
	 * TODO: well, dangerous assumptions
	 * */
	constexpr float ADC_TO_KPA = 8.19f;

	return arg / ADC_TO_KPA;
}


} // namespace brake pressure
