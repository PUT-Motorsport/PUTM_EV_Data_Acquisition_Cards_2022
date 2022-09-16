#pragma once

//suppress -Wmissing-field-initializers and -Wvolatile (C++20) triggered by HAL
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wvolatile"

constexpr std::size_t ADC_BUFFER_SIZE = 3;

struct ADC1_Data {
	uint16_t BrakePressure1;
	uint16_t BrakePressure2;
	uint16_t SuspensionR;
};

struct ADC2_Data {
	uint16_t Analog1;
	uint16_t Analog2;
	uint16_t SuspensionL;
};

constexpr std::size_t SAFETY_ARRAY_SIZE = 5;
enum class Safety : uint8_t {
	Driver_kill,
	Left_kill,
	Right_kill,
	BSPD,
	Overtravel,
};

constexpr std::uint32_t FRAME_TO_FRAME_TIME = 10;

constexpr std::uint32_t BRAKING_PRESSURE_THRESHOLD = 5000;	//fixme: adjust to correct value
