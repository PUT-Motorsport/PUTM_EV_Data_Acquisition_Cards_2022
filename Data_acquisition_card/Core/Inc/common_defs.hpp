/*
 * common_defs.hpp
 *
 *  Created on: Sep 16, 2022
 *      Author: szymo
 */

#ifndef INC_COMMON_DEFS_HPP_
#define INC_COMMON_DEFS_HPP_


constexpr std::size_t ADC_BUFFER_SIZE = 3;

enum class ADC1_Data : uint8_t {
    BrakePressure1,
    BrakePressure2,
    SuspensionR,
};
enum class ADC2_Data : uint8_t {
    Analog1,
    Analog2,
    SuspensionL,
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

#endif /* INC_COMMON_DEFS_HPP_ */
