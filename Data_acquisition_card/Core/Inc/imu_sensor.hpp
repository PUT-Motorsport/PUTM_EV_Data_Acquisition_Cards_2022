#pragma once

#include <array>
#include "ISM330DHCXSensor.hpp"
#include "debugIO.hpp"

namespace {

using SensorData_t = int32_t;

ISM330DHCXSensor sensorInstance_;

constexpr float SENSOR_ODR{52.0f}; //Hertz
constexpr auto ACC_FS{2};	//g
constexpr auto GYR_FS{1000};	//dps
constexpr float MEASUREMENT_TIME_INTERVAL{1000.0f / SENSOR_ODR};
constexpr auto FIFO_SAMPLE_THRESHOLD{199};
constexpr auto FLASH_BUFF_LEN{8192};
}

namespace IMU {

void initialize() {

	/* All functions return 0 in case of success, -1 in case of an error
	 * Therefore only one comparison is needded to decide whether the initialization was successful*/

	int_fast8_t status{};

	// begin function automatically initializes hspi1
	status += sensorInstance_.begin();
	status += sensorInstance_.ACC_Enable();
	status += sensorInstance_.GYRO_Enable();
	status += sensorInstance_.ACC_SetOutputDataRate(SENSOR_ODR);
	status += sensorInstance_.ACC_SetFullScale(ACC_FS);
	status += sensorInstance_.GYRO_SetOutputDataRate(SENSOR_ODR);
	status += sensorInstance_.GYRO_SetFullScale(GYR_FS);
	status += sensorInstance_.FIFO_ACC_Set_BDR(SENSOR_ODR);
	status += sensorInstance_.FIFO_GYRO_Set_BDR(SENSOR_ODR);
	status += sensorInstance_.FIFO_Set_Mode(ISM330DHCX_STREAM_MODE);
}

[[nodiscard]] std::array<SensorData_t, 3> get_acc() {
	std::array<SensorData_t, 3> acc_axes;
	if (not sensorInstance_.ACC_GetAxes(acc_axes.data())) {
		Device::setState(State::BadSensorRead);
	}
	return acc_axes;
}

[[nodiscard]] std::array<SensorData_t, 3> get_gyro() {
	std::array<SensorData_t, 3> gyro_axes;
	if (sensorInstance_.GYRO_GetAxes(gyro_axes.data())) {
		Device::setState(State::BadSensorRead);
	}
	return gyro_axes;
}

} //namespace IMU
