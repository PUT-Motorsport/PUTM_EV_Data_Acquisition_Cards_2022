#include "imu_sensor.hpp"

#include "ISM330dhcx/ism330dhcx_reg.h"
#include "debugIO.hpp"

namespace {
stmdev_ctx_t * instance_;

std::array<int16_t, 3> raw_acc_data{};
std::array<int16_t, 3> raw_gyro_data{};

//sensor settings
constexpr auto SENSOR_BOOT_TIME{20};
}

namespace IMU {

bool initialize(stmdev_ctx_t * instance) {
	instance_ = instance;

	instance_->mdelay(SENSOR_BOOT_TIME);

	uint8_t devID;

	ism330dhcx_device_id_get(instance_, &devID);

	if (devID not_eq ISM330DHCX_ID) {
		Device::setState(State::IOSPIError);
	//	return false;
	}

	//Restore the default configuration
	if (ism330dhcx_reset_set(instance_, PROPERTY_ENABLE)) {
		if (not reinitialize(100)) {
			unrecoverableError(State::IOSPIError);
		}
	}
	//Start device configuration
	ism330dhcx_device_conf_set(instance_, PROPERTY_ENABLE);
	// Enable Block Data Update
	ism330dhcx_block_data_update_set(instance_, PROPERTY_ENABLE);
	//Set Output Data Rate
	ism330dhcx_xl_data_rate_set(instance_, ISM330DHCX_XL_ODR_12Hz5);
	ism330dhcx_gy_data_rate_set(instance_, ISM330DHCX_GY_ODR_12Hz5);
	//Set full scale
	ism330dhcx_xl_full_scale_set(instance_, ISM330DHCX_2g);
	ism330dhcx_gy_full_scale_set(instance_, ISM330DHCX_1000dps);

	ism330dhcx_xl_hp_path_on_out_set(instance_, ISM330DHCX_LP_ODR_DIV_100);
	ism330dhcx_xl_filter_lp2_set(instance_, PROPERTY_ENABLE);

	return true;
}

bool reinitialize(uint32_t timeout) {
	if (not instance_) {
		return false;	//sensor never initialzed
	}
	uint32_t timeStarted{HAL_GetTick()};
	uint8_t reinitSuccessful = false;
	do {
		ism330dhcx_reset_get(instance_, &reinitSuccessful);
	} while (timeStarted + timeout > HAL_GetTick() and
			reinitSuccessful == false);

	return reinitSuccessful;
}

//returns the newest available data
std::array<IMU::IMUData_t, 3> get_acc_data() {

	if (instance_ == nullptr) {
		unrecoverableError(State::BadInitSeq);
	}

	uint8_t reg;
	ism330dhcx_xl_flag_data_ready_get(instance_, &reg);

	if (reg) {
		ism330dhcx_acceleration_raw_get(instance_, raw_acc_data.data());
	}
	return raw_acc_data;
}

std::array<IMU::IMUData_t, 3> get_gyro_data() {

	if (instance_ == nullptr) {
		unrecoverableError(State::BadInitSeq);
	}

	uint8_t reg;
	ism330dhcx_gy_flag_data_ready_get(instance_, &reg);

	if (reg) {
		ism330dhcx_angular_rate_raw_get(instance_, raw_gyro_data.data());
	}
	return raw_gyro_data;
}

}	//namespace IMU
