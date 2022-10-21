#include "imu_sensor.hpp"

#include <algorithm>
#include "ISM330dhcx/ism330dhcx_reg.h"
#include "debugIO.hpp"
#include "kalman.hpp"
#include "usb_communication.hpp"

namespace {
stmdev_ctx_t * instance_;

constexpr auto initialGuess{0};
constexpr auto initialVariance{10000};

constexpr auto Kalman_F = 1;
constexpr auto Kalman_H = 1;
constexpr auto Kalman_Q = 1;	//fixme: to be fine-tuned
constexpr auto Kalman_R = 20;

std::array<KalmanFilter1D<IMU::IMUData_t>, 3> acc_data_kalman{KalmanFilter1D<IMU::IMUData_t>{initialGuess, initialVariance},
													  KalmanFilter1D<IMU::IMUData_t>{initialGuess, initialVariance},
													  KalmanFilter1D<IMU::IMUData_t>{initialGuess, initialVariance}};
std::array<KalmanFilter1D<IMU::IMUData_t>, 3> gyro_data_kalman{KalmanFilter1D<IMU::IMUData_t>{initialGuess, initialVariance},
													  KalmanFilter1D<IMU::IMUData_t>{initialGuess, initialVariance},
													  KalmanFilter1D<IMU::IMUData_t>{initialGuess, initialVariance}};

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

	auto predicate = [](KalmanFilter1D<IMUData_t>& filter){
		filter.F = Kalman_F;
		filter.H = Kalman_H;
		filter.Q = Kalman_Q;
		filter.R = Kalman_R;
	};
	// initialize the Kalman filers
	std::for_each(acc_data_kalman.begin(), acc_data_kalman.end(), predicate);
	std::for_each(gyro_data_kalman.begin(), gyro_data_kalman.end(), predicate);

	return true;
}

bool reinitialize(uint32_t timeout) {
	if (not instance_) {
		return false;	//sensor never initialized
	}
	uint32_t timeStarted{HAL_GetTick()};
	uint8_t reinitSuccessful = false;
	do {
		ism330dhcx_reset_get(instance_, &reinitSuccessful);
	} while (timeStarted + timeout > HAL_GetTick() and
			reinitSuccessful == false);

	return reinitSuccessful;
}

void updateSensorData() {
	if (instance_ == nullptr) {
		unrecoverableError(State::BadInitSeq);
	}

	uint8_t reg;
	ism330dhcx_xl_flag_data_ready_get(instance_, &reg);

	IMUData_t rawData[3]{};

	if (reg) {
		ism330dhcx_acceleration_raw_get(instance_, rawData);

		USB_DebugData::set_raw_acc(rawData[0], rawData[1], rawData[2]);

		acc_data_kalman[0].iterate(rawData[0]);
		acc_data_kalman[1].iterate(rawData[1]);
		acc_data_kalman[2].iterate(rawData[2]);


	}

	ism330dhcx_gy_flag_data_ready_get(instance_, &reg);

	if (reg) {
		ism330dhcx_angular_rate_raw_get(instance_, rawData);

		gyro_data_kalman[0].iterate(rawData[0]);
		gyro_data_kalman[1].iterate(rawData[1]);
		gyro_data_kalman[2].iterate(rawData[2]);
	}

}

//returns the newest available data
std::array<IMU::IMUData_t, 3> get_acc_data() {

	std::array<IMUData_t, 3> imuAccData;
	imuAccData[0] = acc_data_kalman[0].get();
	imuAccData[1] = acc_data_kalman[1].get();
	imuAccData[2] = acc_data_kalman[2].get();

	return imuAccData;
}

std::array<IMU::IMUData_t, 3> get_gyro_data() {

	std::array<IMUData_t, 3> imuGyroData;
	imuGyroData[0] = gyro_data_kalman[0].get();
	imuGyroData[1] = gyro_data_kalman[1].get();
	imuGyroData[2] = gyro_data_kalman[2].get();

	return imuGyroData;
}

}	//namespace IMU
