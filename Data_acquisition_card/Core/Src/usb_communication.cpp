#include "usb_communication.hpp"

#include <cstdio>
#include "usbd_cdc_if.h"

namespace {
USB_DebugData::PotentiometerType_t potentiometer_left;
USB_DebugData::PotentiometerType_t potentiometer_right;

IMU::IMUData_t raw_acc_x, raw_acc_y, raw_acc_z;
IMU::IMUData_t kalman_acc_x, kalman_acc_y, kalman_acc_z;

constexpr auto TEXT_BUFFER_SIZE{256};
char text_buffer[TEXT_BUFFER_SIZE];
}

namespace USB_DebugData {

bool send() {

	auto const buff_effective_length = std::snprintf(text_buffer, TEXT_BUFFER_SIZE, "%u, %u, %i, %i, %i, %i, %i, %i,\r\n",
			potentiometer_left, potentiometer_right, raw_acc_x, raw_acc_y, raw_acc_z,
													 kalman_acc_x, kalman_acc_y, kalman_acc_z);
	//snprintf will protect against overwriting and null terminate the array
	return CDC_Transmit_FS(reinterpret_cast<uint8_t *>(text_buffer), buff_effective_length);
}

void set_potentiometers(PotentiometerType_t left, PotentiometerType_t right) {
	potentiometer_left = left;
	potentiometer_right = right;
}
void set_raw_acc(IMU::IMUData_t x, IMU::IMUData_t y, IMU::IMUData_t z) {
	raw_acc_x = x;
	raw_acc_y = y;
	raw_acc_z = z;
}
void set_kalman_acc(IMU::IMUData_t x, IMU::IMUData_t y, IMU::IMUData_t z) {
	kalman_acc_x = x;
	kalman_acc_y = y;
	kalman_acc_z = z;
}

}	//namespace USB_DebugData
