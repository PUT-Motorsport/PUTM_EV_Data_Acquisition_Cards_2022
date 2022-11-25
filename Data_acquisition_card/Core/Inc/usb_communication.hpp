#pragma once

#include <cstdint>
#include <cstdio>
#include "imu_sensor.hpp"



namespace USB_DebugData {

using PotentiometerType_t = uint16_t;

bool send();

void set_potentiometers(PotentiometerType_t left, PotentiometerType_t right);
void set_raw_acc(IMU::IMUData_t x, IMU::IMUData_t y, IMU::IMUData_t z);
void set_kalman_acc(IMU::IMUData_t x, IMU::IMUData_t y, IMU::IMUData_t z);

}	//namespace USB_DebugData
