#pragma once

/*
 * This is the C++ wrapper of ISM330DHCX low-level functions.
 * The following code is heavily inspired by examples provided by ST.
 * https://github.com/STMicroelectronics/ism330dhcx-pid
 * */

#include <array>
#include <span>
#include "ISM330dhcx/ism330dhcx_reg.h"

namespace IMU {

bool initialize(stmdev_ctx_t * instance);
bool reinitialize(uint32_t timeout);

std::array<float, 3> get_acc_data();
std::array<float, 3> get_gyro_data();

} //namespace IMU

