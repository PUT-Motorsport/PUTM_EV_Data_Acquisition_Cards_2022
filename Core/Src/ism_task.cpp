/*
 * ism_task.cpp
 *
 *  Created on: May 21, 2022
 *      Author: molso
 */

#include <ism_task.hpp>
#include <ISM330DHCXSensor.hpp>

ISM330DHCXSensor AccGyr;
//uint32_t acceleration[3] = {0};
//uint32_t angular_rate[3] = {0};

void ism330_setup(void) {

	AccGyr.begin();
	AccGyr.ACC_Enable();
	AccGyr.GYRO_Enable();
	// Configure ODR and FS of the acc and gyro
	AccGyr.ACC_SetOutputDataRate(SENSOR_ODR);
	AccGyr.ACC_SetFullScale(ACC_FS);
	AccGyr.GYRO_SetOutputDataRate(SENSOR_ODR);
	AccGyr.GYRO_SetFullScale(GYR_FS);
	// Configure FIFO BDR for acc and gyro
	AccGyr.FIFO_ACC_Set_BDR(SENSOR_ODR);
	AccGyr.FIFO_GYRO_Set_BDR(SENSOR_ODR);
	// Set FIFO in Continuous mode
	AccGyr.FIFO_Set_Mode(ISM330DHCX_STREAM_MODE);
}

void ism330_read(int32_t *acceleration, int32_t *angular_rate) {
	AccGyr.ACC_GetAxes(acceleration);
	AccGyr.GYRO_GetAxes(angular_rate);
}



