/*
 * ism_task.cpp
 *
 *  Created on: May 21, 2022
 *      Author: molso
 */

#include <ism_task.hpp>
#include <ISM330DHCXSensor.hpp>

	ISM330DHCXSensor AccGyr;
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

void ism330_read(void){
	AccGyr.ACC_GetAxes(Acceleration);
	AccGyr.GYRO_GetAxes(AngularRate);
}
