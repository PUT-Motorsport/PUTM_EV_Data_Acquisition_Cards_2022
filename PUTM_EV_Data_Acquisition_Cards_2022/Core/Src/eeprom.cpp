/*
 * eeprom.cpp
 *
 *  Created on: Jun 3, 2022
 *      Author: molso
 */
#include




	HAL_I2C_Mem_Write(&hi2c2, 0b10100000, 0x10, 1, (uint8_t*)&test, sizeof(test), HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c2, 0b10100000, 0x10, 1, (uint8_t*)&result, sizeof(result), HAL_MAX_DELAY);
