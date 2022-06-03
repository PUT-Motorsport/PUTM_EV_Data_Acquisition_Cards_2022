/*
 * EepromMemory.cpp
 *
 *  Created on: Jun 3, 2022
 *      Author: molso
 */

#include <EepromMemory.hpp>

EepromMemory::EepromMemory(I2C_HandleTypeDef *hi2c, uint8_t devAddress) {
	// TODO Auto-generated constructor stub
	this->devAddress =devAddress;
	this->hi2c = hi2c;
	HAL_I2C_Init(hi2c);
}

void EepromMemory::read(uint8_t memAddress, uint8_t* result){
	HAL_I2C_Mem_Read(this->hi2c, this->devAddress, memAddress, sizeof(memAddress), result,sizeof(result), HAL_MAX_DELAY);
}

void EepromMemory::write(uint8_t memAddress, uint8_t buffer){
	HAL_I2C_Mem_Write(this->hi2c, this->devAddress, memAddress, sizeof(memAddress), &buffer, sizeof(buffer), HAL_MAX_DELAY);
}
