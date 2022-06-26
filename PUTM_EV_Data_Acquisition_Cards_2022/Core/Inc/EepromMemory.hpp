/*
 * EepromMemory.h
 *
 *  Created on: Jun 3, 2022
 *      Author: molso
 */

#ifndef EEPROMMEMORY_H_
#define EEPROMMEMORY_H_
#include<main.h>
class EepromMemory {
public:
	EepromMemory(I2C_HandleTypeDef *hi2c, uint8_t devAddress);
	void read(uint8_t memAddress, uint8_t* result);
	void write(uint8_t memAddress, uint8_t buffer);

protected:
	I2C_HandleTypeDef *hi2c;
	uint8_t devAddress;
};

#endif /* EEPROMMEMORY_H_ */
