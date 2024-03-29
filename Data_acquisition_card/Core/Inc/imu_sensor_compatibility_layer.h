#pragma once

#include "main.h"
#include "debugIO.hpp"

/*
 *	The IMU sensor requires that user provide the implementation for sending and receiving data.
 *	The functions must have the following signatures:
 *	int32_t platform_write(void *handle, uint8_t Reg, const uint8_t *Bufp, uint16_t len);
 *	int32_t platform_read(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len);
 *
 *	Note: the following is a set of hardcoded functions, depending on the SPI1
 */

//c wrapper to c++ device io api
void spi_error() {
	Device::setState(State::IOSPIError);
}

#ifdef __cplusplus
extern "C" {
#endif

extern SPI_HandleTypeDef hspi1;	//defined in main.cpp

#define SPI_MAX_POLLING_TIME 3 //ms

int32_t spi_write([[maybe_unused]] void * handle, uint8_t reg, const uint8_t *Buffp, uint16_t len) {

	//todo: use the IT or DMA versions
	int32_t result{};
	//select the slave
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	//send the address
	if (HAL_OK != HAL_SPI_Transmit(&hspi1, &reg, 1, SPI_MAX_POLLING_TIME)) {
		spi_error();
		result = -1;
	}
	//send the data
	if (HAL_OK != HAL_SPI_Transmit(&hspi1, const_cast<uint8_t *>(Buffp), len, SPI_MAX_POLLING_TIME)) {
		spi_error();
		result = -1;
	}
	//deselect the slave
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
	return result;
}


int32_t spi_read([[maybe_unused]] void * handle, uint8_t reg, uint8_t * BuffP, uint16_t len) {
	//select the slave
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

	//send the address
	reg |= 0x80;
	int32_t result{};
	if (HAL_OK != HAL_SPI_Transmit(&hspi1, &reg, 1, SPI_MAX_POLLING_TIME)) {
		spi_error();
		result = -1;
	}
	//read the data
	if (HAL_OK != HAL_SPI_Receive(&hspi1, BuffP, len, SPI_MAX_POLLING_TIME)) {
		spi_error();
		result = -1;
	}

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	return result;
}


#ifdef __cplusplus
}	//extern "C"
#endif

