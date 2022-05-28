/*
 * LEDs.cpp
 *
 *  Created on: 27 maj 2022
 *      Author: molso
 */




#include "main.h"
#include "LEDs.hpp"

void Set_Error()
{
	HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin, GPIO_PIN_RESET);

}
void Set_OK()
{
	HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin, GPIO_PIN_RESET);
}
void Clear_OK()
{
	HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin, GPIO_PIN_SET);
}
void Set_Warning()
{
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin, GPIO_PIN_RESET);
}
void Clear_Warning()
{
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin, GPIO_PIN_SET);

}
void Set_CAN_Warning()
{
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin, GPIO_PIN_RESET);
}
void Clear_CAN_Warning()
{
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin, GPIO_PIN_SET);
}
void Clear_All()
{
	HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin, GPIO_PIN_SET);
}
void Set_All()
{
	HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin, GPIO_PIN_RESET);
}
void Start()
{
	Set_All();
	HAL_Delay(200);
	Clear_All();
}
