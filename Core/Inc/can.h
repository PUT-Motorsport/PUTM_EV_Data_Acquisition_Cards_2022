/*
 * CAN_driver.h
 *
 *  Created on: 19 kwi 2022
 *      Author: molso
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "stm32l4xx_hal.h"
#include<stdbool.h>
void can_filter_init(void);
bool can_send_frame(CAN_TxHeaderTypeDef *pHeader, uint8_t aData[], uint32_t *pTxMailbox);
void can_tx_header_init(void);
void can_init(void);
#endif /* INC_CAN_H_ */
