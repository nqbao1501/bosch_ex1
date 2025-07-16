/*
 * ex1.h
 *
 *  Created on: Jul 15, 2025
 *      Author: Admin
 */

#ifndef INC_EX1_H_
#define INC_EX1_H_
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

extern uint8_t CAN1_DATA_TX[8];
extern uint8_t CAN1_DATA_RX[8];
extern uint8_t CAN2_DATA_TX[8];
extern uint8_t CAN2_DATA_RX[8];
extern uint8_t msg_counter;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern CAN_TxHeaderTypeDef CAN1_pHeader;
extern CAN_TxHeaderTypeDef CAN2_pHeader;

extern uint8_t prev_msg_counter;

uint8_t crc8_sae_j1850(uint8_t *data, uint8_t len);
bool verify_checksum(uint8_t *data);
bool verify_msg_counter(uint8_t current_msg_counter);
void CAN2_prep_data_tx(uint8_t value1, uint8_t value2);
void CAN1_prep_data_tx();
void CAN2_SendMessage(uint8_t* data);
void CAN1_SendMessage(uint8_t* data);

#endif /* INC_EX1_H_ */
