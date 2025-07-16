/*
 * ex1.c
 *
 *  Created on: Jul 14, 2025
 *      Author: Admin
 */
#include "ex1.h"



uint8_t crc8_sae_j1850(uint8_t *data, uint8_t len){
	return 1;
}

bool verify_checksum(uint8_t *data){
	uint8_t calculated_checksum = crc8_sae_j1850(data, 7);
	return (calculated_checksum == data[7]);
}

bool verify_msg_counter(uint8_t current_msg_counter){
    if (prev_msg_counter == 0xFF) {
        // First-time receiving: accept any value
        prev_msg_counter = msg_counter;
        return true;
    }

    uint8_t expected_counter = (prev_msg_counter + 1) % 16;
    if (msg_counter == expected_counter) {
        prev_msg_counter = msg_counter;
        return true;
    } else {
        // Error: out-of-order, repeated, or skipped message
        prev_msg_counter = msg_counter; // Still update to avoid cascading errors
        return false;
    }
}

void CAN2_prep_data_tx(uint8_t value1, uint8_t value2){
	CAN2_DATA_TX[0] = value1;
	CAN2_DATA_TX[1] = value2;
	CAN2_DATA_TX[6] = msg_counter & 0x0F;
	CAN2_DATA_TX[7] = crc8_sae_j1850(CAN2_DATA_TX, 7);
}

void CAN1_prep_data_tx(){
	CAN1_DATA_TX[0] = CAN1_DATA_RX[0];
	CAN1_DATA_TX[1] = CAN1_DATA_RX[1];
	CAN1_DATA_TX[2] = CAN1_DATA_RX[0] + CAN1_DATA_RX[1];
	CAN1_DATA_TX[6] = msg_counter & 0x0F;

	//Thêm phần nếu ấn nút thì checksum ở đây sẽ thành sai.
	CAN1_DATA_TX[7] = crc8_sae_j1850(CAN1_DATA_TX, 7);
}

void CAN2_SendMessage(uint8_t* data)
{
    uint32_t txMailbox;
	//Cập nhật msg_counter
	msg_counter ++;

	//Gửi dữ liệu header + CAN2_DATA_TX
	HAL_CAN_AddTxMessage(&hcan2, &CAN2_pHeader, CAN2_DATA_TX, &txMailbox);
}

void CAN1_SendMessage(uint8_t* data)
{
    uint32_t txMailbox;
	//Cập nhật msg_counter
	msg_counter ++;

	//Gửi dữ liệu header + CAN1_DATA_TX
	HAL_CAN_AddTxMessage(&hcan1, &CAN1_pHeader, CAN1_DATA_TX, &txMailbox);

}
