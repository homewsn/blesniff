/*
* Copyright (c) 2025 Vladimir Alemasov
* All rights reserved
*
* This program and the accompanying materials are distributed under
* the terms of GNU General Public License version 2
* as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/

#ifndef MSG_QUEUE_RX2DECRYPT_H_
#define MSG_QUEUE_RX2DECRYPT_H_

//--------------------------------------------
typedef struct
{
	uint8_t *buffer;
	uint32_t size;
	uint32_t head;
	uint32_t tail;
	uint32_t count;
} msg_queue_rx2decrypt_t;

//--------------------------------------------
#pragma pack(1)
typedef struct
{
	uint8_t data[BLE_PDU_MAX_SIZE];
	uint8_t need_to_decrypt;
	uint8_t direction;
	uint32_t timestamp;
	uint8_t mic_status;
} rx2decrypt_item_t;
#pragma pack()

//--------------------------------------------
void msg_queue_rx2decrypt_init(void);
bool msg_queue_rx2decrypt_put_item(uint8_t *data, uint8_t need_to_decrypt, uint8_t direction, uint32_t timestamp);
bool msg_queue_rx2decrypt_get_first_encrypted_item_ptr(rx2decrypt_item_t **item);
bool msg_queue_rx2decrypt_get_first_item_ptr_if_decrypted(rx2decrypt_item_t **item);
bool msg_queue_rx2decrypt_delete_first_item_if_decrypted(void);

#endif /* MSG_QUEUE_RX2DECRYPT_H_ */