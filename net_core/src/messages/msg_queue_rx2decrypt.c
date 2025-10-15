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

#include <stddef.h>    /* size_t, NULL */
#include <stdint.h>    /* uint32_t */
#include <stdbool.h>   /* bool */
#include <string.h>    /* memcpy */
#include <assert.h>    /* assert */
#include "radio_packet.h"
#include "msg_queue_rx2decrypt.h"

//--------------------------------------------
#define MSG_QUEUE_RX2DECRYPT_SIZE          10
#define MSG_QUEUE_RX2DECRYPT_BUF_SIZEOF   (MSG_QUEUE_RX2DECRYPT_SIZE * sizeof(rx2decrypt_item_t))

//--------------------------------------------
static msg_queue_rx2decrypt_t msg_queue;
static uint8_t buffer[MSG_QUEUE_RX2DECRYPT_BUF_SIZEOF];

//--------------------------------------------
void msg_queue_rx2decrypt_init(void)
{
	msg_queue.buffer = buffer;
	msg_queue.size = MSG_QUEUE_RX2DECRYPT_SIZE;
	msg_queue.head = 0;
	msg_queue.count = 0;
	msg_queue.tail = 0;
}

//--------------------------------------------
bool msg_queue_rx2decrypt_put_item(uint8_t *data, uint8_t need_to_decrypt, uint8_t direction, uint32_t timestamp)
{
	assert(data);
	assert(msg_queue.buffer);
	assert(msg_queue.size);

	bool result = false;

	if (msg_queue.count < msg_queue.size)
	{
		memcpy(msg_queue.buffer + (msg_queue.head * sizeof(rx2decrypt_item_t)), (void *)data, BLE_PDU_MAX_SIZE);
		*(msg_queue.buffer + (msg_queue.head * sizeof(rx2decrypt_item_t)) + BLE_PDU_MAX_SIZE) = need_to_decrypt;
		*(msg_queue.buffer + (msg_queue.head * sizeof(rx2decrypt_item_t)) + BLE_PDU_MAX_SIZE + 1) = direction;
		*(uint32_t *)(msg_queue.buffer + (msg_queue.head * sizeof(rx2decrypt_item_t)) + BLE_PDU_MAX_SIZE + 2) = timestamp;
		*(msg_queue.buffer + (msg_queue.head * sizeof(rx2decrypt_item_t)) + BLE_PDU_MAX_SIZE + 6) = 1;
		msg_queue.head = (msg_queue.head + 1) % msg_queue.size;
		msg_queue.count++;
		result = true;
	}

	return result;
}

//--------------------------------------------
bool msg_queue_rx2decrypt_get_first_encrypted_item_ptr(rx2decrypt_item_t **item)
{
	assert(msg_queue.buffer);
	assert(msg_queue.size);

	bool result = false;

	for (size_t cnt = 0; cnt < msg_queue.count; cnt++)
	{
		size_t tail = (msg_queue.tail + cnt) % msg_queue.size;
		*item = (rx2decrypt_item_t *)&msg_queue.buffer[tail * sizeof(rx2decrypt_item_t)];
		if ((*item)->need_to_decrypt == true)
		{
			result = true;
			break;
		}
	}

	return result;
}

//--------------------------------------------
bool msg_queue_rx2decrypt_get_first_item_ptr_if_decrypted(rx2decrypt_item_t **item)
{
	assert(msg_queue.buffer);
	assert(msg_queue.size);

	bool result = false;

	if (msg_queue.count)
	{
		size_t tail = (msg_queue.tail) % msg_queue.size;
		*item = (rx2decrypt_item_t *)&msg_queue.buffer[tail * sizeof(rx2decrypt_item_t)];
		if ((*item)->need_to_decrypt == false)
		{
			result = true;
		}
	}

	return result;
}

//--------------------------------------------
bool msg_queue_rx2decrypt_delete_first_item_if_decrypted(void)
{
	assert(msg_queue.buffer);
	assert(msg_queue.size);

	rx2decrypt_item_t *item;
	bool result = false;

	if (msg_queue.count)
	{
		size_t tail = (msg_queue.tail) % msg_queue.size;
		item = (rx2decrypt_item_t *)&msg_queue.buffer[tail * sizeof(rx2decrypt_item_t)];
		if (item->need_to_decrypt == false)
		{
			msg_queue.tail = (msg_queue.tail + 1) % msg_queue.size;
			msg_queue.count--;
			result = true;
		}
	}

	return result;
}
