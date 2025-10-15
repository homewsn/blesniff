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

#ifndef MSG_QUEUE_H_
#define MSG_QUEUE_H_

//--------------------------------------------
typedef struct
{
	uint8_t *buffer;
	uint32_t size;
	uint32_t item_size;
	uint32_t head;
	uint32_t tail;
	uint32_t count;
	uint32_t mutex;
} msg_queue_t;

//--------------------------------------------
#ifdef NRF5340_XXAA_APPLICATION
void msg_queue_init(msg_queue_t *queue, uint8_t *buffer, uint32_t item_size, uint32_t size, uint32_t mutex);
#endif
bool msg_queue_put(msg_queue_t *queue, uint8_t *data);
bool msg_queue_get(msg_queue_t *queue, uint8_t *data);

#endif /* MSG_QUEUE_H_ */