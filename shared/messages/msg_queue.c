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

#include <stdint.h>    /* uint32_t */
#include <stdbool.h>   /* bool */
#include <string.h>    /* memcpy */
#include <assert.h>    /* assert */
#include "msg_queue.h"
#include "hal_mutex.h"

#ifdef NRF5340_XXAA_APPLICATION
//--------------------------------------------
void msg_queue_init(msg_queue_t *queue, uint8_t *buffer, uint32_t item_size, uint32_t size, uint32_t mutex)
{
	assert(queue);
	assert(buffer);
	assert(item_size);
	assert(size);

	queue->buffer = buffer;
	queue->item_size = item_size;
	queue->size = size;
	queue->mutex = mutex;
	queue->head = 0;
	queue->count = 0;
	queue->tail = 0;
}
#endif

//--------------------------------------------
bool msg_queue_put(msg_queue_t *queue, uint8_t *data)
{
	assert(queue);
	assert(data);
	assert(queue->item_size);
	assert(queue->size);
	assert(queue->buffer);

	bool result = false;

	hal_mutex_lock(queue->mutex);

	if (queue->count < queue->size)
	{
		memcpy(&queue->buffer[queue->head * queue->item_size], data, queue->item_size);
		queue->head = (queue->head + 1) % queue->size;
		queue->count++;
		result = true;
	}

	hal_mutex_unlock(queue->mutex);
	return result;
}

//--------------------------------------------
bool msg_queue_get(msg_queue_t *queue, uint8_t *data)
{
	assert(queue);
	assert(data);
	assert(queue->item_size);
	assert(queue->size);
	assert(queue->buffer);

	bool result = false;

	hal_mutex_lock(queue->mutex);

	if (queue->count > 0)
	{
		memcpy(data, &queue->buffer[queue->tail * queue->item_size], queue->item_size);
		queue->tail = (queue->tail + 1) % queue->size;
		queue->count--;
		result = true;
	}

	hal_mutex_unlock(queue->mutex);

	return result;
}
