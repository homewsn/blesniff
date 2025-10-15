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
#include "msg_queue.h"
#include "radio_packet.h"
#include "msg_queue_ipc2serial.h"
#include "mutexes.h"
#include "hal_mutex.h"

//--------------------------------------------
#define MSG_QUEUE_IPC2SERIAL_SIZE          50
#define MSG_QUEUE_IPC2SERIAL_BUF_SIZEOF   (MSG_QUEUE_IPC2SERIAL_SIZE * sizeof(msg_ipc2serial_t))

//--------------------------------------------
static msg_queue_t msg_queue;
static uint8_t buffer[MSG_QUEUE_IPC2SERIAL_BUF_SIZEOF];

//--------------------------------------------
void msg_queue_ipc2serial_init(void)
{
	msg_queue_init(&msg_queue,
		buffer,
		sizeof(msg_ipc2serial_t),
		MSG_QUEUE_IPC2SERIAL_SIZE,
		MSG_QUEUE_IPC2SERIAL_MUTEX);
}

//--------------------------------------------
bool msg_queue_ipc2serial_put(msg_ipc2serial_t *msg)
{
	if (!msg_queue_put(&msg_queue, (uint8_t *)msg))
	{
		return false;
	}
	return true;
}

//--------------------------------------------
bool msg_queue_ipc2serial_get(msg_ipc2serial_t *msg)
{
	return msg_queue_get(&msg_queue, (uint8_t *)msg);
}
