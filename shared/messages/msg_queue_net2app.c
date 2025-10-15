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
#include "ipc_msg_types.h"
#include "radio_packet.h"
#include "shared_memory.h"
#include "mutexes.h"
#include "hal_mutex.h"
#include "hal_ipc.h"

//--------------------------------------------
msg_queue_t *msg_queue_net2app = (msg_queue_t *)MSG_QUEUE_NET2APP_ADDR;

#ifdef NRF5340_XXAA_APPLICATION
//--------------------------------------------
uint8_t *buffer_net2app = (uint8_t *)MSG_QUEUE_NET2APP_BUF_ADDR;

//--------------------------------------------
void msg_queue_net2app_init(void)
{
	msg_queue_init(msg_queue_net2app,
		buffer_net2app,
		sizeof(msg_net2app_t),
		MSG_QUEUE_NET2APP_SIZE,
		MSG_QUEUE_NET2APP_MUTEX);
}
#endif

//--------------------------------------------
bool msg_queue_net2app_put(msg_net2app_t *msg)
{
	if (!msg_queue_put(msg_queue_net2app, (uint8_t *)msg))
	{
		return false;
	}
	hal_ipc_send_mem_signal();
	return true;
}

//--------------------------------------------
bool msg_queue_net2app_get(msg_net2app_t *msg)
{
	return msg_queue_get(msg_queue_net2app, (uint8_t *)msg);
}
