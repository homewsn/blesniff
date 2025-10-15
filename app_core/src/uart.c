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

#include <stdint.h>    /* uint8_t ... uint64_t */
#include <stddef.h>    /* size_t */
#include <stdbool.h>   /* bool */
#include "cmsis_compiler.h"  /* __disable_irq, __enable_irq */
#include "hal_perm.h"
#include "hal_uarte.h"
#include "hal_mutex.h"
#include "hal_ipc.h"
#include "ipc_msg_types.h"
#include "radio_packet.h"
#include "msg_queue_app2net.h"
#include "msg_queue_net2app.h"
#include "serial.h"
#include "msg_queue_ipc2serial.h"
#include "msg_queue_serial2ipc.h"

//--------------------------------------------
bool serial_enabled;

//--------------------------------------------
void uart_init(void)
{
	// Init hardware
	hal_perm_init();
	hal_ipc_init();
	hal_ipc_set_mem_callback(mem_callback);
	hal_mutex_init();
	hal_uarte_init();
	hal_uarte_set_rx_callback(serial_rx_callback);
	// Init queues
	msg_queue_app2net_init();
	msg_queue_net2app_init();
	msg_queue_ipc2serial_init();
	msg_queue_serial2ipc_init();

	serial_enabled = true;
}

//--------------------------------------------
void uart_loop(void)
{
	static msg_ipc2serial_t msg;
	bool res;

	serial_rx();

	__disable_irq();
	res = msg_queue_ipc2serial_get(&msg);
	__enable_irq();
	if (!res)
	{
		return;
	}

	hal_uarte_write(msg.data, msg.size);
}
