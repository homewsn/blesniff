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
#include <assert.h>    /* assert */
#include <nrf.h>
#include "irq_priorities.h"
#include "ipc_channels.h"
#include "dppi_channels.h"
#include "hal_ipc.h"

#ifdef NRF5340_XXAA_APPLICATION
//--------------------------------------------
static mem_read_callback mem_callback;

//--------------------------------------------
void hal_ipc_init(void)
{
	// Enable IPC channels
	NRF_IPC_S->SEND_CNF[APP2NET_CPU_MEMORY] = (1UL << APP2NET_CPU_MEMORY);
	NRF_IPC_S->RECEIVE_CNF[NET2APP_CPU_MEMORY] = (1UL << NET2APP_CPU_MEMORY);

	// Clear events register
	NRF_IPC_S->EVENTS_RECEIVE[NET2APP_CPU_MEMORY] = 0;

	// Enable IPC channel interrupts
	NRF_IPC_S->INTENSET |= (1UL << NET2APP_CPU_MEMORY);

	// Enable IPC interrupt
	NVIC_SetPriority(IPC_IRQn, IPC_IRQ_PRIORITY);
	NVIC_EnableIRQ(IPC_IRQn);
}

//--------------------------------------------
void hal_ipc_send_mem_signal(void)
{
	NRF_IPC_S->TASKS_SEND[APP2NET_CPU_MEMORY] = 1;
}

//--------------------------------------------
void hal_ipc_set_mem_callback(mem_read_callback callback)
{
	mem_callback = callback;
}

//--------------------------------------------
void IPC_IRQHandler(void)
{
	if (NRF_IPC_S->EVENTS_RECEIVE[NET2APP_CPU_MEMORY])
	{
		NRF_IPC_S->EVENTS_RECEIVE[NET2APP_CPU_MEMORY] = 0;
		if (mem_callback)
		{
			mem_callback();
		}
	}
}

#else

//--------------------------------------------
static mem_read_callback mem_callback;

//--------------------------------------------
void hal_ipc_init(void)
{
	// Enable IPC channels
	NRF_IPC_NS->RECEIVE_CNF[APP2NET_CPU_MEMORY] = (1UL << APP2NET_CPU_MEMORY);
	NRF_IPC_NS->SEND_CNF[NET2APP_CPU_MEMORY] = (1UL << NET2APP_CPU_MEMORY);

	// Clear events register
	NRF_IPC_NS->EVENTS_RECEIVE[APP2NET_CPU_MEMORY] = 0;

	// Enable IPC channel interrupts
	NRF_IPC_NS->INTENSET |= (1UL << APP2NET_CPU_MEMORY);

	// Enable IPC interrupt
	NVIC_SetPriority(IPC_IRQn, IPC_IRQ_PRIORITY);
	NVIC_EnableIRQ(IPC_IRQn);
}

//--------------------------------------------
void hal_ipc_send_mem_signal(void)
{
	NRF_IPC_NS->TASKS_SEND[NET2APP_CPU_MEMORY] = 1;
}

//--------------------------------------------
void hal_ipc_set_mem_callback(mem_read_callback callback)
{
	mem_callback = callback;
}

//--------------------------------------------
void IPC_IRQHandler(void)
{
	if (NRF_IPC_NS->EVENTS_RECEIVE[APP2NET_CPU_MEMORY])
	{
		NRF_IPC_NS->EVENTS_RECEIVE[APP2NET_CPU_MEMORY] = 0;
		if (mem_callback)
		{
			mem_callback();
		}
	}
}

#endif