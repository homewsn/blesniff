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

#include <nrf.h>
#include "cmsis_compiler.h"  /* __disable_irq, __enable_irq */
#include "irq_priorities.h"
#include "hal_uarte.h"

//--------------------------------------------
static uarte_rx_callback_t rx_callback;
static uint8_t rx_byte;

//--------------------------------------------
void hal_uarte_init(void)
{
	NRF_UARTE0_S->PSEL.TXD = 20;
	NRF_UARTE0_S->PSEL.RXD = 22;
	NRF_UARTE0_S->BAUDRATE = UARTE_BAUDRATE_BAUDRATE_Baud1M;

	NRF_UARTE0_S->RXD.PTR = (uint32_t)&rx_byte;
	NRF_UARTE0_S->RXD.MAXCNT = 1;

	NRF_UARTE0_S->INTENSET = UARTE_INTENSET_RXDRDY_Msk | UARTE_INTENSET_ENDRX_Msk;
    
	// Enable UARTE interrupt
	NVIC_SetPriority(SERIAL0_IRQn, UARTE0_IRQ_PRIORITY);
	NVIC_EnableIRQ(SERIAL0_IRQn);

	NRF_UARTE0_S->ENABLE = UARTE_ENABLE_ENABLE_Enabled;
	NRF_UARTE0_S->TASKS_STARTRX = 1;
}

//--------------------------------------------
void hal_uarte_write(uint8_t *data, uint32_t size)
{
	__disable_irq();

	NRF_UARTE0_S->EVENTS_ENDTX = 0;

    // Init DMA
    NRF_UARTE0_S->TXD.PTR = (uint32_t)data;
    NRF_UARTE0_S->TXD.MAXCNT = size;
    
	// Start TX
	NRF_UARTE0_S->TASKS_STARTTX = 1;

	__enable_irq();

    while (!NRF_UARTE0_S->EVENTS_ENDTX);
}

//--------------------------------------------
void hal_uarte_set_rx_callback(uarte_rx_callback_t callback)
{
	rx_callback = callback;
}

//--------------------------------------------
void SERIAL0_IRQHandler(void)
{
	if (NRF_UARTE0_S->EVENTS_RXDRDY)
	{
		NRF_UARTE0_S->EVENTS_RXDRDY = 0;
		if (rx_callback)
		{
			rx_callback(rx_byte);
		}
	}
	if (NRF_UARTE0_S->EVENTS_ENDRX)
	{
		NRF_UARTE0_S->EVENTS_ENDRX = 0;
		NRF_UARTE0_S->TASKS_STARTRX = 1;
	}
}
