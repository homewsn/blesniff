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

#include <stdbool.h>   /* bool */
#include <nrf.h>
#include <nrf53_erratas.h>
#include "platform.h"

//--------------------------------------------
void hal_reset_network_force_off(bool hold)
{
	if (hold)
	{
		NRF_RESET_S->NETWORK.FORCEOFF = RESET_NETWORK_FORCEOFF_FORCEOFF_Hold << RESET_NETWORK_FORCEOFF_FORCEOFF_Pos;
	}
	else if (nrf53_errata_161())
	{
		*(volatile uint32_t *)0x50005618UL = 1UL;
		NRF_RESET_S->NETWORK.FORCEOFF = RESET_NETWORK_FORCEOFF_FORCEOFF_Release << RESET_NETWORK_FORCEOFF_FORCEOFF_Pos;
		delay_us(5);
		NRF_RESET_S->NETWORK.FORCEOFF = RESET_NETWORK_FORCEOFF_FORCEOFF_Hold << RESET_NETWORK_FORCEOFF_FORCEOFF_Pos;
		delay_us(1);
		NRF_RESET_S->NETWORK.FORCEOFF = RESET_NETWORK_FORCEOFF_FORCEOFF_Release << RESET_NETWORK_FORCEOFF_FORCEOFF_Pos;
		*(volatile uint32_t *)0x50005618UL = 0UL;
	}
	else
	{
		NRF_RESET_S->NETWORK.FORCEOFF = RESET_NETWORK_FORCEOFF_FORCEOFF_Release << RESET_NETWORK_FORCEOFF_FORCEOFF_Pos;
	}
}
