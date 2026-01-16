/*
* Copyright (c) 2026 Vladimir Alemasov
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
#include <string.h>    /* memcpy */
#include <nrf.h>


//--------------------------------------------
// tAAR (Address resolution time per IRK) = 6.1us max
// tAAR,1 (Time for address resolution of 1 IRK) = 1 + 6.1 = 7.1us max
bool hal_aar_resolve(uint8_t *irk, uint8_t *s0_L_s1_rpa)
{
	static uint8_t tmp[4];
	bool res = false;

	// Disable AAR_CCM interrupt
	NVIC_DisableIRQ(AAR_CCM_IRQn);

	// Enable AAR
	NRF_AAR_NS->ENABLE = (AAR_ENABLE_ENABLE_Enabled << AAR_ENABLE_ENABLE_Pos);

	// Disable interrupt on AAR_RESOLVED(CCR_ENDCRYPT) state
	NRF_AAR_NS->INTENCLR = AAR_INTENCLR_RESOLVED_Msk;

	// Clear events
	NRF_AAR_NS->EVENTS_END = 0;
	NRF_AAR_NS->EVENTS_RESOLVED = 0;
	NRF_AAR_NS->EVENTS_NOTRESOLVED = 0;

	// Init
	NRF_AAR_NS->NIRK = 1;
	NRF_AAR_NS->IRKPTR = (uint32_t)irk;
	NRF_AAR_NS->ADDRPTR = (uint32_t)s0_L_s1_rpa;
	NRF_AAR_NS->SCRATCHPTR = (uint32_t)tmp;

	// Start resolution
	NRF_AAR_NS->TASKS_START = 1;

	// Wait for result
	while (!(NRF_AAR_NS->EVENTS_END));
	res = NRF_AAR_NS->EVENTS_RESOLVED;

	// Clear events
	NRF_AAR_NS->EVENTS_END = 0;
	NRF_AAR_NS->EVENTS_RESOLVED = 0;
	NRF_AAR_NS->EVENTS_NOTRESOLVED = 0;

	// Disable AAR
	NRF_AAR_NS->ENABLE = (AAR_ENABLE_ENABLE_Disabled << AAR_ENABLE_ENABLE_Pos);

	return res;
}
