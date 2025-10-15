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
#include "hal_ecb.h"


//--------------------------------------------
bool hal_ecb_encrypt(ecb_data_t *ecb_data)
{
	NRF_ECB_NS->ECBDATAPTR = (uint32_t)ecb_data;

	// Clear events
	NRF_ECB_NS->EVENTS_ENDECB = 0;
	NRF_ECB_NS->EVENTS_ERRORECB = 0;

	// Start decryption
	NRF_ECB_NS->TASKS_STARTECB = 1;

	// Wait for result
	while (!(NRF_ECB_NS->EVENTS_ENDECB || NRF_ECB_NS->EVENTS_ERRORECB));

	return (NRF_ECB_NS->EVENTS_ENDECB);
}
