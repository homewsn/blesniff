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
#include "platform.h"

#ifdef NRF5340_XXAA_APPLICATION
//--------------------------------------------
#define ID_MUTEX     48

//--------------------------------------------
void hal_mutex_init(void)
{
	// Set non-secure attribute for MUTEX
	NRF_SPU_S->PERIPHID[ID_MUTEX].PERM &= ~SPU_PERIPHID_PERM_SECATTR_Msk;
}

//--------------------------------------------
void hal_mutex_lock(uint8_t number)
{
	assert(number <= 15);
	while (NRF_MUTEX_NS->MUTEX[number]);
}

//--------------------------------------------
void hal_mutex_unlock(uint8_t number)
{
	assert(number <= 15);
	NRF_MUTEX_NS->MUTEX[number] = 0;
}

#else
//--------------------------------------------
#define APP_CORE_MUTEX_NS    ((NRF_MUTEX_Type *)0x40030000)

//--------------------------------------------
void hal_mutex_lock(uint8_t number)
{
	assert(number <= 15);
	while (APP_CORE_MUTEX_NS->MUTEX[number]);
}

//--------------------------------------------
void hal_mutex_unlock(uint8_t number)
{
	assert(number <= 15);
	APP_CORE_MUTEX_NS->MUTEX[number] = 0;
}
#endif
