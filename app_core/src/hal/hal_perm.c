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
#include "shared_memory_base.h"

//--------------------------------------------
#define RAM_START_ADDR       0x20000000
#define SHARED_RAMREGION     ((SHARED_MEMORY_BASE - RAM_START_ADDR) / SHARED_MEMORY_RAMREGION_SIZE)

//--------------------------------------------
void hal_perm_init(void)
{
	// Access allowed for net core
	NRF_DCNF_S->EXTPERI[0].PROTECT = 0;

	// Access allowed for net core
	NRF_DCNF_S->EXTRAM[0].PROTECT = 0;
	NRF_SPU_S->RAMREGION[SHARED_RAMREGION].PERM = SPU_RAMREGION_PERM_WRITE_Msk | SPU_RAMREGION_PERM_READ_Msk;
}
