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

#ifndef SHARED_MEMORY_BASE_H_
#define SHARED_MEMORY_BASE_H_

//------------------------------
#define SHARED_MEMORY_BASE              0x20070000
#define SHARED_MEMORY_RAMREGION_SIZE    8192

//------------------------------
#if (SHARED_MEMORY_BASE & 0x3) != 0
	#error "SHARED_MEMORY_BASE must be 32-bit aligned"
#endif
#if (SHARED_MEMORY_BASE < 0x20070000) || (SHARED_MEMORY_BASE > 0x2007FFFF)
	#error "SHARED_MEMORY_BASE must be in range 0x20070000 - 0x2007FFFF"
#endif
#if (SHARED_MEMORY_BASE % SHARED_MEMORY_RAMREGION_SIZE) != 0
	#error "SHARED_MEMORY_BASE must match the start of the RAM region, see 'Figure 214: RAM memory regions' in the nRF5340 datasheet"
#endif

#endif // SHARED_MEMORY_BASE_H_
