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

#include <stdint.h>     /* uint8_t ... uint64_t */
#include <stdbool.h>    /* bool */
#include <string.h>     /* memcpy */

//--------------------------------------------
#define CACHE_SIZE              10
#define DEVICE_ADDRESS_LENGTH   6

//--------------------------------------------
typedef struct
{
	uint8_t mac[DEVICE_ADDRESS_LENGTH];
	uint8_t header;
} header_cache_t;

//--------------------------------------------
static header_cache_t cache[CACHE_SIZE];
static size_t write_index;
static bool full;

//--------------------------------------------
void adv_cache_store(uint8_t *mac, uint8_t header)
{
	memcpy(cache[write_index].mac, mac, DEVICE_ADDRESS_LENGTH);
	cache[write_index].header = header;

	write_index++;
	if (write_index >= CACHE_SIZE)
	{
		write_index = 0;
		full = true;
	}
}

//--------------------------------------------
uint8_t adv_cache_fetch(uint8_t *mac)
{
	size_t size = full ? CACHE_SIZE : write_index;
	size_t current_index = write_index;

	for (size_t cnt = 0; cnt < size; cnt++)
	{
		current_index = (current_index == 0) ? CACHE_SIZE - 1 : current_index - 1;
		if (!memcmp(mac, cache[current_index].mac, DEVICE_ADDRESS_LENGTH))
		{
			return cache[current_index].header;
		}
	}
	return 0xFF;
}
