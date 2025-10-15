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
#include <stdlib.h>     /* size_t */
#include <stdbool.h>    /* bool */
#include <string.h>     /* memcpy */
#include "radio_packet.h"
#include "ble.h"
#include "configuration.h"
#include "radio_conf_list.h"

//--------------------------------------------
static radio_conf_list_t radio_conf_list;

//--------------------------------------------
void radio_conf_list_init(void)
{
	for (size_t cnt = 0; cnt < RADIO_CONF_LIST_SIZE; cnt++)
	{
		radio_conf_list.used[cnt] = false;
	}
}

//--------------------------------------------
bool radio_conf_list_put(radio_conf_item_t item)
{
	for (size_t cnt = 0; cnt < RADIO_CONF_LIST_SIZE; cnt++)
	{
		if (!radio_conf_list.used[cnt])
		{
			radio_conf_list.item[cnt] = item;
			radio_conf_list.used[cnt] = true;
			return true;
		}
	}
	return false;
}

//--------------------------------------------
bool radio_conf_list_get(uint16_t next_event_counter, radio_conf_item_t *item)
{
	size_t cnt;
	for (cnt = 0; cnt < RADIO_CONF_LIST_SIZE; cnt++)
	{
		if (!radio_conf_list.used[cnt])
		{
			continue;
		}
		if (radio_conf_list.item[cnt].instant == next_event_counter)
		{
			break;
		}
	}
	if (cnt == RADIO_CONF_LIST_SIZE)
	{
		return false;
	}
	memcpy(item, &radio_conf_list.item[cnt], sizeof(radio_conf_item_t));
	radio_conf_list.used[cnt] = false;
	return true;
}
