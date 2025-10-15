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
#include "hal_rt.h"
#include "radio_rx_list.h"

//--------------------------------------------
static radio_rx_list_t radio_rx_list;

//--------------------------------------------
#define MIDDLE_VALUE_2_32    0x80000000
#define TIMER_MIDDLE_VALUE   MIDDLE_VALUE_2_32

//--------------- Debugging ------------------
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#define DEBUG_RADIO_TIMING   0
//--------------------------------------------
#if DEBUG_RADIO_TIMING
//--------------------------------------------
typedef struct
{
	from_func_t from;
	uint32_t entry_time;
	uint32_t exit_time;
	uint32_t start_radio_time;
} put_t;
//--------------------------------------------
typedef struct
{
	uint32_t entry_time;
	uint32_t exit_time;
	uint32_t start_radio_time;
	ble_chan_type_t chan_type;
} get_t;
//--------------------------------------------
#define ARRAY_SIZE 10
static volatile put_t put[ARRAY_SIZE];
static volatile get_t get[ARRAY_SIZE];
static size_t put_cnt;
static size_t get_cnt;
//--------------------------------------------
#define __debugbreak() __asm("bkpt 0")
#else
//--------------------------------------------
#define __debugbreak()
#endif
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


//--------------------------------------------
void radio_rx_list_init(void)
{
	for (size_t cnt = 0; cnt < RADIO_RX_LIST_SIZE; cnt++)
	{
		radio_rx_list.item[cnt].chan_type = BLE_CHAN_NOT_USED;
	}
}

//--------------------------------------------
bool radio_rx_list_put(radio_rx_item_t item, from_func_t from)
{
	#if DEBUG_RADIO_TIMING
	if (put_cnt < ARRAY_SIZE)
	{
		put[put_cnt].from = from;
		put[put_cnt].entry_time = radio_tmr_get_radio_time();
		put[put_cnt].start_radio_time = item.radio_start_time;
		if (put[put_cnt].start_radio_time - put[put_cnt].entry_time > TIMER_MIDDLE_VALUE)
		{
			// problem: too late
			// this may mean that the first anchor packet was not received
			__debugbreak();
		}
	}
	#endif

	for (size_t cnt = 0; cnt < RADIO_RX_LIST_SIZE; cnt++)
	{
		if (!radio_rx_list.item[cnt].chan_type)
		{
			radio_rx_list.item[cnt] = item;

			#if DEBUG_RADIO_TIMING
			if (put_cnt < ARRAY_SIZE)
			{
				put[put_cnt].exit_time = radio_tmr_get_radio_time();
				if (++put_cnt == ARRAY_SIZE) put_cnt=0;
			}
			#endif

			return true;
		}
	}
	return false;
}

//--------------------------------------------
bool radio_rx_list_get(radio_rx_item_t *item)
{
	uint32_t time = radio_tmr_get_radio_time();

	#if DEBUG_RADIO_TIMING
	if (get_cnt < ARRAY_SIZE)
	{
		get[get_cnt].entry_time = time;
	}
	#endif

	uint32_t cnt_diff = RADIO_RX_LIST_SIZE;
	uint32_t time_diff = TIMER_MIDDLE_VALUE;

	for (size_t cnt = 0; cnt < RADIO_RX_LIST_SIZE; cnt++)
	{
		if (!radio_rx_list.item[cnt].chan_type)
		{
			continue;
		}
		uint32_t time_diff_item = radio_rx_list.item[cnt].radio_start_time - time;
		if (time_diff_item < time_diff)
		{
			time_diff = time_diff_item;
			cnt_diff = cnt;
		}
		if (time_diff_item > TIMER_MIDDLE_VALUE)
		{
			// problem: too late
			// this may mean that the previous anchor packet was not received
			__debugbreak();
			radio_rx_list.item[cnt].chan_type = BLE_CHAN_NOT_USED;
			#if 0
			return false;
			#endif
		}
	}
	if (cnt_diff == RADIO_RX_LIST_SIZE)
	{
		return false;
	}
	memcpy(item, &radio_rx_list.item[cnt_diff], sizeof(radio_rx_item_t));
	radio_rx_list.item[cnt_diff].chan_type = BLE_CHAN_NOT_USED;

	#if DEBUG_RADIO_TIMING
	if (get_cnt < ARRAY_SIZE)
	{
		get[get_cnt].exit_time = radio_tmr_get_radio_time();
		get[get_cnt].start_radio_time = item->radio_start_time;
		get[get_cnt].chan_type = item->chan_type;
		if (++get_cnt == ARRAY_SIZE) get_cnt=0;
	}
	#endif

	return true;
}
