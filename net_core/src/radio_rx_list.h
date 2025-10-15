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

#ifndef RADIO_RX_LIST_H
#define RADIO_RX_LIST_H

//--------------------------------------------
typedef enum
{
	FROM_EMPTY = 0,
	FROM_CONNECT_IND,
	FROM_LL_CIS_IND,
	FROM_LL_PERIODIC_SYNC_IND,
	FROM_LL_PERIODIC_SYNC_WR_IND,
	FROM_BLE_CHAN_ACL,
	FROM_BLE_CHAN_ISO,
	FROM_BLE_CHAN_ISO_BIG,
	FROM_ADV_EXT_IND,
	FROM_AUX_ADV_IND,
	FROM_AUX_SYNC_IND,
	FROM_AUX_SYNC_SUBEVENT_IND
} from_func_t;

//--------------------------------------------
typedef enum
{
	BLE_CHAN_NOT_USED = 0,
	BLE_CHAN_ADV,
	BLE_CHAN_AUX,
	BLE_CHAN_AUX_SYNC,
	BLE_CHAN_ACL,
	BLE_CHAN_ISO_CIG,
	BLE_CHAN_ISO_BIG,
	BLE_CHAN_PAWR
} ble_chan_type_t;

//--------------------------------------------
typedef struct
{
	ble_chan_type_t chan_type;
	uint32_t radio_start_time;
	uint32_t radio_end_time;
} radio_rx_item_t;

//--------------------------------------------
#define RADIO_RX_LIST_SIZE  2

//--------------------------------------------
typedef struct
{
	radio_rx_item_t item[RADIO_RX_LIST_SIZE];
} radio_rx_list_t;

//--------------------------------------------
void radio_rx_list_init(void);
bool radio_rx_list_put(radio_rx_item_t item, from_func_t from);
bool radio_rx_list_get(radio_rx_item_t *item);

#endif /* RADIO_RX_LIST_H */