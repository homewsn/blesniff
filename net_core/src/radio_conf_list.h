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

#ifndef RADIO_CONF_LIST_H
#define RADIO_CONF_LIST_H

//--------------------------------------------
typedef struct
{
	uint8_t opcode;
	uint16_t instant;
	ble_phy_t phy;
	ble_time_cfg_t time_cfg;
	uint8_t channel_map[5];
} radio_conf_item_t;

//--------------------------------------------
#define RADIO_CONF_LIST_SIZE  4

//--------------------------------------------
typedef struct
{
	radio_conf_item_t item[RADIO_CONF_LIST_SIZE];
	bool used[RADIO_CONF_LIST_SIZE];
} radio_conf_list_t;

//--------------------------------------------
void radio_conf_list_init(void);
bool radio_conf_list_put(radio_conf_item_t item);
bool radio_conf_list_get(uint16_t next_event_counter, radio_conf_item_t *item);

#endif /* RADIO_CONF_LIST_H */