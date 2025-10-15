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

#ifndef CHANNEL_H_
#define CHANNEL_H_


//--------------------------------------------
uint8_t lll_chan_iso_event(uint16_t counter, uint16_t chan_id, uint8_t *chan_map, uint8_t chan_count, uint16_t *prn_s, uint8_t *remap_idx);
uint8_t lll_chan_iso_subevent(uint16_t chan_id, uint8_t *chan_map, uint8_t chan_count, uint16_t *prn_subevent_lu, uint8_t *remap_idx);

//--------------------------------------------
uint8_t csa2_aux_chan(ble_conf_t *conf);
uint8_t csa2_aux_sync_chan(ble_conf_t *conf);
uint8_t csa1_acl_chan(ble_conf_t *conf);
uint8_t csa2_acl_chan(ble_conf_t *conf);
uint8_t csa2_iso_event_chan(ble_conf_t *conf);
uint8_t csa2_iso_subevent_chan(ble_conf_t *conf);
uint8_t csa2_pawr_ind_chan(ble_conf_t *conf);
uint8_t csa2_pawr_rsp_chan(ble_conf_t *conf);

#endif /* CHANNEL_H_ */

