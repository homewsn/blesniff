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
#include <stddef.h>     /* size_t */
#include <stdbool.h>    /* bool */
#include <assert.h>     /* assert */
#include "radio_packet.h"
#include "ble.h"
#include "configuration.h"
#include "channel.h"


//--------------------------------------------
static uint8_t lll_chan_sel_2(uint16_t counter, uint16_t chan_id, uint8_t *chan_map, uint8_t chan_count);
static uint8_t lll_chan_sel_1(uint8_t *chan_use, uint8_t hop, uint16_t latency, uint8_t *chan_map,	uint8_t chan_count);

//--------------------------------------------
uint8_t csa2_aux_chan(ble_conf_t *conf)
{
	return lll_chan_sel_2(conf->aux.event_counter, conf->aux.channel_identifier, conf->channel_map, conf->channel_count);
}
//--------------------------------------------
uint8_t csa2_aux_sync_chan(ble_conf_t *conf)
{
	return lll_chan_sel_2(conf->aux.event_counter, conf->aux.channel_identifier, conf->aux.sync.channel_map, conf->aux.sync.channel_count);
}
//--------------------------------------------
uint8_t csa1_acl_chan(ble_conf_t *conf)
{
	return lll_chan_sel_1(&conf->acl.csa1.unmapped_channel, conf->acl.csa1.hop, conf->acl.time_cfg.latency, conf->channel_map, conf->channel_count);
}
//--------------------------------------------
uint8_t csa2_acl_chan(ble_conf_t *conf)
{
	return lll_chan_sel_2(conf->acl.event_counter, conf->acl.csa2.channel_identifier, conf->channel_map, conf->channel_count);
}
//--------------------------------------------
uint8_t csa2_iso_event_chan(ble_conf_t *conf)
{
	return lll_chan_iso_event(conf->iso.event_counter, conf->iso.channel_identifier, conf->channel_map, conf->channel_count, &conf->iso.prn_s, &conf->iso.remap_idx);
}
//--------------------------------------------
uint8_t csa2_iso_subevent_chan(ble_conf_t *conf)
{
	return lll_chan_iso_subevent(conf->iso.channel_identifier, conf->channel_map, conf->channel_count, &conf->iso.prn_s, &conf->iso.remap_idx);
}
//--------------------------------------------
uint8_t csa2_pawr_ind_chan(ble_conf_t *conf)
{
	return lll_chan_sel_2(conf->aux.event_counter ^ conf->aux.pawr.subevent_counter, conf->aux.channel_identifier, conf->aux.sync.channel_map, conf->aux.sync.channel_count);
}
//--------------------------------------------
uint8_t csa2_pawr_rsp_chan(ble_conf_t *conf)
{
	return lll_chan_sel_2(conf->aux.event_counter ^ conf->aux.pawr.subevent_counter, conf->aux.pawr.channel_identifier, conf->aux.sync.channel_map, conf->aux.sync.channel_count);
}

//--------------------------------------------
#define MAX(a, b)  (((a) > (b)) ? (a) : (b))
#define MIN(a, b)  (((a) < (b)) ? (a) : (b))

/*
 * Copyright (c) 2018-2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
 /* Refer to Bluetooth Specification v5.2 Vol 6, Part B, Section 4.5.8.3.2
  * Inputs and basic components, for below operations
  */
//--------------------------------------------
static uint8_t chan_rev_8(uint8_t b)
{
	b = (uint8_t)((((uint32_t)b * 0x0802LU & 0x22110LU) | ((uint32_t)b * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16);
	return b;
}
//--------------------------------------------
static uint16_t chan_perm(uint16_t i)
{
	return (chan_rev_8((i >> 8) & 0xFF) << 8) | chan_rev_8(i & 0xFF);
}
//--------------------------------------------
static uint16_t chan_mam(uint16_t a, uint16_t b)
{
	return ((uint32_t)a * 17U + b) & 0xFFFF;
}
//--------------------------------------------
static uint16_t chan_prn_s(uint16_t counter, uint16_t chan_id)
{
	uint8_t iterate;
	uint16_t prn_s;

	prn_s = counter ^ chan_id;

	for (iterate = 0U; iterate < 3; iterate++)
	{
		prn_s = chan_perm(prn_s);
		prn_s = chan_mam(prn_s, chan_id);
	}

	return prn_s;
}
//--------------------------------------------
static uint16_t chan_prn_e(uint16_t counter, uint16_t chan_id)
{
	uint16_t prn_e;

	prn_e = chan_prn_s(counter, chan_id);
	prn_e ^= chan_id;

	return prn_e;
}
//--------------------------------------------
/* Refer to Bluetooth Specification v5.2 Vol 6, Part B, Section 4.5.8.3
 * Channel Selection algorithm #2, and Section 4.5.8.3.4 Event mapping to used
 * channel index
 */
static uint8_t chan_sel_remap(uint8_t *chan_map, uint8_t chan_index)
{
	uint8_t chan_next;
	uint8_t byte_count;

	chan_next = 0U;
	byte_count = 5U;
	while (byte_count--)
	{
		uint8_t bite;
		uint8_t bit_count;

		bite = *chan_map;
		bit_count = 8U;
		while (bit_count--)
		{
			if (bite & 0x01)
			{
				if (chan_index == 0U)
				{
					break;
				}
				chan_index--;
			}
			chan_next++;
			bite >>= 1;
		}

		if (bit_count < 8)
		{
			break;
		}

		chan_map++;
	}

	return chan_next;
}
//--------------------------------------------
/* Refer to Bluetooth Specification v5.2 Vol 6, Part B, Section 4.5.8.3
 * Channel Selection algorithm #2, and Section 4.5.8.3.1 Overview
 * Below interface is used for ACL connections.
 */
static uint8_t lll_chan_sel_2(uint16_t counter, uint16_t chan_id, uint8_t *chan_map, uint8_t chan_count)
{
	uint8_t chan_next;
	uint16_t prn_e;

	prn_e = chan_prn_e(counter, chan_id);
	chan_next = prn_e % 37;

	if ((chan_map[chan_next >> 3] & (1 << (chan_next % 8))) == 0U)
	{
		uint8_t chan_index;

		chan_index = ((uint32_t)chan_count * prn_e) >> 16;
		chan_next = chan_sel_remap(chan_map, chan_index);

	}
	else
	{
		/* channel can be used, return it */
	}

	return chan_next;
}
//--------------------------------------------
/* Refer to Bluetooth Specification v5.2 Vol 6, Part B, Section 4.5.8.2
* Channel Selection algorithm #1
*/
static uint8_t lll_chan_sel_1(uint8_t *chan_use, uint8_t hop, uint16_t latency, uint8_t *chan_map,	uint8_t chan_count)
{
	uint8_t chan_next;

	chan_next = ((*chan_use) + (hop * (1 + latency))) % 37;
	*chan_use = chan_next;

	if ((chan_map[chan_next >> 3] & (1 << (chan_next % 8))) == 0U) {
		uint8_t chan_index;

		chan_index = chan_next % chan_count;
		chan_next = chan_sel_remap(chan_map, chan_index);

	}
	else {
		/* channel can be used, return it */
	}

	return chan_next;
}
//--------------------------------------------
/* Refer to Bluetooth Specification v5.2 Vol 6, Part B, Section 4.5.8.3
 * Channel Selection algorithm #2, and Section 4.5.8.3.5 Subevent pseudo-random
 * number generation
 */
static uint16_t chan_prn_subevent_se(uint16_t chan_id, uint16_t *prn_subevent_lu)
{
	uint16_t prn_subevent_se;
	uint16_t lu;

	lu = *prn_subevent_lu;
	lu = chan_perm(lu);
	lu = chan_mam(lu, chan_id);

	*prn_subevent_lu = lu;

	prn_subevent_se = lu ^ chan_id;

	return prn_subevent_se;
}
//--------------------------------------------
/* Refer to Bluetooth Specification v5.2 Vol 6, Part B, Section 4.5.8.3
 * Channel Selection algorithm #2, and Section 4.5.8.3.4 Event mapping to used
 * channel index
 *
 * Below function is used in the context of next subevent, return remapping
 * index.
 */
static uint8_t chan_sel_remap_index(uint8_t *chan_map, uint8_t chan_index)
{
	uint8_t octet_count;
	uint8_t remap_index;

	remap_index = 0U;
	octet_count = 5U;
	while (octet_count--) {
		uint8_t octet;
		uint8_t bit_count;

		octet = *chan_map;
		bit_count = 8U;
		while (bit_count--) {
			if (!chan_index) {
				return remap_index;
			}
			chan_index--;

			if (octet & 0x01) {
				remap_index++;
			}
			octet >>= 1;
		}

		chan_map++;
	}

	return 0;
}
//--------------------------------------------
/* Refer to Bluetooth Specification v5.2 Vol 6, Part B, Section 4.5.8.3
 * Channel Selection algorithm #2, and Section 4.5.8.3.6 Subevent mapping to
 * used channel index
 */
static uint8_t chan_d(uint8_t n)
{
	uint8_t x, y;

	/* Sub-expression to get natural number (N - 5) to be used in the
	 * calculation of d.
	 */
	if (n > 5) {
		x = n - 5;
	}
	else {
		x = 0;
	}

	/* Sub-expression to get natural number ((N - 10) / 2) to be used in the
	 * calculation of d.
	 */
	if (n > 10) {
		y = (n - 10) >> 1;
	}
	else {
		y = 0;
	}

	/* Calculate d using the above sub expressions */
	return MAX(1, MAX(MIN(3, x), MIN(11, y)));
}
//--------------------------------------------
/* Refer to Bluetooth Specification v5.2 Vol 6, Part B, Section 4.5.8.3
 * Channel Selection algorithm #2, and Section 4.5.8.3.1 Overview
 *
 * Below interface is used for ISO first subevent.
 */
uint8_t lll_chan_iso_event(uint16_t counter, uint16_t chan_id, uint8_t *chan_map, uint8_t chan_count, uint16_t *prn_s, uint8_t *remap_idx)
{
	uint8_t chan_idx;
	uint16_t prn_e;

	*prn_s = chan_prn_s(counter, chan_id);
	prn_e = *prn_s ^ chan_id;
	chan_idx = prn_e % 37;

	if ((chan_map[chan_idx >> 3] & (1 << (chan_idx % 8))) == 0U) {
		*remap_idx = ((uint32_t)chan_count * prn_e) >> 16;
		chan_idx = chan_sel_remap(chan_map, *remap_idx);

	}
	else {
		*remap_idx = chan_sel_remap_index(chan_map, chan_idx);
	}

	return chan_idx;
}
//--------------------------------------------
/* Refer to Bluetooth Specification v5.2 Vol 6, Part B, Section 4.5.8.3
 * Channel Selection algorithm #2, and Section 4.5.8.3.1 Overview
 *
 * Below interface is used for ISO next subevent.
 */
uint8_t lll_chan_iso_subevent(uint16_t chan_id, uint8_t *chan_map, uint8_t chan_count, uint16_t *prn_subevent_lu, uint8_t *remap_idx)
{
	uint16_t prn_subevent_se;
	uint8_t d;
	uint8_t x;

	prn_subevent_se = chan_prn_subevent_se(chan_id, prn_subevent_lu);

	d = chan_d(chan_count);

	/* Sub-expression to get natural number (N - 2d + 1) to be used in the
	 * calculation of d.
	 */
	if ((chan_count + 1) > (d << 1)) {
		x = (chan_count + 1) - (d << 1);
	}
	else {
		x = 0;
	}

	*remap_idx = ((((uint32_t)prn_subevent_se * x) >> 16) +
		d + *remap_idx) % chan_count;

	return chan_sel_remap(chan_map, *remap_idx);
}
