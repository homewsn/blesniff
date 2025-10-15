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

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

//--------------------------------------------
typedef struct ble_time_cfg
{
	uint32_t window_size_rt;
	uint32_t window_offset_rt;
	uint32_t interval_rt;
	uint16_t latency;
	uint32_t timeout_rt;
} ble_time_cfg_t;

//--------------------------------------------
#define MAX_BIS_EVENTS_IN_BIG_EVENT        3
#define MAX_BIS_SUBEVENTS_IN_BIS_EVENT     3
#define MAX_BIS_SUBEVENTS_IN_BIG_EVENT     (MAX_BIS_EVENTS_IN_BIG_EVENT * MAX_BIS_SUBEVENTS_IN_BIS_EVENT + 1)

//--------------------------------------------
typedef struct ble_conf
{
	struct
	{
		radio_params_t params;
	} adv;
	struct
	{
		radio_params_t params;
		uint32_t aux_offset_rt;
		uint32_t sync_offset_rt;
		uint32_t sync_offset_unit;
		uint32_t interval_rt;
		uint16_t channel_identifier;
		uint16_t event_counter;
		uint8_t next_channel;
		pkt_aux_t aux_type;
		pkt_aux_t next_aux_type;
		struct
		{
			uint32_t ind_address;
			uint32_t rsp_address;
			uint8_t num_subevents;
			uint32_t subevent_interval_rt;
			uint32_t response_slot_delay_rt;
			uint32_t response_slot_spacing_rt;
			uint8_t subevent_counter;
			uint32_t response_slots_spacing_rt;
			uint32_t radio_rsp_end_time;
			uint32_t radio_sync_start_time;
			uint32_t calc_radio_sync_start_time;
			uint16_t channel_identifier;
		} pawr;
		struct
		{
			uint16_t conn_event_count;
			uint16_t last_pa_event_counter;
			uint8_t sid;
			uint8_t sca;
			ble_phy_t phy;
			uint16_t sync_conn_event_count;
		} past;
		struct
		{
			uint8_t channel_map[5];
			uint8_t channel_count;
			uint8_t sca;
			uint32_t max_window_drift_rt;
			uint32_t radio_sync_start_time;
			uint32_t calc_radio_sync_start_time;
		} sync;
	} aux;
	struct
	{
		radio_params_t params;
		bool use_csa2;
		struct
		{
			uint8_t hop;
			uint8_t unmapped_channel;
		} csa1;
		struct
		{
			uint16_t channel_identifier;
		} csa2;
		ble_time_cfg_t time_cfg;
		uint32_t max_window_drift_rt;
		uint16_t event_counter;
		bool anchor_packet;
		uint32_t radio_anchor_time;
		uint32_t calc_radio_anchor_time;
		uint32_t conn_interval_counter;
		uint8_t packet_direction;
		bool first_packet;
		bool encrypted_packet;
		bool connection_terminated;
	} acl;
	struct
	{
		radio_params_t params;
		uint16_t channel_identifier;
		uint16_t prn_s;
		uint8_t remap_idx;
		uint16_t event_counter;
		uint8_t sub_event_counter;
		uint8_t num_sub_events;
		uint32_t sub_interval_rt;
		uint32_t iso_interval_rt;
		bool anchor_packet;
		uint32_t radio_anchor_time;
		uint32_t calc_radio_anchor_time;
		uint8_t packet_direction;
		bool first_packet;
		struct
		{
			uint32_t cis_offset_rt;
			uint32_t cig_sync_delay_rt;
			uint32_t cis_sync_delay_rt;
			uint64_t cis_event_counter;
		} cig;
		struct
		{
			uint32_t big_offset_rt;
			uint8_t num_bis;
			uint8_t nse;
			uint8_t bn;
			uint32_t bis_spacing_rt;
			uint32_t seed_access_address;
			uint32_t base_crc_init;
			uint64_t bis_payload_count;
			uint8_t next_channel;
			uint32_t next_address;
			uint16_t next_channel_identifier;
			uint32_t next_crc_init;
			radio_params_t params[MAX_BIS_SUBEVENTS_IN_BIG_EVENT];
			uint16_t channel_identifier[MAX_BIS_SUBEVENTS_IN_BIG_EVENT];
			uint32_t stop_time_offset[MAX_BIS_SUBEVENTS_IN_BIG_EVENT];
			uint16_t prn_s[MAX_BIS_EVENTS_IN_BIG_EVENT];
			uint8_t remap_idx[MAX_BIS_EVENTS_IN_BIG_EVENT];
			uint8_t bis_event_counter;
			uint8_t bis_subevent_counter;
			bool interleaved_arrangement;
			uint8_t channel_map[5];
			uint8_t channel_count;
		} big;
	} iso;
	uint32_t sca_c;
	uint32_t sca_p;
	uint8_t channel_map[5];
	uint8_t channel_count;
} ble_conf_t;

#endif /* CONFIGURATION_H_ */

