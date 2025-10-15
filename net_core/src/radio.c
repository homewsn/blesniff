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

#include <stdint.h>    /* uint8_t ... uint64_t */
#include <stddef.h>    /* size_t, NULL */
#include <stdbool.h>   /* bool */
#include <string.h>    /* memcpy */
#include <assert.h>    /* assert */
#include "radio_packet.h"
#include "hal_radio.h"
#include "hal_rt.h"
#include "hal_ipc.h"
#include "crypto.h"
#include "hal_ccm.h"
#include "hal_ecb.h"
#include "hal_dppi.h"
#include "ipc_msg_types.h"
#include "msg_queue_app2net.h"
#include "msg_queue_net2app.h"
#include "ble.h"
#include "configuration.h"
#include "radio_rx_list.h"
#include "adv_header_cache.h"
#include "channel.h"
#include "radio_conf_list.h"
#include "msg_queue_rx2decrypt.h"


//--------------------------------------------
#define MAX_WINDOW_DRIFT_US              30
#if HAL_RADIO_FAST_RUMP_UP
#define MAX_RADIO_STARTUP_DELAY_US       40
#else
#define MAX_RADIO_STARTUP_DELAY_US       130
#endif
#define MAX_WINDOW_DRIFT_RT              (MAX_WINDOW_DRIFT_US * RT_PER_US)
#define MAX_RADIO_STARTUP_DELAY_RT       (MAX_RADIO_STARTUP_DELAY_US * RT_PER_US)

//--------------------------------------------
static msg_app2net_t msg_app2net;
static msg_net2app_t msg_net2app;

//--------------------------------------------
typedef enum
{
	START_IMMEDIATE_STOP_NO,
	START_IMMEDIATE_STOP_DELAYED,
	START_DELAYED_STOP_DELAYED,
} radio_start_mode_t;

//--------------------------------------------
static ble_conf_t conf =
{
	.adv.params.phy = BLE_PHY_1M,
	.adv.params.channel = 37,
	.adv.params.address = ADV_CHANNEL_ACCESS_ADDRESS,
	.adv.params.crc_init = ADV_CHANNEL_CRC_INIT,
	.aux.params.address = ADV_CHANNEL_ACCESS_ADDRESS,
	.aux.params.crc_init = ADV_CHANNEL_CRC_INIT,
};
//--------------------------------------------
static ble_chan_type_t ble_chan_type = BLE_CHAN_ADV;
static radio_rx_item_t radio_rx_item;
static radio_start_mode_t start_mode = START_IMMEDIATE_STOP_NO;
static ble_chan_type_t ble_channel_type_after_packet_processing;
static bool ignore_aux_packet_not_received;
static bool sync_aux_packet;
static bool biginfo_already_proccessing;
static bool decrypt_packet;
static bool radio_started;

//--------------------------------------------
#define FOLLOW_CONN   (1 << 0)    // connection
#define FOLLOW_PA     (1 << 1)    // periodic advertising
#define FOLLOW_CIS    (1 << 2)    // connected isochronous stream
#define FOLLOW_BIS    (1 << 3)    // broadcast isochronous stream

//--------------------------------------------
typedef struct
{
	uint8_t follow_filter;
	bool mac_filter;
	uint8_t mac_addr[DEVICE_ADDRESS_LENGTH];
	uint8_t hop_map[3];
	uint8_t hop_map_size;
	uint8_t hop_map_count;
	bool rssi_filter;
	int8_t rssi;
	bool look_for_connect;
	bool do_not_follow;
} packet_processing_filter_t;

//--------------------------------------------
static packet_processing_filter_t filter;

//--------------------------------------------
static void rx_next_aux_sync_packet(bool packet_received, uint32_t timestamp);
static void next_delayed_start_packet_received_and_processed(void);
static void mem_callback(void);
static void radio_start(void);
static void radio_stop(void);


//--------------- Debugging ------------------
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#ifndef DEBUG_RADIO_TIMING
#define DEBUG_RADIO_TIMING         0
#endif
#ifndef DEBUG_APP2NET
#define DEBUG_APP2NET              0
#endif
#ifndef DEBUG_BREAK
#define DEBUG_BREAK                0
#endif
#ifndef DEBUG_STOP
#define DEBUG_STOP                 0
#endif
//--------------------------------------------
#if DEBUG_RADIO_TIMING
//--------------------------------------------
typedef enum
{
	func_radio_irq_callback,
	func_radio_start,
	func_next_delayed_start,
	func_acl_packet_not_received,
	func_aux_packet_not_received,
	func_adv_packet_not_received,
	func_immediate_start_t_ifs_stop,
	func_adv_packet_received,
	func_rx_next_iso_cig_packet,
	func_rx_next_iso_big_packet,
	func_rx_next_aux_sync_packet,
	func_rx_next_pawr_packet
} start_func_t;
//--------------------------------------------
typedef struct
{
	// start radio rx
	start_func_t from_function;
	ble_chan_type_t chan_type;
	uint8_t chan;
	bool immediate;
	uint32_t want_now;
	uint32_t want_to_start;
	uint32_t want_to_stop;
	uint16_t event_counter;
	uint8_t subevent_counter;
	// DISABLED radio state irq
	start_func_t irq_function;
	bool really_rx;
	uint32_t really_now;
	uint32_t really_start;
	uint32_t really_stop;
	uint32_t timestamp;
} radio_debug_t;
//--------------------------------------------
#define RADIO_ARRAY_SIZE 20
static volatile radio_debug_t radio_debug[RADIO_ARRAY_SIZE];
static size_t radio_debug_cnt = 0;
//--------------------------------------------
static void radio_debug_add_start(bool immediate, uint32_t want_to_start, uint32_t want_to_stop, ble_chan_type_t chan_type, start_func_t from_function)
{
	uint32_t now = radio_tmr_get_radio_time();
	radio_debug[radio_debug_cnt].immediate = immediate;
	radio_debug[radio_debug_cnt].want_now = now;
	if (immediate)
	{
		radio_debug[radio_debug_cnt].want_to_start = now;
	}
	else
	{
		radio_debug[radio_debug_cnt].want_to_start = want_to_start;
	}
	radio_debug[radio_debug_cnt].want_to_stop = want_to_stop;
	radio_debug[radio_debug_cnt].chan_type = chan_type;
	switch (chan_type)
	{
	case BLE_CHAN_ADV:
		radio_debug[radio_debug_cnt].chan = conf.adv.params.channel;
		break;
	case BLE_CHAN_AUX:
	case BLE_CHAN_PAWR:
		radio_debug[radio_debug_cnt].chan = conf.aux.params.channel;
		radio_debug[radio_debug_cnt].event_counter = conf.aux.event_counter;
		break;
	case BLE_CHAN_ACL:
		radio_debug[radio_debug_cnt].chan = conf.acl.params.channel;
		radio_debug[radio_debug_cnt].event_counter = conf.acl.event_counter;
		radio_debug[radio_debug_cnt].subevent_counter = 0;
		break;
	case BLE_CHAN_ISO_CIG:
	case BLE_CHAN_ISO_BIG:
		radio_debug[radio_debug_cnt].chan = conf.iso.params.channel;
		radio_debug[radio_debug_cnt].event_counter = conf.iso.event_counter;
		radio_debug[radio_debug_cnt].subevent_counter = conf.iso.sub_event_counter;
		break;
	default:
		break;
	}
	radio_debug[radio_debug_cnt].from_function = from_function;

	radio_debug[radio_debug_cnt].really_rx = 0;
	radio_debug[radio_debug_cnt].really_now = 0;
	radio_debug[radio_debug_cnt].really_start = 0;
	radio_debug[radio_debug_cnt].really_stop = 0;
	radio_debug[radio_debug_cnt].timestamp = 0;
}
//--------------------------------------------
static void radio_debug_add_stop(bool really_rx)
{
	radio_debug[radio_debug_cnt].really_rx = really_rx;
	radio_debug[radio_debug_cnt].really_now = radio_tmr_get_radio_time();
	radio_debug[radio_debug_cnt].really_start = hal_rt_get_rxready_time();
	radio_debug[radio_debug_cnt].really_stop = hal_rt_get_disabled_time();

	if (!really_rx)
	{
		radio_debug[radio_debug_cnt].timestamp = 0;
		radio_debug_cnt++;
		if (radio_debug_cnt == RADIO_ARRAY_SIZE)
		{
			radio_debug_cnt = 0;
		}
	}
}
//--------------------------------------------
static void radio_debug_add_timestamp(uint32_t timestamp)
{
	radio_debug[radio_debug_cnt].timestamp = timestamp;
	radio_debug_cnt++;
	if (radio_debug_cnt == RADIO_ARRAY_SIZE)
	{
		radio_debug_cnt = 0;
	}
}
#endif
//--------------------------------------------
#if DEBUG_BREAK
#define __debugbreak() __asm("bkpt 0")
#else
//--------------------------------------------
#define __debugbreak()
#endif
//--------------------------------------------
#if DEBUG_APP2NET
//--------------------------------------------
typedef struct
{
	uint32_t timestamp;
	ipc_msg_type_t command;
} app2net_debug_t;
//--------------------------------------------
#define APP2NET_ARRAY_SIZE 10
static volatile app2net_debug_t app2net_debug[APP2NET_ARRAY_SIZE];
static size_t app2net_debug_cnt = 0;
//--------------------------------------------
static void app2net_debug_add(ipc_msg_type_t command)
{
	app2net_debug[app2net_debug_cnt].timestamp = radio_tmr_get_radio_time();
	app2net_debug[app2net_debug_cnt].command = command;
	app2net_debug_cnt++;
	if (app2net_debug_cnt == APP2NET_ARRAY_SIZE)
	{
		app2net_debug_cnt = 0;
	}
}
#endif
#if DEBUG_STOP
static bool stop;
#endif
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//--------------------------------------------
static radio_params_t *rx_get_params(ble_chan_type_t channel_type)
{
	switch (channel_type)
	{
	case BLE_CHAN_ADV:
		return &conf.adv.params;
	case BLE_CHAN_AUX:
	case BLE_CHAN_AUX_SYNC:
	case BLE_CHAN_PAWR:
		return &conf.aux.params;
	case BLE_CHAN_ACL:
		return &conf.acl.params;
	case BLE_CHAN_ISO_CIG:
	case BLE_CHAN_ISO_BIG:
		return &conf.iso.params;
	default:
		break;
	}
	return NULL;
}

//--------------------------------------------
static ble_phy_t get_current_phy(void)
{
	switch (ble_chan_type)
	{
	case BLE_CHAN_ADV:
		return conf.adv.params.phy;
	case BLE_CHAN_AUX:
	case BLE_CHAN_AUX_SYNC:
	case BLE_CHAN_PAWR:
		return conf.aux.params.phy;
	case BLE_CHAN_ACL:
		return conf.acl.params.phy;
	case BLE_CHAN_ISO_CIG:
	case BLE_CHAN_ISO_BIG:
		return conf.iso.params.phy;
	default:
		break;
	}
	return BLE_PHY_1M;
}

//--------------------------------------------
static void set_radio_start_time(uint32_t time)
{
	hal_rt_set_compare_time(time, RADIO_RXEN_COMPARE_CH);
}

//--------------------------------------------
static void set_radio_stop_time(uint32_t time)
{
	hal_rt_set_compare_time(time, RADIO_DISABLE_COMPARE_CH);
}

//--------------------------------------------
static void set_next_acl_channel(void)
{
	if (conf.acl.use_csa2)
	{
		conf.acl.params.channel = csa2_acl_chan(&conf);
	}
	else
	{
		conf.acl.params.channel = csa1_acl_chan(&conf);
	}
}

//--------------------------------------------
static uint32_t get_sleep_clock_accuracy(uint8_t sca)
{
	// 2.3.3.1 CONNECT_IND and AUX_CONNECT_REQ
	// The value of the SCA field shall be set as defined in Table 2.13
	const uint32_t sca_ppm[] = { 500, 250, 150, 100, 75, 50, 30, 20 };
	if (sca < sizeof(sca_ppm))
	{
		return sca_ppm[sca];
	}
	return sca_ppm[0];
}

//--------------------------------------------
static void set_sleep_clock_accuracy_from_connection_packet(uint8_t sca)
{
	conf.sca_c = get_sleep_clock_accuracy(sca);
	// 4.2.2 Sleep clock accuracy
	// If the Link Layer has not initiated or responded to the Sleep Clock Accuracy Update procedure
	// (see Section 5.1.14) in the current connection, the Central shall use a sleep clock
	// accuracy that is better than or equal to the worst case indicated in the SCA field of the
	// CONNECT_IND or AUX_CONNECT_REQ PDU used to create the connection and the
	// Peripheral shall use a sleep clock accuracy of ±500 ppm or better.
	conf.sca_p = 500;
}

static void set_sleep_clock_accuracy_from_sync_info(uint8_t sca)
{
	conf.aux.sync.sca = get_sleep_clock_accuracy(sca);
	// 4.2.2 Sleep clock accuracy
	// If the Link Layer has not initiated or responded to the Sleep Clock Accuracy Update procedure
	// (see Section 5.1.14) in the current connection, the Central shall use a sleep clock
	// accuracy that is better than or equal to the worst case indicated in the SCA field of the
	// CONNECT_IND or AUX_CONNECT_REQ PDU used to create the connection and the
	// Peripheral shall use a sleep clock accuracy of ±500 ppm or better.
	conf.sca_p = 500;
}

//--------------------------------------------
static void calculate_max_acl_window_drift(void)
{
	conf.acl.max_window_drift_rt = conf.acl.time_cfg.interval_rt * (conf.sca_c + conf.sca_p) / 1000000;
	if (conf.acl.max_window_drift_rt < MAX_WINDOW_DRIFT_RT)
	{
		conf.acl.max_window_drift_rt = MAX_WINDOW_DRIFT_RT;
	}
}

//--------------------------------------------
static void calculate_max_aux_sync_window_drift(void)
{
	conf.aux.sync.max_window_drift_rt = conf.aux.interval_rt * (conf.aux.sync.sca + conf.sca_p /*500*/) / 1000000;
	if (conf.aux.sync.max_window_drift_rt < MAX_WINDOW_DRIFT_RT)
	{
		conf.aux.sync.max_window_drift_rt = MAX_WINDOW_DRIFT_RT;
	}
}

//--------------------------------------------
static uint32_t calculate_max_aux_sync_window_offset_drift(void)
{
	uint32_t drift = conf.aux.sync_offset_rt * (conf.aux.sync.sca + 500) / 1000000;
	return (drift > MAX_WINDOW_DRIFT_RT) ? drift : MAX_WINDOW_DRIFT_RT;
}

//--------------------------------------------
static void connection_packet_processing(radio_packet_t *packet)
{
	conf.acl.params.address = *(uint32_t *)(packet->data + CONNECT_REQ_LL_DATA);
	conf.acl.params.crc_init = *(uint32_t *)(packet->data + CONNECT_REQ_LL_DATA + ACCESS_ADDRESS_LENGTH) & 0xFFFFFF;
	conf.acl.time_cfg.window_size_rt = *(packet->data + CONNECT_REQ_LL_DATA + ACCESS_ADDRESS_LENGTH + 3) * CONNECT_REQ_TIME_UNIT * RT_PER_US;
	conf.acl.time_cfg.window_offset_rt = (uint32_t)(*(uint16_t *)(packet->data + CONNECT_REQ_LL_DATA + ACCESS_ADDRESS_LENGTH + 4)) * CONNECT_REQ_TIME_UNIT * RT_PER_US;
	conf.acl.time_cfg.interval_rt = (uint32_t)(*(uint16_t *)(packet->data + CONNECT_REQ_LL_DATA + ACCESS_ADDRESS_LENGTH + 6)) * CONNECT_REQ_TIME_UNIT * RT_PER_US;
	conf.acl.time_cfg.latency = *(uint16_t *)(packet->data + CONNECT_REQ_LL_DATA + ACCESS_ADDRESS_LENGTH + 8);
	conf.acl.time_cfg.timeout_rt = (uint32_t)(*(uint16_t *)(packet->data + CONNECT_REQ_LL_DATA + ACCESS_ADDRESS_LENGTH + 10)) * 10000 * RT_PER_US;
	memcpy(conf.channel_map, packet->data + CONNECT_REQ_LL_DATA + ACCESS_ADDRESS_LENGTH + 12, DATA_CHANNELS_BYTES_NUMBER);
	set_sleep_clock_accuracy_from_connection_packet(*(packet->data + CONNECT_REQ_LL_DATA + ACCESS_ADDRESS_LENGTH + 17) >> 5);
	calculate_max_acl_window_drift();
	conf.channel_count = count_used_channels(conf.channel_map);
	if (conf.acl.use_csa2)
	{
		conf.acl.csa2.channel_identifier = (conf.acl.params.address >> 16) ^ (conf.acl.params.address & 0xFFFF);
	}
	else
	{
		conf.acl.csa1.hop = packet->data[CONNECT_REQ_LL_DATA + ACCESS_ADDRESS_LENGTH + 12 + DATA_CHANNELS_BYTES_NUMBER] & HOP_MASK;
		conf.acl.csa1.unmapped_channel = 0;
	}
	conf.acl.packet_direction = 0;
	conf.acl.event_counter = 0;
	conf.acl.anchor_packet = true;
	conf.acl.first_packet = true;
	conf.acl.connection_terminated = false;
	conf.acl.conn_interval_counter = 1;
	set_next_acl_channel();

	start_mode = START_DELAYED_STOP_DELAYED;

	// set up first ACL radio reception
	radio_rx_item.chan_type = BLE_CHAN_ACL;
	// 4.5.3 Connection event transmit window
	// The transmit window starts at transmitWindowDelay + transmitWindowOffset
	// after the end of the packet containing the CONNECT_IND PDU or AUX_CONNECT_REQ PDU
	conf.acl.calc_radio_anchor_time = packet->timestamp                            // connection packet timestamp
		+ ble_get_packet_transmission_time_rt(conf.acl.params.phy, packet->size)   // + connection packet duration
		+ ble_get_transmit_window_delay_rt(conf.acl.params.phy, packet->channel)   // + transmitWindowDelay
		+ conf.acl.time_cfg.window_offset_rt;                                      // + transmitWindowOffset
	radio_rx_item.radio_start_time = conf.acl.calc_radio_anchor_time               // calculated new anchor point
		- MAX_WINDOW_DRIFT_RT                                                      // - maximum window drift
		- MAX_RADIO_STARTUP_DELAY_RT;                                              // - maximum time for radio startup
	radio_rx_item.radio_end_time = conf.acl.calc_radio_anchor_time                 // calculated new anchor point
		+ conf.acl.time_cfg.window_size_rt                                         // + transmitWindowSize
		+ ble_get_access_address_transmission_time_rt(conf.acl.params.phy)         // + time to receive access address
		+ MAX_WINDOW_DRIFT_RT;                                                     // + maximum window drift
	radio_rx_list_put(radio_rx_item, FROM_CONNECT_IND);
}

//--------------------------------------------
static void adv_packet_processing(radio_packet_t *packet)
{
	switch (packet->data[0] & PDU_TYPE_MASK)
	{
	case ADV_IND:
		adv_cache_store(packet->data + MINIMUM_HEADER_LENGTH, packet->data[0]);
		break;
	case ADV_DIRECT_IND:
		adv_cache_store(packet->data + MINIMUM_HEADER_LENGTH, packet->data[0]);
		break;
	case ADV_EXT_IND:
	{
		uint8_t extended_header_flags = packet->data[MINIMUM_HEADER_LENGTH + 1];
		if (!(extended_header_flags & EXTENDED_HEADER_ADVERTISING_DATA_INFO_Msk))
		{
			break;
		}
		uint8_t advertising_data_info_offset =
			(((extended_header_flags & EXTENDED_HEADER_ADVERTISING_ADDRESS_Msk) >> EXTENDED_HEADER_ADVERTISING_ADDRESS_Pos) * EXTENDED_HEADER_ADVERTISING_ADDRESS_Len) +
			(((extended_header_flags & EXTENDED_HEADER_TARGET_ADDRESS_Msk) >> EXTENDED_HEADER_TARGET_ADDRESS_Pos) * EXTENDED_HEADER_TARGET_ADDRESS_Len) +
			(((extended_header_flags & EXTENDED_HEADER_CTE_INFO_Msk) >> EXTENDED_HEADER_CTE_INFO_Pos) * EXTENDED_HEADER_CTE_INFO_Len) +
			(((extended_header_flags & EXTENDED_HEADER_ADVERTISING_DATA_INFO_Msk) >> EXTENDED_HEADER_ADVERTISING_DATA_INFO_Pos) * EXTENDED_HEADER_ADVERTISING_DATA_INFO_Len) +
			1;

		uint8_t advertising_aux_pointer_1 = packet->data[MINIMUM_HEADER_LENGTH + 1 + advertising_data_info_offset];
		uint8_t channel = advertising_aux_pointer_1 & 0x3f;
		uint32_t units = advertising_aux_pointer_1 & 0x80 ? 300 : 30;
		uint16_t advertising_aux_pointer_23 = *(uint16_t *)(packet->data + MINIMUM_HEADER_LENGTH + 1 + advertising_data_info_offset + 1);
		uint32_t aux_offset = advertising_aux_pointer_23 & 0x1fff;
		uint8_t phy = (advertising_aux_pointer_23 & 0xe000) >> 13;

		ble_value_to_phy(phy, &conf.aux.params.phy);
		conf.aux.params.channel = channel;
		conf.aux.aux_offset_rt = aux_offset * units * RT_PER_US;

		conf.aux.params.address = ADV_CHANNEL_ACCESS_ADDRESS;
		conf.aux.params.crc_init = ADV_CHANNEL_CRC_INIT;

		conf.aux.next_aux_type = PKT_AUX_ADV_IND;

		start_mode = START_DELAYED_STOP_DELAYED;

		// set up first AUX_ADV_IND radio reception
		radio_rx_item.chan_type = BLE_CHAN_AUX;
		// 2.3.4.5 AuxPtr field
		// The auxiliary packet shall not start any earlier than the Aux Offset
		// after the reference point and shall start no later than the Aux Offset
		// plus one Offset Unit after the reference point.
		uint32_t aux_window_start = packet->timestamp                            // ADV_EXT_IND packet timestamp
			+ conf.aux.aux_offset_rt;                                            // + AuxOffset
		radio_rx_item.radio_start_time = aux_window_start                        // auxiliary packet window start
			- MAX_WINDOW_DRIFT_RT                                                // - maximum window drift
			- MAX_RADIO_STARTUP_DELAY_RT;                                        // - maximum time for radio startup
		radio_rx_item.radio_end_time = aux_window_start                          // auxiliary packet window start
			+ units * RT_PER_US                                                  // + one Offset unit
			+ ble_get_access_address_transmission_time_rt(conf.acl.params.phy)   // + time to receive access address
			+ MAX_WINDOW_DRIFT_RT;                                               // + maximum window drift
		radio_rx_list_put(radio_rx_item, FROM_ADV_EXT_IND);
		break;
	}
	case CONNECT_IND:
		conf.acl.use_csa2 = packet->data[0] & CSA_MASK ? true : false;
		uint8_t adv_hdr = adv_cache_fetch(packet->data + MINIMUM_HEADER_LENGTH + DEVICE_ADDRESS_LENGTH);
		if (!(adv_hdr & CSA_MASK))
		{
			conf.acl.use_csa2 = false;
		}
		conf.acl.params.phy = (ble_phy_t)packet->phy;
		connection_packet_processing(packet);
		break;
	}
}

//--------------------------------------------
static void next_big_event_channel_numbers_calculating(void)
{
	size_t se_cnt;
	for (size_t e_cnt = 0; e_cnt < conf.iso.big.num_bis; e_cnt++)
	{
		bool bis_event = true;
		for (size_t s_cnt = 0; s_cnt < conf.iso.big.nse; s_cnt++)
		{
			if (conf.iso.big.interleaved_arrangement)
			{
				// Interleaved arrangement
				se_cnt = s_cnt * conf.iso.big.num_bis + e_cnt;
			}
			else
			{
				// Sequential arrangement
				se_cnt = e_cnt * conf.iso.big.nse + s_cnt;
			}
			if (bis_event)
			{
				conf.iso.big.params[se_cnt].channel = lll_chan_iso_event(
					conf.iso.event_counter,
					conf.iso.big.channel_identifier[se_cnt],
					conf.iso.big.channel_map,
					conf.iso.big.channel_count,
					&conf.iso.big.prn_s[e_cnt],
					&conf.iso.big.remap_idx[e_cnt]);
				bis_event = false;
			}
			else
			{
				conf.iso.big.params[se_cnt].channel = lll_chan_iso_subevent(
					conf.iso.big.channel_identifier[se_cnt],
					conf.iso.big.channel_map,
					conf.iso.big.channel_count,
					&conf.iso.big.prn_s[e_cnt],
					&conf.iso.big.remap_idx[e_cnt]);
			}
		}
	}
	// Control subevent
	se_cnt = conf.iso.big.num_bis * conf.iso.big.nse;
	conf.iso.big.params[se_cnt].channel = lll_chan_iso_event(
		conf.iso.event_counter,
		conf.iso.big.channel_identifier[se_cnt],
		conf.iso.big.channel_map,
		conf.iso.big.channel_count,
		&conf.iso.big.prn_s[conf.iso.big.num_bis],
		&conf.iso.big.remap_idx[conf.iso.big.num_bis]);
}

//--------------------------------------------
static void big_info_processing(radio_packet_t *packet, uint8_t acad_offset)
{
	uint8_t cnt = acad_offset;
	uint8_t len = packet->data[cnt];

	// set up next AUX_SYNC_IND radio reception
	radio_rx_item.chan_type = BLE_CHAN_AUX;
	uint32_t aux_sync_start = packet->timestamp                              // AUX_SYNC_IND with BIGInfo packet timestamp
		+ conf.aux.interval_rt;                                              // + Interval
	radio_rx_item.radio_start_time = aux_sync_start                          // auxiliary packet window start
		- conf.aux.sync.max_window_drift_rt                                  // - maximum window drift
		- MAX_RADIO_STARTUP_DELAY_RT;                                        // - maximum time for radio startup
	radio_rx_item.radio_end_time = aux_sync_start                            // auxiliary packet window start
		+ ble_get_access_address_transmission_time_rt(conf.aux.params.phy)   // + time to receive access address
		+ conf.aux.sync.max_window_drift_rt;                                 // + maximum window drift
	radio_rx_list_put(radio_rx_item, FROM_AUX_SYNC_IND);

	conf.aux.event_counter += 1;
	conf.aux.params.channel = csa2_aux_sync_chan(&conf);

	if (len != 34 && len != 58)
	{
		// Not BIGInfo or corrupted packet
		return;
	}

	if (biginfo_already_proccessing)
	{
		ignore_aux_packet_not_received = true;
		return;
	}
	biginfo_already_proccessing = true;

	// Unencrypted or Encrypted BIG
	cnt += 2;
	uint32_t big_offset = (uint32_t)(*(uint16_t *)(packet->data + cnt) & 0x3FFF);
	cnt += 1;
	uint32_t big_offset_units = packet->data[cnt] & 0x40 ? 300 : 30;
	conf.iso.big.big_offset_rt = big_offset * big_offset_units * RT_PER_US;
	conf.iso.iso_interval_rt = ((*(uint32_t *)(packet->data + cnt) & 0x7FF80) >> 7) * CONNECT_REQ_TIME_UNIT * RT_PER_US;
	cnt += 2;
	conf.iso.big.num_bis = packet->data[cnt] >> 3;
	cnt += 1;
	conf.iso.big.nse = packet->data[cnt] & 0x1F;
	conf.iso.num_sub_events = conf.iso.big.num_bis * conf.iso.big.nse;
	conf.iso.big.bn = packet->data[cnt] >> 5;
	cnt += 1;
	conf.iso.sub_interval_rt = (*(uint32_t *)(packet->data + cnt) & 0xFFFFF) * RT_PER_US;
	cnt += 3;
	conf.iso.big.bis_spacing_rt = (*(uint32_t *)(packet->data + cnt) & 0xFFFFF) * RT_PER_US;
	cnt += 5;
	conf.iso.big.seed_access_address = *(uint32_t *)(packet->data + cnt);
	cnt += 8;
	// 3.1.1 CRC generation
	// For every Broadcast Isochronous PDU, the shift register shall be preset with
	// the BaseCRCInit value from the BIGInfo data in the most significant 2 octets
	// and the BIS_Number for the specific BIS in the least significant octet.
	// For BIG Control PDUs, the least significant octet shall be 0.
	conf.iso.big.base_crc_init = (uint32_t)(*(uint16_t *)(packet->data + cnt)) << 8;
	cnt += 2;
	memcpy(conf.iso.big.channel_map, packet->data + cnt, 5);
	conf.iso.big.channel_map[4] &= ~0xE0;
	conf.iso.big.channel_count = count_used_channels(conf.iso.big.channel_map);
	cnt += 4;
	uint8_t phy = packet->data[cnt] >> 5;
	ble_value_to_phy(phy, &conf.iso.params.phy);
	cnt += 1;
	// conf.iso.big.bis_payload_count = *(uint64_t *)(packet->data + cnt) & 0x1FFFFFFFFF;
	// cannot be used because the 64-bit value being read is not aligned to an 8-byte boundary,
	// which will cause a hard fault on the LDRD assembler instruction.
	// use memcpy instead
	memcpy(&conf.iso.big.bis_payload_count, packet->data + cnt, sizeof(uint64_t));
	conf.iso.big.bis_payload_count &= 0x1FFFFFFFFF;
	if (len == 58)
	{
		// Encrypted BIG
		// not yet supported
		__debugbreak();
		return;
	}

	// Sequential or interleaved BISes arrangement
	// 4.4.6.4 BIG event
	if (conf.iso.sub_interval_rt * conf.iso.big.nse <= conf.iso.big.bis_spacing_rt)
	{
		conf.iso.big.interleaved_arrangement = false;
	}
	else
	{
		conf.iso.big.interleaved_arrangement = true;
	}

	if (conf.iso.big.num_bis * conf.iso.big.nse >= MAX_BIS_SUBEVENTS_IN_BIG_EVENT)
	{
		// Limit the total number of BIS subevents to MAX_BIS_SUBEVENTS_IN_BIG_EVENT
		// not supported
		__debugbreak();
		return;
	}

	// Create radio parameters (except channel number) of all BIS subevents in advance
	size_t se_cnt;
	for (size_t e_cnt = 0; e_cnt < conf.iso.big.num_bis; e_cnt++)
	{
		for (size_t s_cnt = 0; s_cnt < conf.iso.big.nse; s_cnt++)
		{
			if (conf.iso.big.interleaved_arrangement)
			{
				// Interleaved arrangement
				se_cnt = s_cnt * conf.iso.big.num_bis + e_cnt;
			}
			else
			{
				// Sequential arrangement
				se_cnt = e_cnt * conf.iso.big.nse + s_cnt;
			}
			conf.iso.big.params[se_cnt].phy = conf.iso.params.phy;
			conf.iso.big.params[se_cnt].address = ble_bis_aa_from_seed_aa_and_bis_number(conf.iso.big.seed_access_address, e_cnt + 1);
			conf.iso.big.params[se_cnt].crc_init = conf.iso.big.base_crc_init | (e_cnt + 1);
			conf.iso.big.channel_identifier[se_cnt] = (conf.iso.big.params[se_cnt].address >> 16) ^ (conf.iso.big.params[se_cnt].address & 0xFFFF);
			conf.iso.big.stop_time_offset[se_cnt] = e_cnt * conf.iso.big.bis_spacing_rt + s_cnt * conf.iso.sub_interval_rt;
		}
	}
	// Control subevent
	se_cnt = conf.iso.big.num_bis * conf.iso.big.nse;
	conf.iso.big.params[se_cnt].phy = conf.iso.params.phy;
	conf.iso.big.params[se_cnt].address = ble_bis_aa_from_seed_aa_and_bis_number(conf.iso.big.seed_access_address, 0);
	conf.iso.big.params[se_cnt].crc_init = conf.iso.big.base_crc_init | 0;
	conf.iso.big.channel_identifier[se_cnt] = (conf.iso.big.params[se_cnt].address >> 16) ^ (conf.iso.big.params[se_cnt].address & 0xFFFF);
	if (conf.iso.big.interleaved_arrangement)
	{
		// Interleaved arrangement
		conf.iso.big.stop_time_offset[se_cnt] = conf.iso.big.num_bis * conf.iso.sub_interval_rt;
	}
	else
	{
		// Sequential arrangement
		conf.iso.big.stop_time_offset[se_cnt] = conf.iso.big.num_bis * conf.iso.big.bis_spacing_rt;
	}

	// set the event counter and calculate the channel numbers
	conf.iso.event_counter = (conf.iso.big.bis_payload_count / conf.iso.big.bn) & 0xFFFF;
	next_big_event_channel_numbers_calculating();

	conf.iso.sub_event_counter = 0;
	conf.iso.params = conf.iso.big.params[conf.iso.sub_event_counter];

	ignore_aux_packet_not_received = true;
	start_mode = START_DELAYED_STOP_DELAYED;
	// set up first ISO BIS radio reception
	radio_rx_item.chan_type = BLE_CHAN_ISO_BIG;
	conf.iso.calc_radio_anchor_time = packet->timestamp                // AUX_SYNC_IND with BIGInfo packet timestamp
		+ conf.iso.big.big_offset_rt;                                  // + BIG_Offset
	radio_rx_item.radio_start_time = conf.iso.calc_radio_anchor_time   // calculated new anchor point
		- MAX_WINDOW_DRIFT_RT                                          // - maximum window drift
		- MAX_RADIO_STARTUP_DELAY_RT;                                  // - maximum time for radio startup
	radio_rx_item.radio_end_time = conf.iso.calc_radio_anchor_time     // calculated new anchor point
		+ conf.iso.sub_interval_rt                                     // + sub_interval
		+ MAX_WINDOW_DRIFT_RT;                                         // + maximum window drift
	radio_rx_list_put(radio_rx_item, FROM_AUX_SYNC_IND);

	conf.iso.radio_anchor_time = conf.iso.calc_radio_anchor_time;
}

//--------------------------------------------
static void sync_info_parsing(uint8_t *data)
{
	uint16_t sync_info_01 = *(uint16_t *)(data);
	uint32_t sync_offset = sync_info_01 & 0x1FFF;
	conf.aux.sync_offset_unit = sync_info_01 & 0x2000 ? 300 : 30;
	uint32_t offset_adjust = sync_info_01 & 0x4000 ? 2457600 : 0;
	// 2.3.4.6 SyncInfo field
	// The value of syncPacketWindowOffset is determined by multiplying the value
	// of the Offset Base field by the unit of time indicated by the Offset Units field
	// and then, if the Offset Adjust field is set to 1, adding 2.4576 seconds.
	conf.aux.sync_offset_rt = (sync_offset * conf.aux.sync_offset_unit + offset_adjust) * RT_PER_US;

	uint16_t sync_info_23 = *(uint16_t *)(data + 2);
	conf.aux.interval_rt = sync_info_23 * CONNECT_REQ_TIME_UNIT; 

	memcpy(conf.aux.sync.channel_map, data + 4, DATA_CHANNELS_BYTES_NUMBER);
	conf.aux.sync.channel_map[DATA_CHANNELS_BYTES_NUMBER - 1] &= ~0xE0;
	conf.aux.sync.channel_count = count_used_channels(conf.aux.sync.channel_map);

	set_sleep_clock_accuracy_from_sync_info((data[8] & 0xE0) >> 5);
	calculate_max_aux_sync_window_drift();

	conf.aux.params.address = *(uint32_t *)(data + 4 + DATA_CHANNELS_BYTES_NUMBER);
	conf.aux.pawr.ind_address = conf.aux.params.address;
	conf.aux.channel_identifier = (conf.aux.params.address >> 16) ^ (conf.aux.params.address & 0xFFFF);
	conf.aux.params.crc_init = *(uint32_t *)(data + 4 + DATA_CHANNELS_BYTES_NUMBER + ACCESS_ADDRESS_LENGTH) & 0xFFFFFF;
	conf.aux.event_counter = *(uint16_t *)(data + 4 + DATA_CHANNELS_BYTES_NUMBER + ACCESS_ADDRESS_LENGTH + 3);

	conf.aux.params.channel = csa2_aux_sync_chan(&conf);
}

//--------------------------------------------
static void pawr_info_parsing(uint8_t *data)
{
	conf.aux.pawr.rsp_address = *(uint32_t *)(data);
	conf.aux.pawr.num_subevents = data[4];
	conf.aux.pawr.subevent_interval_rt = (uint32_t)(data[5]) * 1250;
	conf.aux.pawr.response_slot_delay_rt = (uint32_t)(data[6]) * 1250;
	conf.aux.pawr.response_slot_spacing_rt = (uint32_t)(data[7]) * 125;
	conf.aux.pawr.subevent_counter = 0;
	uint8_t response_slots_number = (conf.aux.pawr.subevent_interval_rt - conf.aux.pawr.response_slot_delay_rt) / conf.aux.pawr.response_slot_spacing_rt;
	conf.aux.pawr.response_slots_spacing_rt = conf.aux.pawr.response_slot_spacing_rt * response_slots_number;
	conf.aux.pawr.channel_identifier = (conf.aux.pawr.rsp_address >> 16) ^ (conf.aux.pawr.rsp_address & 0xFFFF);
}

//--------------------------------------------
static void aux_packet_processing(radio_packet_t *packet)
{
	switch (packet->data[0] & PDU_TYPE_MASK)
	{
	case AUX_ADV_IND:
	// AUX_ADV_IND, AUX_SCAN_RSP, AUX_SYNC_IND, AUX_CHAIN_IND, AUX_SYNC_SUBEVENT_IND, AUX_SYNC_SUBEVENT_RSP
	{
		conf.aux.aux_type = conf.aux.next_aux_type;

		uint8_t extended_header_length = packet->data[MINIMUM_HEADER_LENGTH] & 0x3F;
		// 2.3.4 Common Extended Advertising Payload Format
		// The Extended Header field is a variable length header that is present if,
		// and only if, the Extended Header Length field is non-zero.
		if (!extended_header_length)
		{
			// No data for controller, only Host Advertising Data possible.
			break;
		}

		uint8_t extended_header_flags = packet->data[MINIMUM_HEADER_LENGTH + 1];
		uint8_t acad_offset =
			(((extended_header_flags & EXTENDED_HEADER_ADVERTISING_ADDRESS_Msk) >> EXTENDED_HEADER_ADVERTISING_ADDRESS_Pos) * EXTENDED_HEADER_ADVERTISING_ADDRESS_Len) +
			(((extended_header_flags & EXTENDED_HEADER_TARGET_ADDRESS_Msk) >> EXTENDED_HEADER_TARGET_ADDRESS_Pos) * EXTENDED_HEADER_TARGET_ADDRESS_Len) +
			(((extended_header_flags & EXTENDED_HEADER_CTE_INFO_Msk) >> EXTENDED_HEADER_CTE_INFO_Pos) * EXTENDED_HEADER_CTE_INFO_Len) +
			(((extended_header_flags & EXTENDED_HEADER_ADVERTISING_DATA_INFO_Msk) >> EXTENDED_HEADER_ADVERTISING_DATA_INFO_Pos) * EXTENDED_HEADER_ADVERTISING_DATA_INFO_Len) +
			(((extended_header_flags & EXTENDED_HEADER_AUX_POINTER_Msk) >> EXTENDED_HEADER_AUX_POINTER_Pos) * EXTENDED_HEADER_AUX_POINTER_Len) +
			(((extended_header_flags & EXTENDED_HEADER_SYNC_INFO_Msk) >> EXTENDED_HEADER_SYNC_INFO_Pos) * EXTENDED_HEADER_SYNC_INFO_Len) +
			(((extended_header_flags & EXTENDED_HEADER_TX_POWER_Msk) >> EXTENDED_HEADER_TX_POWER_Pos) * EXTENDED_HEADER_TX_POWER_Len) +
			1;

		if (acad_offset < extended_header_length)
		{
			// 2.3.4.8 ACAD field
			// The remainder of the extended header forms the Additional Controller Advertising Data
			// (ACAD) field. The length of this field is the Extended Header length minus the sum of
			// the size of the extended header flags (1 octet) and those fields indicated by the flags
			// as present.
			uint8_t acad_type = packet->data[MINIMUM_HEADER_LENGTH + 1 + acad_offset + 1];
			if (acad_type == 0x2C && (filter.follow_filter & FOLLOW_BIS))
			{
				// BIGInfo
				biginfo_already_proccessing = false;
				big_info_processing(packet, MINIMUM_HEADER_LENGTH + 1 + acad_offset);
				conf.aux.next_aux_type = PKT_AUX_SYNC_IND;
				break;
			}
			if (acad_type == 0x32)
			{
				// Periodic Advertising Response Timing Information
				pawr_info_parsing(packet->data + MINIMUM_HEADER_LENGTH + 1 + acad_offset + 2);
				conf.aux.next_aux_type = PKT_AUX_SYNC_SUBEVENT_IND;
			}
		}

		if (sync_aux_packet && conf.aux.next_aux_type == PKT_AUX_SYNC_IND)
		{
			// First AUX_SYNC_IND
			// Receive all next AUX_SYNC_IND
			rx_next_aux_sync_packet(true, packet->timestamp);
			conf.aux.aux_type = PKT_AUX_SYNC_IND;
			next_delayed_start_packet_received_and_processed();
			break;
		}

		if (!(extended_header_flags & EXTENDED_HEADER_SYNC_INFO_Msk))
		{
			// No SyncInfo field
			conf.aux.next_aux_type = PKT_AUX_CHAIN_IND;  // ??
			break;
		}

		if (!(filter.follow_filter & FOLLOW_PA) && conf.aux.aux_type == PKT_AUX_ADV_IND)
		{
			// Don't follow AUX_SYNC_IND or AUX_SYNC_SUBEVENT_IND
			break;
		}

		uint8_t sync_info_offset =
			(((extended_header_flags & EXTENDED_HEADER_ADVERTISING_ADDRESS_Msk) >> EXTENDED_HEADER_ADVERTISING_ADDRESS_Pos) * EXTENDED_HEADER_ADVERTISING_ADDRESS_Len) +
			(((extended_header_flags & EXTENDED_HEADER_TARGET_ADDRESS_Msk) >> EXTENDED_HEADER_TARGET_ADDRESS_Pos) * EXTENDED_HEADER_TARGET_ADDRESS_Len) +
			(((extended_header_flags & EXTENDED_HEADER_CTE_INFO_Msk) >> EXTENDED_HEADER_CTE_INFO_Pos) * EXTENDED_HEADER_CTE_INFO_Len) +
			(((extended_header_flags & EXTENDED_HEADER_ADVERTISING_DATA_INFO_Msk) >> EXTENDED_HEADER_ADVERTISING_DATA_INFO_Pos) * EXTENDED_HEADER_ADVERTISING_DATA_INFO_Len) +
			(((extended_header_flags & EXTENDED_HEADER_AUX_POINTER_Msk) >> EXTENDED_HEADER_AUX_POINTER_Pos) * EXTENDED_HEADER_AUX_POINTER_Len) +
			1;

		sync_info_parsing(packet->data + MINIMUM_HEADER_LENGTH + 1 + sync_info_offset);

		start_mode = START_DELAYED_STOP_DELAYED;
		ignore_aux_packet_not_received = true;
		if (conf.aux.next_aux_type == PKT_AUX_SYNC_SUBEVENT_IND)
		{
			if (sync_aux_packet)
			{
				// Receive all AUX_SYNC_SUBEVENT_IND and AUX_SYNC_SUBEVENT_RSP
				radio_rx_item.chan_type = BLE_CHAN_PAWR;
			}
			else
			{
				// Receive only first AUX_SYNC_SUBEVENT_IND
				radio_rx_item.chan_type = BLE_CHAN_AUX;
			}
			conf.aux.next_aux_type = PKT_AUX_SYNC_SUBEVENT_IND;
			conf.aux.pawr.calc_radio_sync_start_time = packet->timestamp + conf.aux.sync_offset_rt;
		}
		else
		{
			// Receive only first AUX_SYNC_IND
			radio_rx_item.chan_type = BLE_CHAN_AUX;
			conf.aux.next_aux_type = PKT_AUX_SYNC_IND;
		}
		// 2.3.4.6 SyncInfo field
		// The packet containing the AUX_SYNC_IND PDU or AUX_SYNC_SUBEVENT_IND PDU shall not start
		// any earlier than syncPacketWindowOffset after the reference point and shall start
		// no later than syncPacketWindowOffset plus one Offset unit after the reference point.
		uint32_t max_aux_sync_window_offset_drift_rt = calculate_max_aux_sync_window_offset_drift();
		conf.aux.sync.calc_radio_sync_start_time = packet->timestamp                // AUX_ADV_IND packet timestamp
			+ conf.aux.sync_offset_rt;                                              // + Sync Offset
		radio_rx_item.radio_start_time = conf.aux.sync.calc_radio_sync_start_time   // reference point
			- max_aux_sync_window_offset_drift_rt                                   // - maximum window drift
			- MAX_RADIO_STARTUP_DELAY_RT;                                           // - maximum time for radio startup
		radio_rx_item.radio_end_time = conf.aux.sync.calc_radio_sync_start_time     // reference point
			+ conf.aux.sync_offset_unit * RT_PER_US                                 // + one Offset unit after the reference point
			+ max_aux_sync_window_offset_drift_rt;                                  // + maximum window drift
		radio_rx_list_put(radio_rx_item, FROM_AUX_ADV_IND);
		break;
	}
	case AUX_CONNECT_REQ:
		if (filter.follow_filter & FOLLOW_CONN)
		{
			conf.acl.use_csa2 = true;
			conf.acl.params.phy = conf.aux.params.phy;
			connection_packet_processing(packet);
		}
		break;
	case AUX_CONNECT_RSP:
		break;
	}
}

//--------------------------------------------
static void acl_packet_processing_anchor(radio_packet_t *packet)
{
	if (conf.acl.anchor_packet)
	{
		if (conf.acl.first_packet)
		{
			conf.acl.radio_anchor_time = packet->timestamp;
			conf.acl.first_packet = false;
		}
		else
		{
			uint32_t diff = packet->timestamp > conf.acl.calc_radio_anchor_time ?
				packet->timestamp - conf.acl.calc_radio_anchor_time :
				conf.acl.calc_radio_anchor_time -  packet->timestamp;
			#if DEBUG_RADIO_TIMING
			if (diff > conf.acl.max_window_drift_rt * conf.acl.conn_interval_counter)
			{
#if 0
				__debugbreak();
#endif
			}
			#endif
			conf.acl.radio_anchor_time = diff < conf.acl.max_window_drift_rt * conf.acl.conn_interval_counter ?
				packet->timestamp :
				conf.acl.calc_radio_anchor_time;
		}
		conf.acl.conn_interval_counter = 1;
		conf.acl.anchor_packet = false;
	}
}

//--------------------------------------------
static void acl_packet_processing(uint8_t *data, uint8_t direction, uint32_t timestamp)
{
	if ((data[0] & LLID_MASK) != LL_CONTROL_PDU)
	{
		return;
	}

	radio_conf_item_t radio_conf_item = { 0 };
	switch (data[MINIMUM_HEADER_LENGTH])
	{
	case LL_PHY_UPDATE_IND:
		// 5.5 Procedures with Instants
		// The instant field in the LL procedure PDU must always be greater than the current event counter.
		radio_conf_item.instant = *((uint16_t *)(data + MINIMUM_HEADER_LENGTH + 3));
		ble_bit_number_to_phy(data[MINIMUM_HEADER_LENGTH + 1], &radio_conf_item.phy);
		radio_conf_item.opcode = LL_PHY_UPDATE_IND;
		radio_conf_list_put(radio_conf_item);
		break;
	case LL_CONNECTION_UPDATE_IND:
		// 5.5 Procedures with Instants
		// The instant field in the LL procedure PDU must always be greater than the current event counter.
		radio_conf_item.instant = *((uint16_t *)(data + MINIMUM_HEADER_LENGTH + 10));
		radio_conf_item.time_cfg.window_size_rt = *(data + MINIMUM_HEADER_LENGTH + 1) * CONNECT_REQ_TIME_UNIT * RT_PER_US;
		radio_conf_item.time_cfg.window_offset_rt = (uint32_t)(*(uint16_t *)(data + MINIMUM_HEADER_LENGTH + 2)) * CONNECT_REQ_TIME_UNIT * RT_PER_US;
		radio_conf_item.time_cfg.interval_rt = (uint32_t)(*(uint16_t *)(data + MINIMUM_HEADER_LENGTH + 4)) * CONNECT_REQ_TIME_UNIT * RT_PER_US;
		radio_conf_item.time_cfg.latency = *(uint16_t *)(data + MINIMUM_HEADER_LENGTH + 6);
		radio_conf_item.time_cfg.timeout_rt = (uint32_t)(*(uint16_t *)(data + MINIMUM_HEADER_LENGTH + 8)) * 10000 * RT_PER_US;
		radio_conf_item.opcode = LL_CONNECTION_UPDATE_IND;
		radio_conf_list_put(radio_conf_item);
		break;
	case LL_CHANNEL_MAP_IND:
		// 5.5 Procedures with Instants
		// The instant field in the LL procedure PDU must always be greater than the current event counter.
		memcpy(radio_conf_item.channel_map, data + MINIMUM_HEADER_LENGTH + 1, DATA_CHANNELS_BYTES_NUMBER);
		radio_conf_item.instant = *((uint16_t *)(data + MINIMUM_HEADER_LENGTH + 6));
		radio_conf_item.opcode = LL_CHANNEL_MAP_IND;
		radio_conf_list_put(radio_conf_item);
		break;
	case LL_CIS_REQ:
		ble_bit_number_to_phy(data[MINIMUM_HEADER_LENGTH + 3], &conf.iso.params.phy);
		conf.iso.num_sub_events = data[MINIMUM_HEADER_LENGTH + 19];
		conf.iso.sub_interval_rt = (*((uint32_t *)(data + MINIMUM_HEADER_LENGTH + 20)) & 0xfffff) * RT_PER_US;
		conf.iso.iso_interval_rt = *((uint16_t *)(data + MINIMUM_HEADER_LENGTH + 26)) * CONNECT_REQ_TIME_UNIT * RT_PER_US;
		break;
	case LL_CIS_IND:
		if (!(filter.follow_filter & FOLLOW_CIS))
		{
			break;
		}
		conf.iso.params.address = *(uint32_t *)(data + MINIMUM_HEADER_LENGTH + 1);
		conf.iso.params.crc_init = conf.acl.params.crc_init;
		conf.iso.channel_identifier = (conf.iso.params.address >> 16) ^ (conf.iso.params.address & 0xFFFF);
		conf.iso.cig.cis_offset_rt = ((*(uint32_t *)(data + MINIMUM_HEADER_LENGTH + 5)) & 0xffffff) * RT_PER_US;
		conf.iso.cig.cig_sync_delay_rt = ((*(uint32_t *)(data + MINIMUM_HEADER_LENGTH + 8)) & 0xffffff) * RT_PER_US;
		conf.iso.cig.cis_sync_delay_rt = ((*(uint32_t *)(data + MINIMUM_HEADER_LENGTH + 11)) & 0xffffff) * RT_PER_US;
		// In fact, this field in the PDU is called connEventCount
		// and does not follow the rules specified in 5.5 Procedures with Instants
		radio_conf_item.instant = *((uint16_t *)(data + MINIMUM_HEADER_LENGTH + 14));

		// reset the event counter and calculate the channel number
		conf.iso.packet_direction = 0;
		conf.iso.event_counter = 0;
		conf.iso.cig.cis_event_counter = 0;
		conf.iso.anchor_packet = true;
		conf.iso.first_packet = true;
		conf.iso.params.channel = csa2_iso_event_chan(&conf);

		if (radio_conf_item.instant == conf.acl.event_counter)
		{
			// set up first ISO radio reception
			radio_rx_item.chan_type = BLE_CHAN_ISO_CIG;
			// 2.4.2.31 LL_CIS_IND
			// CIS_Offset shall be set to the time, in microseconds, from the ACL anchor point of the
			// connection event that is referenced by connEventCount to the first CIS anchor point.
			conf.iso.calc_radio_anchor_time = timestamp                        // LL_CIS_IND packet timestamp
				+ conf.iso.cig.cis_offset_rt;                                  // + CIS_Offset
			radio_rx_item.radio_start_time = conf.iso.calc_radio_anchor_time   // calculated new anchor point
				- MAX_WINDOW_DRIFT_RT                                          // - maximum window drift
				- MAX_RADIO_STARTUP_DELAY_RT;                                  // - maximum time for radio startup
			radio_rx_item.radio_end_time = conf.iso.calc_radio_anchor_time     // calculated new anchor point
				+ conf.iso.sub_interval_rt                                     // + sub_interval
				+ MAX_WINDOW_DRIFT_RT;                                         // + maximum window drift
			radio_rx_list_put(radio_rx_item, FROM_LL_CIS_IND);
		}
		else
		{
			// not yet implemented
#if 1
			__debugbreak();
#endif
		}
		break;
	case LL_ENC_REQ:
		crypto_set_skd_iv_c(data + 13);
		break;
	case LL_ENC_RSP:
		crypto_set_skd_iv_p(data + 3);
		crypto_generate_session_key();
		break;
	case LL_START_ENC_REQ:
		crypto_reset_encrypted_packet_counters();
		conf.acl.encrypted_packet = true;
		break;
	case LL_PAUSE_ENC_RSP:
		conf.acl.encrypted_packet = false;
		break;
	case LL_CLOCK_ACCURACY_REQ:
	case LL_CLOCK_ACCURACY_RSP:
		if (direction == 0)
		{
			conf.sca_c = get_sleep_clock_accuracy(data[2]);
		}
		else
		{
			conf.sca_p = get_sleep_clock_accuracy(data[2]);
		}
		calculate_max_acl_window_drift();
		break;
	case LL_SUBRATE_IND:
		// No additional implementation is required, since it is implemented by the sniffer's operating algorithm itself
		conf.acl.time_cfg.timeout_rt = (uint32_t)(*(uint16_t *)(data + MINIMUM_HEADER_LENGTH + 9)) * 10000 * RT_PER_US;
		break;
	case LL_TERMINATE_IND:
		conf.acl.connection_terminated = true;
		break;
	case LL_PERIODIC_SYNC_IND:
	{
		sync_info_parsing(data + MINIMUM_HEADER_LENGTH + 3);
		conf.aux.past.conn_event_count = *(uint16_t *)(data + MINIMUM_HEADER_LENGTH + 1 + 20);
		conf.aux.past.last_pa_event_counter = *(uint16_t *)(data + MINIMUM_HEADER_LENGTH + 1 + 22);
		conf.aux.past.sca = get_sleep_clock_accuracy(data[MINIMUM_HEADER_LENGTH + 1 + 24] >> 5);
		ble_bit_number_to_phy(data[MINIMUM_HEADER_LENGTH + 1 + 25] & 0x07, &conf.aux.params.phy);

		if (sync_aux_packet)
		{
			// Receive all AUX_SYNC_IND
			radio_rx_item.chan_type = BLE_CHAN_AUX_SYNC;
		}
		else
		{
			// Receive only first AUX_SYNC_IND
			radio_rx_item.chan_type = BLE_CHAN_AUX;
			ignore_aux_packet_not_received = true;
		}
		conf.aux.next_aux_type = PKT_AUX_SYNC_IND;

		uint16_t event_diff = conf.acl.event_counter > conf.aux.past.conn_event_count ?
			conf.acl.event_counter - conf.aux.past.conn_event_count :
			conf.aux.past.conn_event_count - conf.acl.event_counter;
		conf.aux.sync.calc_radio_sync_start_time = conf.acl.event_counter > conf.aux.past.conn_event_count ?
			timestamp - conf.acl.time_cfg.interval_rt * event_diff : timestamp + conf.acl.time_cfg.interval_rt * event_diff;
		// 2.3.4.6 SyncInfo field
		// The packet containing the AUX_SYNC_IND PDU or AUX_SYNC_SUBEVENT_IND PDU shall not start
		// any earlier than syncPacketWindowOffset after the reference point and shall start
		// no later than syncPacketWindowOffset plus one Offset unit after the reference point.
		uint32_t max_aux_sync_window_offset_drift_rt = calculate_max_aux_sync_window_offset_drift();
		conf.aux.sync.calc_radio_sync_start_time +=                                 // reference packet timestamp
			+ conf.aux.sync_offset_rt;                                              // + Sync Offset
		radio_rx_item.radio_start_time = conf.aux.sync.calc_radio_sync_start_time   // reference point
			- max_aux_sync_window_offset_drift_rt                                   // - maximum window drift
			- MAX_RADIO_STARTUP_DELAY_RT;                                           // - maximum time for radio startup
		radio_rx_item.radio_end_time = conf.aux.sync.calc_radio_sync_start_time     // reference point
			+ conf.aux.sync_offset_unit * RT_PER_US                                 // + one Offset unit after the reference point
			+ max_aux_sync_window_offset_drift_rt;                                  // + maximum window drift
		radio_rx_list_put(radio_rx_item, FROM_LL_PERIODIC_SYNC_IND);
		break;
	}
	case LL_PERIODIC_SYNC_WR_IND:
	{
		sync_info_parsing(data + MINIMUM_HEADER_LENGTH + 3);
		conf.aux.past.conn_event_count = *(uint16_t *)(data + MINIMUM_HEADER_LENGTH + 1 + 20);
		conf.aux.past.last_pa_event_counter = *(uint16_t *)(data + MINIMUM_HEADER_LENGTH + 1 + 22);
		conf.aux.past.sca = get_sleep_clock_accuracy(data[MINIMUM_HEADER_LENGTH + 1 + 24] >> 5);
		ble_bit_number_to_phy(data[MINIMUM_HEADER_LENGTH + 1 + 25] & 0x07, &conf.aux.params.phy);
		pawr_info_parsing(data + MINIMUM_HEADER_LENGTH + 1 + 34);

#if 1
		// Receive only first AUX_SYNC_SUBEVENT_IND
		radio_rx_item.chan_type = BLE_CHAN_AUX;
		ignore_aux_packet_not_received = true;
#else
		// Try to receive all PAwR packets, but:
		// Which subevent to listen to and in which response slot to respond is application specific,
		// so there is a high chance of packet overlap on PAWR and ACL channels.
		radio_rx_item.chan_type = BLE_CHAN_PAWR;
#endif
		conf.aux.next_aux_type = PKT_AUX_SYNC_SUBEVENT_IND;

		uint16_t event_diff = conf.acl.event_counter > conf.aux.past.conn_event_count ?
			conf.acl.event_counter - conf.aux.past.conn_event_count :
			conf.aux.past.conn_event_count - conf.acl.event_counter;
		conf.aux.pawr.calc_radio_sync_start_time = conf.acl.event_counter > conf.aux.past.conn_event_count ?
			timestamp - conf.acl.time_cfg.interval_rt * event_diff : timestamp + conf.acl.time_cfg.interval_rt * event_diff;
		// 2.3.4.6 SyncInfo field
		// The packet containing the AUX_SYNC_IND PDU or AUX_SYNC_SUBEVENT_IND PDU shall not start
		// any earlier than syncPacketWindowOffset after the reference point and shall start
		// no later than syncPacketWindowOffset plus one Offset unit after the reference point.
		uint32_t max_aux_sync_window_offset_drift_rt = calculate_max_aux_sync_window_offset_drift();
		conf.aux.pawr.calc_radio_sync_start_time +=                                 // reference packet timestamp
			+ conf.aux.sync_offset_rt;                                              // + Sync Offset
		radio_rx_item.radio_start_time = conf.aux.pawr.calc_radio_sync_start_time   // reference point
			- max_aux_sync_window_offset_drift_rt                                   // - maximum window drift
			- MAX_RADIO_STARTUP_DELAY_RT;                                           // - maximum time for radio startup
		radio_rx_item.radio_end_time = conf.aux.pawr.calc_radio_sync_start_time     // reference point
			+ conf.aux.sync_offset_unit * RT_PER_US                                 // + one Offset unit after the reference point
			+ max_aux_sync_window_offset_drift_rt;                                  // + maximum window drift
		radio_rx_list_put(radio_rx_item, FROM_LL_PERIODIC_SYNC_WR_IND);
		break;
	}
	}
}

//--------------------------------------------
void acl_packet_processing_from_queue(void)
{
	rx2decrypt_item_t *item;

	for (bool res = true; res;)
	{
		res = msg_queue_rx2decrypt_get_first_item_ptr_if_decrypted(&item);
		if (res)
		{
			acl_packet_processing(item->data, item->direction, item->timestamp);
			msg_queue_rx2decrypt_delete_first_item_if_decrypted();
		}
	}
	crypto_decrypt();
}

//--------------------------------------------
static void acl_packet_processing_encryption(radio_packet_t *packet)
{
	packet->enc_status = ENC_UNENCRYPTED;
	if (conf.acl.encrypted_packet)
	{
		packet->enc_status = ENC_ENCRYPTED;
		if (decrypt_packet)
		{
			uint8_t need_to_decrypt = packet->data[1] == 0 ? false : true;
			msg_queue_rx2decrypt_put_item(packet->data, need_to_decrypt, conf.acl.packet_direction, packet->timestamp);
			crypto_decrypt();
			acl_packet_processing_from_queue();
		}
	}
	else
	{
		acl_packet_processing(packet->data, conf.acl.packet_direction, packet->timestamp);
	}
	conf.acl.packet_direction ^= 1;
}

//--------------------------------------------
static void pawr_packet_processing(radio_packet_t *packet)
{
}

//--------------------------------------------
static void iso_cig_packet_processing(radio_packet_t *packet, bool anchor_packet)
{
	if (anchor_packet)
	{
		if (conf.iso.first_packet)
		{
			conf.iso.radio_anchor_time = packet->timestamp;
			conf.iso.first_packet = false;
		}
		else
		{
			uint32_t diff = packet->timestamp > conf.iso.calc_radio_anchor_time ?
				packet->timestamp - conf.iso.calc_radio_anchor_time :
				conf.iso.calc_radio_anchor_time - packet->timestamp;
			#if DEBUG_RADIO_TIMING
			if (diff > MAX_WINDOW_DRIFT_RT)
			{
				__debugbreak();
			}
			#endif
			conf.iso.radio_anchor_time = diff < MAX_WINDOW_DRIFT_RT ?
				packet->timestamp :
				conf.iso.calc_radio_anchor_time;
		}
		conf.iso.anchor_packet = false;
	}
#if 0
	uint32_t calc_radio_sub_start_time = conf.iso.radio_anchor_time + conf.iso.sub_event_counter * conf.iso.sub_interval_rt;
	uint32_t diff = packet->timestamp > calc_radio_sub_start_time ?
		packet->timestamp - calc_radio_sub_start_time :
		calc_radio_sub_start_time - packet->timestamp;
#endif
}

//--------------------------------------------
static void iso_big_packet_processing(radio_packet_t *packet)
{
}

//--------------------------------------------
static ble_chan_type_t next_delayed_start(void)
{
	if (!radio_rx_list_get(&radio_rx_item))
	{
#if 0
		__debugbreak();
#endif
		radio_start();

		#if DEBUG_RADIO_TIMING
		radio_debug_add_start(true, 0, 0xFFFFFFFF, BLE_CHAN_ADV, func_next_delayed_start);
		#endif

		return BLE_CHAN_ADV;
	}

	#if DEBUG_STOP
	if (stop)
	{
		static uint32_t stop_counter;
		static uint32_t stop_time;
		stop_counter++;
		if (stop_counter == 3)
		{
			stop_time = radio_tmr_get_radio_time();
			__debugbreak();
		}
	}
	#endif

	hal_radio_set_params(rx_get_params(radio_rx_item.chan_type));

	set_radio_start_time(radio_rx_item.radio_start_time);
	set_radio_stop_time(radio_rx_item.radio_end_time);

	start_mode = START_IMMEDIATE_STOP_DELAYED;

	#if DEBUG_RADIO_TIMING
	radio_debug_add_start(false, radio_rx_item.radio_start_time, radio_rx_item.radio_end_time, radio_rx_item.chan_type, func_next_delayed_start);
	#endif

	return radio_rx_item.chan_type;
}

//--------------------------------------------
static void next_delayed_start_packet_not_received(void)
{
	ble_chan_type = next_delayed_start();
}

//--------------------------------------------
static void next_delayed_start_packet_received_not_processed(void)
{
	ble_channel_type_after_packet_processing = next_delayed_start();
}

//--------------------------------------------
static void next_delayed_start_packet_received_and_processed(void)
{
	ble_chan_type = next_delayed_start();
}

//--------------------------------------------
static void aux_packet_not_received(void)
{
	if (ignore_aux_packet_not_received)
	{
		ignore_aux_packet_not_received = false;
		// proccess the next item from the radio start queue
		next_delayed_start_packet_not_received();
		return;
	}
	// AUX_CONNECT_REQ not received
	// start the advertising channel listening again
	radio_start();

	#if DEBUG_RADIO_TIMING
	radio_debug_add_start(true, 0, 0xFFFFFFFF, BLE_CHAN_ADV, func_aux_packet_not_received);
	#endif
}

//--------------------------------------------
static void rx_next_aux_sync_packet(bool packet_received, uint32_t timestamp)
{
	#define MAX_AUX_SYNC_IND_PKT_CNT  5
	static size_t aux_sync_ind_lost_pkt_cnt;
	if (packet_received)
	{
		uint32_t diff = timestamp > conf.aux.sync.calc_radio_sync_start_time ?
		timestamp - conf.aux.sync.calc_radio_sync_start_time :
		conf.aux.sync.calc_radio_sync_start_time - timestamp;
		uint32_t max_drift = conf.aux.sync.max_window_drift_rt;
		#if DEBUG_RADIO_TIMING
		if (diff > max_drift)
		{
			__debugbreak();
		}
		#endif
		conf.aux.sync.radio_sync_start_time = diff < max_drift ?
			timestamp :
			conf.aux.sync.calc_radio_sync_start_time;
		aux_sync_ind_lost_pkt_cnt = 0;
	}
	else
	{
		aux_sync_ind_lost_pkt_cnt++;
		if (aux_sync_ind_lost_pkt_cnt > MAX_AUX_SYNC_IND_PKT_CNT)
		{
			// start the advertising channel listening again
			radio_start();

			#if DEBUG_RADIO_TIMING
			radio_debug_add_start(true, 0, 0xFFFFFFFF, BLE_CHAN_ADV, func_rx_next_aux_sync_packet);
			#endif
			return;
		}
		conf.aux.sync.radio_sync_start_time = conf.aux.sync.calc_radio_sync_start_time;
	}

	start_mode = START_DELAYED_STOP_DELAYED;
	// set up next AUX_SYNC_IND radio reception
	radio_rx_item.chan_type = BLE_CHAN_AUX_SYNC;
	conf.aux.sync.calc_radio_sync_start_time = conf.aux.sync.radio_sync_start_time   // previous real or calculated AUX_SYNC_IND packet timestamp
		+ conf.aux.interval_rt;                                                      // + Interval
	radio_rx_item.radio_start_time = conf.aux.sync.calc_radio_sync_start_time        // auxiliary packet window start
		- conf.aux.sync.max_window_drift_rt                                          // - maximum window drift
		- MAX_RADIO_STARTUP_DELAY_RT;                                                // - maximum time for radio startup
	radio_rx_item.radio_end_time = conf.aux.sync.calc_radio_sync_start_time          // auxiliary packet window start
		+ ble_get_access_address_transmission_time_rt(conf.aux.params.phy)           // + time to receive access address
		+ conf.aux.sync.max_window_drift_rt;                                         // + maximum window drift
	radio_rx_list_put(radio_rx_item, FROM_AUX_SYNC_IND);

	conf.aux.event_counter += 1;
	conf.aux.params.channel = csa2_aux_sync_chan(&conf);
}

//--------------------------------------------
static void aux_sync_packet_processing(radio_packet_t *packet)
{
	rx_next_aux_sync_packet(true, packet->timestamp);
	conf.aux.aux_type = PKT_AUX_SYNC_IND;
}

//--------------------------------------------
static void aux_sync_packet_not_received(void)
{
	rx_next_aux_sync_packet(false, 0);
	// proccess the next item from the radio start queue
	next_delayed_start_packet_not_received();
}

//--------------------------------------------
static void acl_packet_not_received(void)
{
	#if DEBUG_STOP
	if (stop)
	{
		static uint32_t stop_counter;
		static uint32_t stop_time;
		stop_counter++;
		if (stop_counter == 2)
		{
			stop_time = radio_tmr_get_radio_time();
			__debugbreak();
		}
	}
	#endif

	if (conf.acl.connection_terminated)
	{
		// Connection terminated
		radio_start();

		#if DEBUG_RADIO_TIMING
		radio_debug_add_start(true, 0, 0xFFFFFFFF, BLE_CHAN_ADV, func_acl_packet_not_received);
		#endif

		return;
	}

	if (conf.acl.anchor_packet)
	{
		conf.acl.radio_anchor_time += conf.acl.time_cfg.interval_rt;
		conf.acl.conn_interval_counter++;
		if (conf.acl.time_cfg.interval_rt * conf.acl.conn_interval_counter > conf.acl.time_cfg.timeout_rt)
		{
			// Connection lost
			radio_start();

			#if DEBUG_RADIO_TIMING
			radio_debug_add_start(true, 0, 0xFFFFFFFF, BLE_CHAN_ADV, func_acl_packet_not_received);
			#endif

			return;
		}
	}

	conf.acl.event_counter++;
	conf.acl.packet_direction = 0;
	conf.acl.anchor_packet = true;

	// Link Layer update procedures (if needed)
	radio_conf_item_t radio_conf_item;
	uint32_t interval_rt = conf.acl.time_cfg.interval_rt;                    // connInterval
	uint32_t window_offset_rt = 0;                                           // transmitWindowOffset
	uint32_t window_size_rt = 0;                                             // transmitWindowSize
	bool new_max_window_drift;
	if (radio_conf_list_get(conf.acl.event_counter, &radio_conf_item))
	{
		switch (radio_conf_item.opcode)
		{
		case LL_PHY_UPDATE_IND:
			conf.acl.params.phy = radio_conf_item.phy;
			break;
		case LL_CHANNEL_MAP_IND:
			memcpy(conf.channel_map, radio_conf_item.channel_map, DATA_CHANNELS_BYTES_NUMBER);
			conf.channel_count = count_used_channels(conf.channel_map);
			break;
		case LL_CONNECTION_UPDATE_IND:
			conf.acl.time_cfg = radio_conf_item.time_cfg;
			window_offset_rt = conf.acl.time_cfg.window_offset_rt;           // transmitWindowOffset
			window_size_rt = conf.acl.time_cfg.window_size_rt;               // transmitWindowSize
			new_max_window_drift = true;
			break;
		case LL_CIS_IND:
			// not yet implemented
			__debugbreak();
			break;
		}
	}

	set_next_acl_channel();

	// set up next ACL radio reception
	radio_rx_item.chan_type = BLE_CHAN_ACL;
	conf.acl.calc_radio_anchor_time = conf.acl.radio_anchor_time             // previous real or calculated anchor point
		+ interval_rt                                                        // + connInterval
		+ window_offset_rt;                                                  // + transmitWindowOffset
	radio_rx_item.radio_start_time = conf.acl.calc_radio_anchor_time         // calculated new anchor point
		- conf.acl.max_window_drift_rt * conf.acl.conn_interval_counter      // - maximum window drift
		- MAX_RADIO_STARTUP_DELAY_RT;                                        // - maximum time for radio startup
	radio_rx_item.radio_end_time = conf.acl.calc_radio_anchor_time           // calculated new anchor point
		+ window_size_rt                                                     // + transmitWindowSize
		+ ble_get_access_address_transmission_time_rt(conf.acl.params.phy)   // + time to receive access address
		+ conf.acl.max_window_drift_rt * conf.acl.conn_interval_counter;     // + maximum window drift
	// add to radio start queue (delayed start and stop)
	radio_rx_list_put(radio_rx_item, FROM_BLE_CHAN_ACL);

	if (new_max_window_drift)
	{
		calculate_max_acl_window_drift();
	}

	// proccess the next item from the radio start queue
	next_delayed_start_packet_not_received();
}

//--------------------------------------------
static bool rx_next_iso_cig_packet(bool packet_received, bool close_iso_event)
{
	if (close_iso_event)
	{
		// CIS PDU with the CIE bit set to 1
		// 4.5.13.4 Closing CIS events
		while (conf.iso.sub_event_counter < conf.iso.num_sub_events - 1)
		{
			conf.iso.sub_event_counter++;
		}
	}

	conf.iso.sub_event_counter++;
	if (conf.iso.sub_event_counter < conf.iso.num_sub_events)
	{
		// new subevent
		conf.iso.params.channel = csa2_iso_subevent_chan(&conf);

		// immediate radio start
		hal_radio_set_params(rx_get_params(BLE_CHAN_ISO_CIG));
		hal_radio_start();

		uint32_t end_time = conf.iso.radio_anchor_time                          // previous real or calculated anchor point
			+ conf.iso.sub_interval_rt * conf.iso.sub_event_counter             // + sub_interval_rt * sub_event_counter
			+ ble_get_access_address_transmission_time_rt(conf.iso.params.phy)  // + time to receive access address
			+ MAX_WINDOW_DRIFT_RT;                                              // + maximum window drift

		// delayed radio stop
		set_radio_stop_time(end_time);

		#if DEBUG_RADIO_TIMING
		radio_debug_add_start(true, 0, end_time, BLE_CHAN_ISO_CIG, func_rx_next_iso_cig_packet);
		#endif

		return false;
	}
	else
	{
		// new event
		conf.iso.event_counter++;
		conf.iso.cig.cis_event_counter++;
		conf.iso.params.channel = csa2_iso_event_chan(&conf);
		conf.iso.sub_event_counter = 0;

		// set up next ISO radio reception
		radio_rx_item.chan_type = BLE_CHAN_ISO_CIG;
		conf.iso.calc_radio_anchor_time = conf.iso.radio_anchor_time            // previous real or calculated anchor point
			+ conf.iso.iso_interval_rt;                                         // + connInterval
		radio_rx_item.radio_start_time = conf.iso.calc_radio_anchor_time        // calculated new anchor point
			- MAX_WINDOW_DRIFT_RT                                               // - maximum window drift
			- MAX_RADIO_STARTUP_DELAY_RT;                                       // - maximum time for radio startup
		radio_rx_item.radio_end_time = conf.iso.calc_radio_anchor_time          // calculated new anchor point
			+ ble_get_access_address_transmission_time_rt(conf.iso.params.phy)  // + time to receive access address
			+ MAX_WINDOW_DRIFT_RT;                                              // + maximum window drift
		// add to radio start queue (delayed start and stop)
		radio_rx_list_put(radio_rx_item, FROM_BLE_CHAN_ISO);

		// proccess the next item from the radio start queue
		if (packet_received)
		{
			next_delayed_start_packet_received_not_processed();
		}
		else
		{
			next_delayed_start_packet_not_received();
		}
		return true;
	}
}

//--------------------------------------------
static bool iso_cig_packet_received(bool close_iso_event)
{
	return rx_next_iso_cig_packet(true, close_iso_event);
}

//--------------------------------------------
static void iso_cig_packet_not_received(void)
{
	if (conf.iso.anchor_packet)
	{
		conf.iso.radio_anchor_time = conf.iso.calc_radio_anchor_time;
	}

	conf.iso.anchor_packet = rx_next_iso_cig_packet(false, false);
}

//--------------------------------------------
static bool rx_next_pawr_packet(bool packet_received, uint32_t timestamp)
{
	#if DEBUG_RADIO_TIMING
	static size_t _pawr_lost_pkt_cnt;
	#endif

	if (conf.aux.next_aux_type == PKT_AUX_SYNC_SUBEVENT_IND)
	{
		#define MAX_AUX_SYNC_SUBEVENT_IND_PKT_CNT  5
		static size_t aux_sync_subevent_ind_lost_pkt_cnt;
		if (packet_received)
		{
			uint32_t diff = timestamp > conf.aux.pawr.calc_radio_sync_start_time ?
				timestamp - conf.aux.pawr.calc_radio_sync_start_time :
				conf.aux.pawr.calc_radio_sync_start_time - timestamp;
			uint32_t max_drift = conf.aux.sync.max_window_drift_rt;
			#if DEBUG_RADIO_TIMING
			if (diff > max_drift)
			{
				__debugbreak();
			}
			#endif
			conf.aux.pawr.radio_sync_start_time = diff < max_drift ?
				timestamp :
				conf.aux.pawr.calc_radio_sync_start_time;
			aux_sync_subevent_ind_lost_pkt_cnt = 0;
			#if DEBUG_STOP
			stop = true;
			#endif
		}
		else
		{
			#if DEBUG_RADIO_TIMING
			_pawr_lost_pkt_cnt++;
			__debugbreak();
			#endif
			aux_sync_subevent_ind_lost_pkt_cnt++;
			if (aux_sync_subevent_ind_lost_pkt_cnt > MAX_AUX_SYNC_SUBEVENT_IND_PKT_CNT)
			{
				// start the advertising channel listening again
				radio_start();

				#if DEBUG_RADIO_TIMING
				radio_debug_add_start(true, 0, 0xFFFFFFFF, BLE_CHAN_ADV, func_rx_next_pawr_packet);
				#endif

				return true;
			}
			conf.aux.pawr.radio_sync_start_time = conf.aux.pawr.calc_radio_sync_start_time;
		}

		conf.aux.next_aux_type = PKT_AUX_SYNC_SUBEVENT_RSP;
		conf.aux.params.address = conf.aux.pawr.rsp_address;

		start_mode = START_DELAYED_STOP_DELAYED;
		// set up AUX_SYNC_SUBEVENT_RSP radio reception
		radio_rx_item.chan_type = BLE_CHAN_PAWR;
		uint32_t aux_sync_start = conf.aux.pawr.radio_sync_start_time   // previous real or calculated AUX_SYNC_SUBEVENT_IND packet timestamp
			+ conf.aux.pawr.response_slot_delay_rt;                     // + response slot delay
		radio_rx_item.radio_start_time = aux_sync_start                 // auxiliary packet window start
			- conf.aux.sync.max_window_drift_rt                         // - maximum window drift
			- MAX_RADIO_STARTUP_DELAY_RT;                               // - maximum time for radio startup
		radio_rx_item.radio_end_time = aux_sync_start                   // auxiliary packet window start
			+ conf.aux.pawr.response_slots_spacing_rt                   // + response slots spacing
			+ conf.aux.sync.max_window_drift_rt;                        // + maximum window drift
		radio_rx_list_put(radio_rx_item, FROM_AUX_SYNC_SUBEVENT_IND);

		conf.aux.pawr.radio_rsp_end_time = radio_rx_item.radio_end_time;
	}
	else
	{
		#define ADD_PROC_TIME_RT  10
		uint32_t time = radio_tmr_get_radio_time() + ADD_PROC_TIME_RT;
		if (packet_received && time < conf.aux.pawr.radio_rsp_end_time)
		{
			// Response slots spacing, continue to accept responses
			// immediate radio start
			hal_radio_start();
			// delayed radio stop
			set_radio_stop_time(conf.aux.pawr.radio_rsp_end_time);
			return false;
		}

		uint32_t interval_rt;
		conf.aux.pawr.subevent_counter++;
		if (conf.aux.pawr.subevent_counter == conf.aux.pawr.num_subevents)
		{
			conf.aux.event_counter++;
			conf.aux.pawr.subevent_counter = 0;
			interval_rt = conf.aux.interval_rt - conf.aux.pawr.subevent_interval_rt * (conf.aux.pawr.num_subevents - 1);
		}
		else
		{
			interval_rt = conf.aux.pawr.subevent_interval_rt;
		}

		conf.aux.next_aux_type = PKT_AUX_SYNC_SUBEVENT_IND;
		conf.aux.params.channel = csa2_pawr_ind_chan(&conf);
		conf.aux.params.address = conf.aux.pawr.ind_address;

		start_mode = START_DELAYED_STOP_DELAYED;
		// set up next AUX_SYNC_SUBEVENT_IND radio reception
		radio_rx_item.chan_type = BLE_CHAN_PAWR;
		conf.aux.pawr.calc_radio_sync_start_time = conf.aux.pawr.radio_sync_start_time   // previous real or calculated AUX_SYNC_SUBEVENT_IND packet timestamp
			+ interval_rt;                                                               // + Interval
		radio_rx_item.radio_start_time = conf.aux.pawr.calc_radio_sync_start_time        // auxiliary packet window start
			- conf.aux.sync.max_window_drift_rt                                          // - maximum window drift
			- MAX_RADIO_STARTUP_DELAY_RT;                                                // - maximum time for radio startup
		radio_rx_item.radio_end_time = conf.aux.pawr.calc_radio_sync_start_time          // auxiliary packet window start
			+ ble_get_access_address_transmission_time_rt(conf.aux.params.phy)           // + time to receive access address
			+ conf.aux.sync.max_window_drift_rt;                                         // + maximum window drift
		radio_rx_list_put(radio_rx_item, FROM_AUX_SYNC_SUBEVENT_IND);
	}
	// proccess the next item from the radio start queue
	if (packet_received)
	{
		next_delayed_start_packet_received_not_processed();
	}
	else
	{
		next_delayed_start_packet_not_received();
	}
	return true;
}

//--------------------------------------------
static bool pawr_packet_received(uint32_t timestamp)
{
	conf.aux.aux_type = conf.aux.next_aux_type;
	return rx_next_pawr_packet(true, timestamp);
}

//--------------------------------------------
static void pawr_packet_not_received(void)
{
	rx_next_pawr_packet(false, 0);
}

//--------------------------------------------
static bool rx_next_iso_big_packet(bool packet_received, uint8_t *data, uint32_t timestamp)
{
	static bool last;
	#if DEBUG_RADIO_TIMING
	static size_t _iso_big_lost_pkt_cnt;
	#endif

	if (conf.iso.anchor_packet)
	{
		if (packet_received)
		{
			#define MAX_FIRST_BIG_WINDOW_DRIFT_RT  300
			#define MAX_BIG_WINDOW_DRIFT_RT        40
			uint32_t diff = timestamp > conf.iso.calc_radio_anchor_time ?
				timestamp - conf.iso.calc_radio_anchor_time :
				conf.iso.calc_radio_anchor_time - timestamp;
			#if 0
			// does not work
			uint32_t max_drift = calculate_max_big_anchor_window_drift();
			#else
			uint32_t max_drift = MAX_BIG_WINDOW_DRIFT_RT;
			#endif
			if (conf.iso.first_packet)
			{
				max_drift = MAX_FIRST_BIG_WINDOW_DRIFT_RT;
				conf.iso.first_packet = false;
			}
			#if DEBUG_RADIO_TIMING
			if (diff > max_drift)
			{
				__debugbreak();
			}
			#endif
			conf.iso.radio_anchor_time = diff < max_drift ?
				timestamp :
				conf.iso.calc_radio_anchor_time;
		}
		else
		{
			#if DEBUG_RADIO_TIMING
			__debugbreak();
			#endif
			conf.iso.radio_anchor_time = conf.iso.calc_radio_anchor_time;
		}
		conf.iso.anchor_packet = false;
	}

	if (!last)
	{
		#if DEBUG_RADIO_TIMING
		if (!packet_received)
		{
			_iso_big_lost_pkt_cnt++;
		}
		#endif
		conf.iso.sub_event_counter += 1;
		conf.iso.params = conf.iso.big.params[conf.iso.sub_event_counter];

		// immediate radio start
		hal_radio_set_params(rx_get_params(BLE_CHAN_ISO_BIG));
		hal_radio_start();

		uint32_t end_time = conf.iso.radio_anchor_time                          // previous real or calculated anchor point
			+ conf.iso.big.stop_time_offset[conf.iso.sub_event_counter]         // + offset
			+ ble_get_access_address_transmission_time_rt(conf.iso.params.phy)  // + time to receive access address
			+ MAX_WINDOW_DRIFT_RT;                                              // + maximum window drift

		#if DEBUG_RADIO_TIMING
		static volatile uint32_t now;
		now = radio_tmr_get_radio_time();
		if (end_time < now)
		{
			// why does this happen and what to do about it?
			__debugbreak();
		}
		#endif

		// delayed radio stop
		set_radio_stop_time(end_time);

		if (conf.iso.sub_event_counter == conf.iso.big.nse * conf.iso.big.num_bis)
		{
			last = true;
		}

		#if DEBUG_RADIO_TIMING
		radio_debug_add_start(true, 0, end_time, BLE_CHAN_ISO_BIG, func_rx_next_iso_big_packet);
		#endif

		return false;
	}
	else
	{
		last = false;

		if (packet_received)
		{
			// control subevent
			if (data[2] == BIG_TERMINATE_IND && conf.iso.event_counter == (*(uint16_t *)(data + 4) - 1))
			{
				// close BIG
				#if 0
				__debugbreak();
				#endif
				biginfo_already_proccessing = false;
				next_delayed_start_packet_received_not_processed();
				// because answer in 150 us is not needed
				start_mode = START_DELAYED_STOP_DELAYED;
				return true;
			}
			if (data[2] == BIG_CHANNEL_MAP_IND)
			{
				#if 0
				// not supported yet
				__debugbreak();
				#endif
			}
		}

		// set the event counter and calculate the channel number
		conf.iso.event_counter += 1;
		conf.iso.big.bis_event_counter = 1;
		conf.iso.big.bis_subevent_counter = 1;
		next_big_event_channel_numbers_calculating();
		conf.iso.packet_direction = 0;
		conf.iso.anchor_packet = true;

		conf.iso.sub_event_counter = 0;
		conf.iso.params = conf.iso.big.params[conf.iso.sub_event_counter];

		// set up next ISO radio reception
		radio_rx_item.chan_type = BLE_CHAN_ISO_BIG;
		conf.iso.calc_radio_anchor_time = conf.iso.radio_anchor_time            // previous real or calculated anchor point
			+ conf.iso.iso_interval_rt;                                         // + connInterval
		radio_rx_item.radio_start_time = conf.iso.calc_radio_anchor_time        // calculated new anchor point
			- MAX_WINDOW_DRIFT_RT                                               // - maximum window drift
			- MAX_RADIO_STARTUP_DELAY_RT;                                       // - maximum time for radio startup
		radio_rx_item.radio_end_time = conf.iso.calc_radio_anchor_time          // calculated new anchor point
			+ ble_get_access_address_transmission_time_rt(conf.iso.params.phy)  // + time to receive access address
			+ MAX_WINDOW_DRIFT_RT;                                              // + maximum window drift
		// add to radio start queue (delayed start and stop)
		radio_rx_list_put(radio_rx_item, FROM_BLE_CHAN_ISO_BIG);

		#if DEBUG_RADIO_TIMING
		if (radio_rx_item.radio_start_time < radio_tmr_get_radio_time())
		{
			__debugbreak();
		}
		#endif

		// proccess the next item from the radio start queue
		if (packet_received)
		{
			next_delayed_start_packet_received_not_processed();
		}
		else
		{
			next_delayed_start_packet_not_received();
		}
		return true;
	}
}

//--------------------------------------------
static bool iso_big_packet_received(uint8_t *data, uint32_t timestamp)
{
	return rx_next_iso_big_packet(true, data, timestamp);
}

//--------------------------------------------
static void iso_big_packet_not_received(void)
{
	rx_next_iso_big_packet(false, NULL, 0);
}

//--------------------------------------------
static void immediate_start_t_ifs_stop(void)
{
	// Start the radio to receive a packet with the same channel number after
	// the T_IFS time has elapsed since the end of reception of the previous packet
	hal_radio_start();

	uint32_t end_time = hal_rt_get_counter()
		+ T_IFS_RT
		+ ble_get_access_address_transmission_time_rt(get_current_phy())
		+ MAX_WINDOW_DRIFT_RT;

	set_radio_stop_time(end_time);

	#if DEBUG_RADIO_TIMING
	radio_debug_add_start(true, 0, end_time, ble_chan_type, func_immediate_start_t_ifs_stop);
	#endif
}

//--------------------------------------------
static void adv_packet_not_received(void)
{
	filter.look_for_connect = false;
	if (++filter.hop_map_count == filter.hop_map_size)
	{
		filter.hop_map_count = 0;
		conf.adv.params.channel = filter.hop_map[0];

		hal_radio_set_params(rx_get_params(BLE_CHAN_ADV));
		hal_radio_start();

		#if DEBUG_RADIO_TIMING
		radio_debug_add_start(true, 0, 0xFFFFFFFF, BLE_CHAN_ADV, func_adv_packet_not_received);
		#endif
	}
	else
	{
		conf.adv.params.channel = filter.hop_map[filter.hop_map_count];

		hal_radio_set_params(rx_get_params(BLE_CHAN_ADV));
		hal_radio_start();

		// hop time: 170
		// ToDo: make it customizable
		uint32_t end_time = hal_rt_get_counter() + 170;

		set_radio_stop_time(end_time);

		#if DEBUG_RADIO_TIMING
		radio_debug_add_start(true, 0, end_time, BLE_CHAN_ADV, func_adv_packet_not_received);
		#endif
	}
}

//--------------------------------------------
static bool adv_packet_received(uint8_t *data, uint32_t timestamp)
{
	uint8_t pdu_type = data[0] & PDU_TYPE_MASK;
	switch (pdu_type)
	{
	case ADV_IND:
	case ADV_DIRECT_IND:
	case ADV_NONCONN_IND:
	case ADV_SCAN_IND:
		if (!filter.mac_filter || (filter.mac_filter && !memcmp(filter.mac_addr, data + 2, DEVICE_ADDRESS_LENGTH)))
		{
			filter.look_for_connect = true;
			filter.do_not_follow = false;
			hal_radio_start();

			uint32_t end_time = hal_rt_get_counter()
				+ T_IFS_RT
				+ ble_get_access_address_transmission_time_rt(get_current_phy());

			set_radio_stop_time(end_time);

			#if DEBUG_RADIO_TIMING
			radio_debug_add_start(true, 0, end_time, BLE_CHAN_ADV, func_adv_packet_received);
			#endif

			return true;
		}
		filter.do_not_follow = true;
		break;
	case CONNECT_IND:
		if (filter.follow_filter & FOLLOW_CONN)
		{
			filter.look_for_connect = false;
			filter.do_not_follow = false;
			return false;
		}
		filter.do_not_follow = true;
		break;
	case SCAN_REQ:
		if (!filter.mac_filter || (filter.mac_filter && (!memcmp(filter.mac_addr, data + 2, DEVICE_ADDRESS_LENGTH) ||
			!memcmp(filter.mac_addr, data + 2 + DEVICE_ADDRESS_LENGTH, DEVICE_ADDRESS_LENGTH))))
		{
			filter.do_not_follow = false;
		}
		else
		{
			filter.do_not_follow = true;
		}
		break;
	case SCAN_RSP:
		if (!filter.mac_filter || (filter.mac_filter && !memcmp(filter.mac_addr, data + 2, DEVICE_ADDRESS_LENGTH)))
		{
			filter.do_not_follow = false;
		}
		else
		{
			filter.do_not_follow = true;
		}
		break;
	case ADV_EXT_IND:
		if ((!filter.mac_filter || (filter.mac_filter && (data[3] & EXTENDED_HEADER_ADVERTISING_DATA_INFO_Msk) &&
			!memcmp(filter.mac_addr, data + 4, DEVICE_ADDRESS_LENGTH))))
		{
			filter.look_for_connect = false;
			filter.do_not_follow = false;
			return false;
		}
		filter.look_for_connect = true;
		filter.do_not_follow = true;
		break;
	}

	if (filter.look_for_connect)
	{
		adv_packet_not_received();
		return true;
	}

	hal_radio_set_params(rx_get_params(BLE_CHAN_ADV));
	hal_radio_start();

	#if DEBUG_RADIO_TIMING
	radio_debug_add_start(true, 0, 0xFFFFFFFF, BLE_CHAN_ADV, func_adv_packet_received);
	#endif

	return true;
}

//--------------------------------------------
static void radio_irq_callback(uint8_t *data, bool rx_done)
{
	bool start_immediate_stop_delayed = false;
	bool set_iso_anchor_packet = false;
	bool restore_ble_channel_type_after_packet_processing = false;
	bool anchor_packet = conf.iso.anchor_packet;
	static radio_packet_t *packet;

	#if DEBUG_RADIO_TIMING
	radio_debug_add_stop(rx_done);
	#endif

	if (!radio_started)
	{
		return;
	}

	if (!rx_done)
	{
		// radio packet not received
		switch (ble_chan_type)
		{
		case BLE_CHAN_ADV:
			adv_packet_not_received();
			break;
		case BLE_CHAN_ACL:
			acl_packet_not_received();
			break;
		case BLE_CHAN_AUX:
			aux_packet_not_received();
			break;
		case BLE_CHAN_AUX_SYNC:
			aux_sync_packet_not_received();
			break;
		case BLE_CHAN_PAWR:
			pawr_packet_not_received();
			break;
		case BLE_CHAN_ISO_CIG:
			iso_cig_packet_not_received();
			conf.iso.packet_direction = 0;
			break;
		case BLE_CHAN_ISO_BIG:
			iso_big_packet_not_received();
			break;
		default:
			__debugbreak();
			break;
		}
		return;
	}

	// radio packet received
	msg_net2app.type = IPC_NET2APP_BLE_PACKET;
	msg_net2app.size = sizeof(radio_params_t);
	packet = (radio_packet_t *)msg_net2app.data;

	radio_params_t *rx_params = rx_get_params(ble_chan_type);
	packet->channel = rx_params->channel;
	packet->phy = rx_params->phy;
	packet->address = rx_params->address;
	packet->crc_init = rx_params->crc_init;
	packet->timestamp = hal_rt_get_address_time() - ble_get_access_address_transmission_time_rt((ble_phy_t)packet->phy);
	packet->mic_status = MIC_UNKNOWN;

	bool crc_ok = hal_radio_crc_is_valid();

	#if DEBUG_RADIO_TIMING
	radio_debug_add_timestamp(packet->timestamp);
	#endif

	if (start_mode != START_DELAYED_STOP_DELAYED)
	{
		start_immediate_stop_delayed = true;
		switch (ble_chan_type)
		{
		case BLE_CHAN_ISO_CIG:
		{
			if (!conf.iso.packet_direction)
			{
				immediate_start_t_ifs_stop();
			}
			else
			{
				set_iso_anchor_packet = iso_cig_packet_received(data[0] & 0x10); // CIE bit
			}
			conf.iso.packet_direction = !conf.iso.packet_direction;
			break;
		}
		case BLE_CHAN_ISO_BIG:
			restore_ble_channel_type_after_packet_processing = iso_big_packet_received(data, packet->timestamp);
			break;
		case BLE_CHAN_PAWR:
			restore_ble_channel_type_after_packet_processing = pawr_packet_received(packet->timestamp);
			break;
		case BLE_CHAN_ADV:
			if (crc_ok)
			{
				start_immediate_stop_delayed = adv_packet_received(data, packet->timestamp);
			}
			else
			{
				adv_packet_not_received();
			}
			break;
		case BLE_CHAN_ACL:
		case BLE_CHAN_AUX:
			immediate_start_t_ifs_stop();
			break;
		case BLE_CHAN_AUX_SYNC:
			start_immediate_stop_delayed = false;
			break;
		default:
			break;
		}
	}

	packet->rssi = hal_radio_get_rssi();
	packet->crc_status = crc_ok ? CRC_VALID : CRC_FAIL;

	switch (ble_chan_type)
	{
	case BLE_CHAN_ADV:
	case BLE_CHAN_AUX:
		packet->direction = 0;
		packet->event_counter = 0;
		break;
	case BLE_CHAN_ACL:
		packet->direction = conf.acl.packet_direction;
		packet->event_counter = conf.acl.event_counter;
		break;
	case BLE_CHAN_PAWR:
	case BLE_CHAN_AUX_SYNC:
		packet->event_counter = conf.aux.event_counter;
		break;
	case BLE_CHAN_ISO_CIG:
		if (set_iso_anchor_packet)
		{
			packet->event_counter = (uint16_t)(conf.iso.cig.cis_event_counter - 1);
		}
		else
		{
			packet->event_counter = (uint16_t)conf.iso.cig.cis_event_counter;
		}
		packet->direction = !conf.iso.packet_direction;
		break;
	case BLE_CHAN_ISO_BIG:
		packet->direction = 0;
		packet->event_counter = conf.iso.event_counter;
		break;
	default:
		break;
	}

	packet->size = data[1] + MINIMUM_HEADER_LENGTH;
	memcpy(packet->data, data, packet->size);

	switch (ble_chan_type)
	{
	case BLE_CHAN_ADV:
		if (!filter.do_not_follow)
		{
			if (crc_ok)
			{
				adv_packet_processing(packet);
			}
		}
		packet->pdu_type = PDU_ADV;
		break;
	case BLE_CHAN_ACL:
		acl_packet_processing_anchor(packet);
		acl_packet_processing_encryption(packet);
		packet->pdu_type = PDU_ACL;
		break;
	case BLE_CHAN_AUX:
		if (crc_ok)
		{
			aux_packet_processing(packet);
		}
		packet->pdu_type = PDU_AUX;
		packet->aux_pdu_type = conf.aux.aux_type;
		break;
	case BLE_CHAN_AUX_SYNC:
		aux_sync_packet_processing(packet);
		packet->pdu_type = PDU_AUX;
		packet->aux_pdu_type = conf.aux.aux_type;
		break;
	case BLE_CHAN_PAWR:
		pawr_packet_processing(packet);
		packet->pdu_type = PDU_AUX;
		packet->aux_pdu_type = conf.aux.aux_type;
		break;
	case BLE_CHAN_ISO_CIG:
		iso_cig_packet_processing(packet, anchor_packet);
		if (set_iso_anchor_packet)
		{
			conf.iso.anchor_packet = true;
		}
		packet->pdu_type = PDU_ISO_CIG;
		break;
	case BLE_CHAN_ISO_BIG:
		iso_big_packet_processing(packet);
		packet->pdu_type = PDU_ISO_BIG;
		break;
	default:
		break;
	}

	if (set_iso_anchor_packet || restore_ble_channel_type_after_packet_processing)
	{
		ble_chan_type = ble_channel_type_after_packet_processing;
	}

	if (start_mode == START_IMMEDIATE_STOP_NO && !hal_radio_is_started())
	{
		radio_start();

		#if DEBUG_RADIO_TIMING
		radio_debug_add_start(true, 0, 0xFFFFFFFF, ble_chan_type, func_radio_irq_callback);
		#endif
	}

	if (start_immediate_stop_delayed == false && start_mode == START_DELAYED_STOP_DELAYED)
	{
		next_delayed_start_packet_received_and_processed();
	}

	if (!crc_ok && ble_chan_type == BLE_CHAN_ADV)
	{
		return;
	}
	if (filter.rssi_filter && packet->rssi <  filter.rssi && ble_chan_type == BLE_CHAN_ADV)
	{
		return;
	}
	if (filter.mac_filter && filter.do_not_follow)
	{
		return;
	}

	msg_queue_net2app_put(&msg_net2app);
}

//--------------------------------------------
void radio_init(void)
{
	hal_rt_init();
	hal_ipc_init();
	hal_ipc_set_mem_callback(mem_callback);
	hal_radio_init();
	hal_radio_set_irq_callback(radio_irq_callback);
	hal_dppi_init();
	msg_queue_rx2decrypt_init();

	msg_net2app.type = IPC_NET2APP_READY;
	msg_net2app.size = 0;
	assert(msg_queue_net2app_put(&msg_net2app));
}

//--------------------------------------------
static void radio_reset(void)
{
	radio_stop();

	filter.follow_filter = FOLLOW_CONN | FOLLOW_PA | FOLLOW_CIS | FOLLOW_BIS;
	filter.mac_filter = false;
	filter.hop_map[0] = 37;
	filter.hop_map[1] = 38;
	filter.hop_map[2] = 39;
	filter.hop_map_size = 3;
	filter.rssi_filter = false;

	decrypt_packet = false;
}

//--------------------------------------------
static void radio_start(void)
{
	if (filter.follow_filter & FOLLOW_PA)
	{
		sync_aux_packet = true;
	}
	else
	{
		sync_aux_packet = false;
	}

	// clear radio rx list
	radio_rx_list_init();
	conf.adv.params.channel = filter.hop_map[0];
	if (filter.hop_map_size == 1)
	{
		start_mode = START_IMMEDIATE_STOP_NO;
	}
	else
	{
		filter.hop_map_count = 0;
		start_mode = START_IMMEDIATE_STOP_DELAYED;
	}
	ble_chan_type = BLE_CHAN_ADV;
	hal_radio_set_params(rx_get_params(BLE_CHAN_ADV));
	radio_started = true;
	hal_radio_start();

	#if DEBUG_RADIO_TIMING
	radio_debug_add_start(true, 0, 0xFFFFFFFF, BLE_CHAN_ADV, func_radio_start);
	#endif
}

//--------------------------------------------
static void radio_stop(void)
{
	radio_started = false;
	hal_radio_stop();
}

//--------------------------------------------
static void radio_set_hop_map(uint8_t map_size, uint8_t *map)
{
	memcpy(filter.hop_map, map, map_size);
	filter.hop_map_size = map_size;
	filter.hop_map_count = 0;
}

//--------------------------------------------
static void radio_set_follow_filter(uint8_t follow_filter)
{
	filter.follow_filter = follow_filter;
}

//--------------------------------------------
static void radio_set_ltk(uint8_t *ltk)
{
	crypto_set_ltk(ltk);
	decrypt_packet = true;
}

//--------------------------------------------
static void radio_reset_ltk(void)
{
	decrypt_packet = false;
}

//--------------------------------------------
static void radio_set_mac_filter(uint8_t *mac_addr)
{
	memcpy(filter.mac_addr, mac_addr, DEVICE_ADDRESS_LENGTH);
	filter.mac_filter = true;
}

//--------------------------------------------
static void radio_reset_mac_filter(void)
{
	filter.mac_filter = false;
}

//--------------------------------------------
static void radio_set_rssi_filter(int8_t rssi)
{
	filter.rssi = rssi;
	filter.rssi_filter = true;
}

//--------------------------------------------
static void radio_reset_rssi_filter(void)
{
	filter.rssi_filter = false;
}

//--------------------------------------------
static void mem_callback(void)
{
	while (msg_queue_app2net_get(&msg_app2net))
	{
		switch (msg_app2net.type)
		{
		case IPC_APP2NET_RADIO_RESET:
			radio_reset();
			break;
		case IPC_APP2NET_RADIO_START:
			radio_start();
			break;
		case IPC_APP2NET_RADIO_STOP:
			radio_stop();
			break;
		case IPC_APP2NET_SET_HOP_MAP:
			radio_set_hop_map(msg_app2net.data[0], &msg_app2net.data[1]);
			break;
		case IPC_APP2NET_SET_FOLLOW_FILTER:
			radio_set_follow_filter(msg_app2net.data[0]);
			break;
		case IPC_APP2NET_SET_LTK:
			radio_set_ltk(msg_app2net.data);
			break;
		case IPC_APP2NET_RESET_LTK:
			radio_reset_ltk();
			break;
		case IPC_APP2NET_SET_MAC_FILTER:
			radio_set_mac_filter(msg_app2net.data);
			break;
		case IPC_APP2NET_RESET_MAC_FILTER:
			radio_reset_mac_filter();
			break;
		case IPC_APP2NET_SET_RSSI_FILTER:
			radio_set_rssi_filter(msg_app2net.data[0]);
			break;
		case IPC_APP2NET_RESET_RSSI_FILTER:
			radio_reset_rssi_filter();
			break;
		default:
			break;
		}
#if DEBUG_APP2NET
		app2net_debug_add(msg_app2net.type);
#endif
	}
}
