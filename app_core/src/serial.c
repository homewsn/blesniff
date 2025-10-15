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
#include <stddef.h>    /* size_t */
#include <string.h>    /* memcpy */
#include <stdbool.h>   /* bool */
#include <assert.h>    /* assert */
#include "cmsis_compiler.h"  /* __disable_irq, __enable_irq */
#include "radio_packet.h"
#include "ipc_msg_types.h"
#include "msg_queue_app2net.h"
#include "msg_queue_net2app.h"
#include "serial.h"
#include "msg_queue_ipc2serial.h"
#include "msg_queue_serial2ipc.h"

//--------------------------------------------
#define SYNC_BYTE0              0x55
#define SYNC_BYTE1              0xAA
//--------------------------------------------
#define CMD_RESET               0x01
#define CMD_START               0x02
#define CMD_STOP                0x03
#define CMD_SET_HOP_MAP         0x04
#define CMD_SET_FOLLOW_FILTER   0x05
#define CMD_SET_LTK             0x06
#define CMD_RESET_LTK           0x07
#define CMD_SET_MAC_FILTER      0x08
#define CMD_RESET_MAC_FILTER    0x09
#define CMD_SET_RSSI_FILTER     0x0A
#define CMD_RESET_RSSI_FILTER   0x0B
//--------------------------------------------
#define MSG_BLE_PACKET          0x80
#define MSG_INFO                0x81

//--------------------------------------------
static msg_app2net_t msg_app2net;
static msg_net2app_t msg_net2app;

//--------------------------------------------
extern bool serial_enabled;

//--------------- Debugging ------------------
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#ifndef DEBUG_APP2NET
#define DEBUG_APP2NET              0
#endif
#ifndef DEBUG_DERIAL2IPC
#define DEBUG_DERIAL2IPC           0
#endif
//--------------------------------------------
#if DEBUG_APP2NET
//--------------------------------------------
typedef struct
{
	ipc_msg_type_t command;
} app2net_debug_t;
//--------------------------------------------
#define APP2NET_ARRAY_SIZE 10
static volatile app2net_debug_t app2net_debug[APP2NET_ARRAY_SIZE];
static size_t app2net_debug_cnt = 0;
//--------------------------------------------
static void app2net_debug_add(ipc_msg_type_t command)
{
	app2net_debug[app2net_debug_cnt].command = command;
	app2net_debug_cnt++;
	if (app2net_debug_cnt == APP2NET_ARRAY_SIZE)
	{
		app2net_debug_cnt = 0;
	}
}
#endif
//--------------------------------------------
#if DEBUG_DERIAL2IPC
//--------------------------------------------
typedef struct
{
	uint8_t command;
	msg_serial2ipc_t msg;
} serial2ipc_debug_t;
//--------------------------------------------
#define SERIAL2IPC_ARRAY_SIZE 10
static volatile serial2ipc_debug_t serial2ipc_debug[SERIAL2IPC_ARRAY_SIZE];
static size_t serial2ipc_debug_cnt = 0;
//--------------------------------------------
static void serial2ipc_debug_add(msg_serial2ipc_t *msg)
{
	memcpy((msg_serial2ipc_t *)&serial2ipc_debug[serial2ipc_debug_cnt].msg, msg, sizeof(msg_serial2ipc_t));
	serial2ipc_debug[serial2ipc_debug_cnt].command = msg->data[2];
	serial2ipc_debug_cnt++;
	if (serial2ipc_debug_cnt == SERIAL2IPC_ARRAY_SIZE)
	{
		serial2ipc_debug_cnt = 0;
	}
}
#endif
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


//--------------------------------------------
void serial_tx_ble_packet(void)
{
	static msg_ipc2serial_t msg;
	radio_packet_t *packet;

	packet = (radio_packet_t *)msg_net2app.data;

	uint8_t *msg_ptr = msg.data + MSG_HEADER_SIZE;

	// timestamp (little endian)
	*msg_ptr++ = packet->timestamp;
	*msg_ptr++ = packet->timestamp >> 8;
	*msg_ptr++ = packet->timestamp >> 16;
	*msg_ptr++ = packet->timestamp >> 24;
	// event_counter (little endian)
	*msg_ptr++ = packet->event_counter;
	*msg_ptr++ = packet->event_counter >> 8;
	// channel
	*msg_ptr++ = packet->channel;
	// rssi
	*msg_ptr++ = (uint8_t)packet->rssi;
	// phy, direction, enc_status, mic_status, crc_status
	*msg_ptr++ = (uint8_t)((packet->phy << 6) | (packet->direction << 4) | (packet->enc_status << 2) | (packet->mic_status << 1) | (packet->crc_status));
	// pdu_type, aux_pdu_type
	*msg_ptr++ = (uint8_t)((packet->pdu_type << 4) | (packet->aux_pdu_type));
	// address (little endian)
	*msg_ptr++ = packet->address;
	*msg_ptr++ = packet->address >> 8;
	*msg_ptr++ = packet->address >> 16;
	*msg_ptr++ = packet->address >> 24;
	// crc_init (little endian)
	*msg_ptr++ = packet->crc_init;
	*msg_ptr++ = packet->crc_init >> 8;
	*msg_ptr++ = packet->crc_init >> 16;
	*msg_ptr++ = packet->crc_init >> 24;
	// size (little endian)
	*msg_ptr++ = packet->size;
	*msg_ptr++ = packet->size >> 8;
	// pdu
	memcpy(msg_ptr, packet->data, packet->size);
	msg_ptr += packet->size;

	msg.size = msg_ptr - msg.data;

	msg.data[0] = SYNC_BYTE0;
	msg.data[1] = SYNC_BYTE1;
	msg.data[2] = MSG_BLE_PACKET;
	msg.data[3] = (msg.size - MSG_HEADER_SIZE);
	msg.data[4] = (msg.size - MSG_HEADER_SIZE) >> 8;

	if (serial_enabled)
	{
		msg_queue_ipc2serial_put(&msg);
	}
}

//------------------------------------------------------------------
static int32_t serial_message_check(const uint8_t *buf, const size_t size)
{
	int32_t len;

	if (!size || buf[0] != SYNC_BYTE0)
	{
		return -1;
	}
	if (size < MSG_HEADER_SIZE)
	{
		return 0;
	}
	if (buf[1] != SYNC_BYTE1)
	{
		return -1;
	}
	len = *(buf + 3) | (*(buf + 4) << 8);
	if (size < MSG_HEADER_SIZE + len)
	{
		return 0;
	}
	assert(len <= APP2NET_DATA_LENGTH);
	return (MSG_HEADER_SIZE + len);
}

//--------------------------------------------
void serial_rx_callback(uint8_t byte)
{
	static uint8_t serial_rx_buf[CMD_BUF_LENGTH];
	static size_t serial_rx_cnt;
	static msg_serial2ipc_t msg;
	int32_t len;

	if (serial_rx_cnt == sizeof(serial_rx_buf))
	{
		serial_rx_cnt = 0;
	}
	serial_rx_buf[serial_rx_cnt++] = byte;

	len = serial_message_check(serial_rx_buf, serial_rx_cnt);
	if (len < 0)
	{
		serial_rx_cnt = 0;
	}
	if (len > 0)
	{
		msg.size = len;
		memcpy(msg.data, serial_rx_buf, msg.size);
		msg_queue_serial2ipc_put(&msg);
#if DEBUG_DERIAL2IPC
		serial2ipc_debug_add(&msg);
#endif
		serial_rx_cnt = 0;
	}
}

//--------------------------------------------
void serial_rx(void)
{
	static msg_serial2ipc_t msg;
	bool res;

	__disable_irq();
	res =  msg_queue_serial2ipc_get(&msg);
	__enable_irq();

	if (!res)
	{
		return;
	}

	switch (msg.data[2])
	{
	case CMD_RESET:
		msg_app2net.type = IPC_APP2NET_RADIO_RESET;
		msg_app2net.size = 0;
		break;
	case CMD_START:
		msg_app2net.type = IPC_APP2NET_RADIO_START;
		msg_app2net.size = 0;
		break;
	case CMD_STOP:
		msg_app2net.type = IPC_APP2NET_RADIO_STOP;
		msg_app2net.size = 0;
		break;
	case CMD_SET_HOP_MAP:
		msg_app2net.type = IPC_APP2NET_SET_HOP_MAP;
		msg_app2net.size = 4;
		memcpy(msg_app2net.data, &msg.data[5], 4);
		break;
	case CMD_SET_FOLLOW_FILTER:
		msg_app2net.type = IPC_APP2NET_SET_FOLLOW_FILTER;
		msg_app2net.size = 1;
		msg_app2net.data[0] = msg.data[5];
		break;
	case CMD_SET_LTK:
		msg_app2net.type = IPC_APP2NET_SET_LTK;
		msg_app2net.size = 16;
		memcpy(msg_app2net.data, &msg.data[5], 16);
		break;
	case CMD_RESET_LTK:
		msg_app2net.type = IPC_APP2NET_RESET_LTK;
		msg_app2net.size = 0;
		break;
	case CMD_SET_MAC_FILTER:
		msg_app2net.type = IPC_APP2NET_SET_MAC_FILTER;
		msg_app2net.size = 6;
		memcpy(msg_app2net.data, &msg.data[5], 6);
		break;
	case CMD_RESET_MAC_FILTER:
		msg_app2net.type = IPC_APP2NET_RESET_MAC_FILTER;
		msg_app2net.size = 0;
		break;
	case CMD_SET_RSSI_FILTER:
		msg_app2net.type = IPC_APP2NET_SET_RSSI_FILTER;
		msg_app2net.size = 1;
		msg_app2net.data[0] = msg.data[5];
		break;
	case CMD_RESET_RSSI_FILTER:
		msg_app2net.type = IPC_APP2NET_RESET_RSSI_FILTER;
		msg_app2net.size = 0;
		break;
	default:
		return;
	}
#if DEBUG_APP2NET
	app2net_debug_add(msg_app2net.type);
#endif
	msg_queue_app2net_put(&msg_app2net);
}

//--------------------------------------------
void mem_callback(void)
{
	if (!msg_queue_net2app_get(&msg_net2app))
	{
		return;
	}
	switch (msg_net2app.type)
	{
	case IPC_NET2APP_READY:
		msg_app2net.type = IPC_APP2NET_RADIO_RESET;
		msg_app2net.size = 0;
		msg_queue_app2net_put(&msg_app2net);
		msg_app2net.type = IPC_APP2NET_RADIO_START;
		msg_app2net.size = 0;
		msg_queue_app2net_put(&msg_app2net);
#if DEBUG_APP2NET
		app2net_debug_add(msg_app2net.type);
#endif
		break;
	case IPC_NET2APP_BLE_PACKET:
		serial_tx_ble_packet();
		break;
	default:
		break;
	}
}
