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

#ifndef IPC_MSG_TYPES_H_
#define IPC_MSG_TYPES_H_

//------------------------------
typedef enum
{
	IPC_NET2APP_READY,
	IPC_NET2APP_BLE_PACKET,
	IPC_NET2APP_INFO_MESSAGE,
	IPC_APP2NET_RADIO_RESET,
	IPC_APP2NET_RADIO_START,
	IPC_APP2NET_RADIO_STOP,
	IPC_APP2NET_SET_HOP_MAP,
	IPC_APP2NET_SET_FOLLOW_FILTER,
	IPC_APP2NET_SET_LTK,
	IPC_APP2NET_RESET_LTK,
	IPC_APP2NET_SET_MAC_FILTER,
	IPC_APP2NET_RESET_MAC_FILTER,
	IPC_APP2NET_SET_RSSI_FILTER,
	IPC_APP2NET_RESET_RSSI_FILTER,
} ipc_msg_type_t;


#endif /* IPC_MSG_TYPES_H_ */