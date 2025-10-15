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

#ifndef MSG_QUEUE_APP2NET_H_
#define MSG_QUEUE_APP2NET_H_

//--------------------------------------------
#define APP2NET_DATA_LENGTH  16

//--------------------------------------------
typedef struct
{
	ipc_msg_type_t type;
	size_t size;
	uint8_t data[APP2NET_DATA_LENGTH];
} msg_app2net_t;

//--------------------------------------------
#ifdef NRF5340_XXAA_APPLICATION
void msg_queue_app2net_init(void);
#endif
bool msg_queue_app2net_put(msg_app2net_t *msg);
bool msg_queue_app2net_get(msg_app2net_t *msg);

#endif /* MSG_QUEUE_APP2NET_H_ */