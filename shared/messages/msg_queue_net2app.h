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

#ifndef MSG_QUEUE_NET2APP_H_
#define MSG_QUEUE_NET2APP_H_

//--------------------------------------------
#define NET2APP2_DATA_LENGTH  (sizeof(radio_packet_t))

//--------------------------------------------
typedef struct
{
	ipc_msg_type_t type;
	size_t size;
	uint8_t data[NET2APP2_DATA_LENGTH];
} msg_net2app_t;

//--------------------------------------------
#ifdef NRF5340_XXAA_APPLICATION
void msg_queue_net2app_init(void);
#endif
bool msg_queue_net2app_put(msg_net2app_t *msg);
bool msg_queue_net2app_get(msg_net2app_t *msg);

#endif /* MSG_QUEUE_NET2APP_H_ */