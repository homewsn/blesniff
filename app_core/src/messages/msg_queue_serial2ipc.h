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

#ifndef MSG_QUEUE_SERIAL2IPC_H_
#define MSG_QUEUE_SERIAL2IPC_H_

//--------------------------------------------
#define MAX_SERIAL2IPC_MSG_SIZE    CMD_MAX_LENGTH
//--------------------------------------------
typedef struct
{
	size_t size;
	uint8_t data[MAX_SERIAL2IPC_MSG_SIZE];
} msg_serial2ipc_t;

//--------------------------------------------
void msg_queue_serial2ipc_init(void);
bool msg_queue_serial2ipc_put(msg_serial2ipc_t *msg);
bool msg_queue_serial2ipc_get(msg_serial2ipc_t *msg);

#endif /* MSG_QUEUE_SERIAL2IPC_H_ */