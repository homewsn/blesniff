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

#ifndef MSG_QUEUE_IPC2UARTE_H_
#define MSG_QUEUE_IPC2UARTE_H_

//--------------------------------------------
#define MAX_IPC2SERIAL_MSG_SIZE    300
//--------------------------------------------
typedef struct
{
	size_t size;
	uint8_t data[MAX_IPC2SERIAL_MSG_SIZE];
} msg_ipc2serial_t;

//--------------------------------------------
void msg_queue_ipc2serial_init(void);
bool msg_queue_ipc2serial_put(msg_ipc2serial_t *msg);
bool msg_queue_ipc2serial_get(msg_ipc2serial_t *msg);

#endif /* MSG_QUEUE_IPC2UARTE_H_ */