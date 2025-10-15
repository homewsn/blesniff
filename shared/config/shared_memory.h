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

#ifndef SHARED_MEMORY_H_
#define SHARED_MEMORY_H_

#include "shared_memory_base.h"
#include "msg_queue.h"
#include "msg_queue_app2net.h"
#include "msg_queue_net2app.h"

//------------------------------
#define MSG_QUEUE_APP2NET_SIZE          10
#define MSG_QUEUE_NET2APP_SIZE          20

//------------------------------
#define MSG_QUEUE_APP2NET_SIZEOF        (sizeof(msg_queue_t))
#define MSG_QUEUE_APP2NET_BUF_SIZEOF    (MSG_QUEUE_APP2NET_SIZE * sizeof(msg_app2net_t))
#define MSG_QUEUE_NET2APP_SIZEOF        (sizeof(msg_queue_t))
#define MSG_QUEUE_NET2APP_BUF_SIZEOF    (MSG_QUEUE_NET2APP_SIZE * sizeof(msg_net2app_t))

//------------------------------
#define ALIGN_32BIT(addr)               ((((addr) + 3) >> 2) << 2)

//------------------------------
#define MSG_QUEUE_APP2NET_ADDR          SHARED_MEMORY_BASE
#define MSG_QUEUE_APP2NET_BUF_ADDR      ALIGN_32BIT((MSG_QUEUE_APP2NET_ADDR + MSG_QUEUE_APP2NET_SIZEOF))
#define MSG_QUEUE_NET2APP_ADDR          ALIGN_32BIT((MSG_QUEUE_APP2NET_BUF_ADDR + MSG_QUEUE_APP2NET_BUF_SIZEOF))
#define MSG_QUEUE_NET2APP_BUF_ADDR      ALIGN_32BIT((MSG_QUEUE_NET2APP_ADDR + MSG_QUEUE_NET2APP_SIZEOF))

//------------------------------
_Static_assert((MSG_QUEUE_NET2APP_BUF_ADDR + MSG_QUEUE_NET2APP_BUF_SIZEOF - MSG_QUEUE_APP2NET_ADDR) <= SHARED_MEMORY_RAMREGION_SIZE,\
	"Not enough SHARED_MEMORY_RAMREGION_SIZE for shared memory");

#endif // SHARED_MEMORY_H_