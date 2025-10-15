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

#ifndef HAL_IPC_H_
#define HAL_IPC_H_

//--------------------------------------------
typedef void (*mem_read_callback)(void);

//--------------------------------------------
void hal_ipc_init(void);
void hal_ipc_send_mem_signal(void);
void hal_ipc_set_mem_callback(mem_read_callback callback);

#endif /* HAL_IPC_H_ */