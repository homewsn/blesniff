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

#ifndef SERIAL_H_
#define SERIAL_H_

//--------------------------------------------
#define MSG_HEADER_SIZE         5
#define CMD_MAX_LENGTH          (MSG_HEADER_SIZE + APP2NET_DATA_LENGTH)
#define CMD_BUF_LENGTH          (CMD_MAX_LENGTH * 2)

//--------------------------------------------
void serial_rx_callback(uint8_t byte);
void serial_rx(void);
void mem_callback(void);


#endif /* SERIAL_H_ */