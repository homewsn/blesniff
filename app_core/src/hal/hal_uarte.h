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

#ifndef HAL_UARTE_H_
#define HAL_UARTE_H_

//--------------------------------------------
typedef void (*uarte_rx_callback_t)(uint8_t byte);

//--------------------------------------------
void hal_uarte_init(void);
void hal_uarte_write(uint8_t *data, uint32_t size);
void hal_uarte_set_rx_callback(uarte_rx_callback_t callback);

#endif /* HAL_UARTE_H_ */