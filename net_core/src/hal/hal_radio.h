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

#ifndef HAL_RADIO_H_
#define HAL_RADIO_H_

//--------------------------------------------
#define HAL_RADIO_FAST_RUMP_UP    1

//--------------------------------------------
typedef void (*radio_irq_callback_t)(uint8_t *rx_buffer, bool rx_done);

//--------------------------------------------
void hal_radio_init(void);
void hal_radio_set_params(radio_params_t *params);
void hal_radio_stop(void);
void hal_radio_start(void);
void hal_radio_set_irq_callback(radio_irq_callback_t callback);
int8_t hal_radio_get_rssi(void);
bool hal_radio_crc_is_valid(void);
bool hal_radio_is_started(void);

#endif /* HAL_RADIO_H_ */