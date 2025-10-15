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

#ifndef HAL_RT_H_
#define HAL_RT_H_

//--------------------------------------------
#define GET_COUNTER_CH               0
#define RADIO_RXEN_COMPARE_CH        1
#define RADIO_DISABLE_COMPARE_CH     2
#define RADIO_ADDRESS_CAPTURE_CH     3
#define RADIO_END_CAPTURE_CH         4
#define RADIO_RXREADY_CAPTURE_CH     5
#define RADIO_DISABLED_CAPTURE_CH    6

//--------------------------------------------
void hal_rt_init(void);
uint32_t hal_rt_get_counter(void);
uint32_t hal_rt_get_address_time(void);
uint32_t hal_rt_get_end_time(void);
uint32_t hal_rt_get_rxready_time(void);
uint32_t hal_rt_get_disabled_time(void);
void hal_rt_set_compare_time(uint32_t time, uint8_t channel);
void hal_rt_disable_channel(uint8_t channel);

//--------------------------------------------
#define radio_tmr_get_radio_time hal_rt_get_counter
#define radio_tmr_start_us

#endif /* HAL_RT_H_ */