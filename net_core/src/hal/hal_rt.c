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

#include <stddef.h>    /* size_t, NULL */
#include <assert.h>    /* assert */
#include <stdbool.h>   /* bool */
#include <nrf.h>
#include "irq_priorities.h"
#include "dppi_channels.h"
#include "hal_rt.h"
#include "radio_packet.h"
#include "ble.h"

//--------------- Debugging ------------------
#define DEBUG_RADIO_TIMER_COUNTER_OVERFLOW  0

//--------------------------------------------
#if DEBUG_RADIO_TIMER_COUNTER_OVERFLOW
#define __debugbreak() __asm("bkpt 0")
//--------------------------------------------
static void hal_rt_init(void)
{
	NRF_TIMER0_NS->MODE = TIMER_MODE_MODE_Timer;
	NRF_TIMER0_NS->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
	NRF_TIMER0_NS->PRESCALER = 1;  // Input clock = 16 MHz

	NRF_TIMER0_NS->TASKS_START = 1;

	uint32_t cnt;
	while (1)
	{
		cnt = hal_rt_get_counter();
		if (cnt > 0xFE363C7F)
		{
			NRF_TIMER0_NS->TASKS_STOP = 1;
			break;
		}
	}
	__debugbreak();	

	cnt = hal_rt_get_counter();

#if RT_PER_US == 1
	NRF_TIMER0_NS->PRESCALER = 4;  // Input clock = 16 MHz / 2^4 = 1 MHz -> 1 us resolution
#elif RT_PER_US == 4
	NRF_TIMER0_NS->PRESCALER = 2;  // Input clock = 16 MHz / 2^2 = 4 MHz -> 250 ns resolution
#else
#error "Unsupported RT_PER_US value"
#endif

	// Enable DPPI events
	NRF_TIMER0_NS->PUBLISH_COMPARE[RADIO_RXEN_COMPARE_CH] = (RTC_PUBLISH_COMPARE_EN_Enabled << RTC_PUBLISH_COMPARE_EN_Pos) | DPPI_TIMER0_COMPARE_CH_TO_RADIO_RXEN;
	NRF_TIMER0_NS->PUBLISH_COMPARE[RADIO_DISABLE_COMPARE_CH] = (RTC_PUBLISH_COMPARE_EN_Enabled << RTC_PUBLISH_COMPARE_EN_Pos) | DPPI_TIMER0_COMPARE_CH_TO_RADIO_DISABLE;
	NRF_TIMER0_NS->SUBSCRIBE_CAPTURE[RADIO_ADDRESS_CAPTURE_CH] = (RTC_SUBSCRIBE_CAPTURE_EN_Enabled << RTC_SUBSCRIBE_CAPTURE_EN_Pos) | DPPI_RADIO_ADDRESS_TO_TIMER0_CAPTURE_CH;
	NRF_TIMER0_NS->SUBSCRIBE_CAPTURE[RADIO_END_CAPTURE_CH] = (RTC_SUBSCRIBE_CAPTURE_EN_Enabled << RTC_SUBSCRIBE_CAPTURE_EN_Pos) | DPPI_RADIO_END_TO_TIMER0_CAPTURE_CH;
	NRF_TIMER0_NS->SUBSCRIBE_CAPTURE[RADIO_DISABLE_COMPARE_CH] = (RTC_SUBSCRIBE_CAPTURE_EN_Enabled << RTC_SUBSCRIBE_CAPTURE_EN_Pos) | DPPI_RADIO_ADDRESS_TO_TIMER0_CAPTURE_CH;
	NRF_TIMER0_NS->SUBSCRIBE_CAPTURE[RADIO_RXREADY_CAPTURE_CH] = (RTC_SUBSCRIBE_CAPTURE_EN_Enabled << RTC_SUBSCRIBE_CAPTURE_EN_Pos) | DPPI_RADIO_RXREADY_TO_TIMER0_CAPTURE_CH;
	NRF_TIMER0_NS->SUBSCRIBE_CAPTURE[RADIO_DISABLED_CAPTURE_CH] = (RTC_SUBSCRIBE_CAPTURE_EN_Enabled << RTC_SUBSCRIBE_CAPTURE_EN_Pos) | DPPI_RADIO_DISABLED_TO_TIMER0_CAPTURE_CH;

	// Enable one short mode
	NRF_TIMER0_NS->ONESHOTEN[RADIO_RXEN_COMPARE_CH] = 1;
	NRF_TIMER0_NS->ONESHOTEN[RADIO_DISABLE_COMPARE_CH] = 1;

	// Start TIMER0
	NRF_TIMER0_NS->TASKS_START = 1;
}

#else
//--------------------------------------------
void hal_rt_init(void)
{
	// Set up TIMER0
	NRF_TIMER0_NS->MODE = TIMER_MODE_MODE_Timer;
	NRF_TIMER0_NS->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
#if RT_PER_US == 1
	NRF_TIMER0_NS->PRESCALER = 4;  // Input clock = 16 MHz / 2^4 = 1 MHz -> 1 us resolution
#elif RT_PER_US == 4
	NRF_TIMER0_NS->PRESCALER = 2;  // Input clock = 16 MHz / 2^2 = 4 MHz -> 250 ns resolution
#else
#error "Unsupported RT_PER_US value"
#endif

	// Enable DPPI events
	NRF_TIMER0_NS->PUBLISH_COMPARE[RADIO_RXEN_COMPARE_CH] = (RTC_PUBLISH_COMPARE_EN_Enabled << RTC_PUBLISH_COMPARE_EN_Pos) | DPPI_TIMER0_COMPARE_CH_TO_RADIO_RXEN;
	NRF_TIMER0_NS->PUBLISH_COMPARE[RADIO_DISABLE_COMPARE_CH] = (RTC_PUBLISH_COMPARE_EN_Enabled << RTC_PUBLISH_COMPARE_EN_Pos) | DPPI_TIMER0_COMPARE_CH_TO_RADIO_DISABLE;
	NRF_TIMER0_NS->SUBSCRIBE_CAPTURE[RADIO_ADDRESS_CAPTURE_CH] = (RTC_SUBSCRIBE_CAPTURE_EN_Enabled << RTC_SUBSCRIBE_CAPTURE_EN_Pos) | DPPI_RADIO_ADDRESS_TO_TIMER0_CAPTURE_CH;
	NRF_TIMER0_NS->SUBSCRIBE_CAPTURE[RADIO_END_CAPTURE_CH] = (RTC_SUBSCRIBE_CAPTURE_EN_Enabled << RTC_SUBSCRIBE_CAPTURE_EN_Pos) | DPPI_RADIO_END_TO_TIMER0_CAPTURE_CH;
	NRF_TIMER0_NS->SUBSCRIBE_CAPTURE[RADIO_DISABLE_COMPARE_CH] = (RTC_SUBSCRIBE_CAPTURE_EN_Enabled << RTC_SUBSCRIBE_CAPTURE_EN_Pos) | DPPI_RADIO_ADDRESS_TO_TIMER0_CAPTURE_CH;
	NRF_TIMER0_NS->SUBSCRIBE_CAPTURE[RADIO_RXREADY_CAPTURE_CH] = (RTC_SUBSCRIBE_CAPTURE_EN_Enabled << RTC_SUBSCRIBE_CAPTURE_EN_Pos) | DPPI_RADIO_RXREADY_TO_TIMER0_CAPTURE_CH;
	NRF_TIMER0_NS->SUBSCRIBE_CAPTURE[RADIO_DISABLED_CAPTURE_CH] = (RTC_SUBSCRIBE_CAPTURE_EN_Enabled << RTC_SUBSCRIBE_CAPTURE_EN_Pos) | DPPI_RADIO_DISABLED_TO_TIMER0_CAPTURE_CH;

	// Enable one short mode
	NRF_TIMER0_NS->ONESHOTEN[RADIO_RXEN_COMPARE_CH] = 1;
	NRF_TIMER0_NS->ONESHOTEN[RADIO_DISABLE_COMPARE_CH] = 1;

	// Start TIMER0
	NRF_TIMER0_NS->TASKS_CLEAR = 1;
	NRF_TIMER0_NS->TASKS_START = 1;
}
#endif

//--------------------------------------------
uint32_t hal_rt_get_counter(void)
{
	NRF_TIMER0_NS->TASKS_CAPTURE[GET_COUNTER_CH] = 1;
	return NRF_TIMER0_NS->CC[GET_COUNTER_CH];
}

//--------------------------------------------
uint32_t hal_rt_get_address_time(void)
{
	return NRF_TIMER0_NS->CC[RADIO_ADDRESS_CAPTURE_CH];
}

//--------------------------------------------
uint32_t hal_rt_get_end_time(void)
{
	return NRF_TIMER0_NS->CC[RADIO_END_CAPTURE_CH];
}

//--------------------------------------------
uint32_t hal_rt_get_rxready_time(void)
{
	return NRF_TIMER0_NS->CC[RADIO_RXREADY_CAPTURE_CH];
}

//--------------------------------------------
uint32_t hal_rt_get_disabled_time(void)
{
	return NRF_TIMER0_NS->CC[RADIO_DISABLED_CAPTURE_CH];
}

//--------------------------------------------
void hal_rt_set_compare_time(uint32_t time, uint8_t channel)
{
	if (channel == RADIO_RXEN_COMPARE_CH)
	{
		NRF_TIMER0_NS->EVENTS_COMPARE[RADIO_RXEN_COMPARE_CH] = 0;
		NRF_TIMER0_NS->PUBLISH_COMPARE[RADIO_RXEN_COMPARE_CH] = (RTC_PUBLISH_COMPARE_EN_Enabled << RTC_PUBLISH_COMPARE_EN_Pos) | DPPI_TIMER0_COMPARE_CH_TO_RADIO_RXEN;
	}
	if (channel == RADIO_DISABLE_COMPARE_CH)
	{
		NRF_TIMER0_NS->EVENTS_COMPARE[RADIO_DISABLE_COMPARE_CH] = 0;
		NRF_TIMER0_NS->PUBLISH_COMPARE[RADIO_DISABLE_COMPARE_CH] = (RTC_PUBLISH_COMPARE_EN_Enabled << RTC_PUBLISH_COMPARE_EN_Pos) | DPPI_TIMER0_COMPARE_CH_TO_RADIO_DISABLE;
	}
	NRF_TIMER0_NS->CC[channel] = time;
}

//--------------------------------------------
void hal_rt_disable_channel(uint8_t channel)
{
	NRF_TIMER0_NS->CC[channel] = 0;
	if (channel == RADIO_RXEN_COMPARE_CH)
	{
		NRF_TIMER0_NS->PUBLISH_COMPARE[RADIO_RXEN_COMPARE_CH] = 0;
	}
	if (channel == RADIO_DISABLE_COMPARE_CH)
	{
		NRF_TIMER0_NS->PUBLISH_COMPARE[RADIO_DISABLE_COMPARE_CH] = 0;
	}
}
