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

#include <nrf.h>
#include "irq_priorities.h"

//--------------------------------------------
static volatile uint32_t counter = 0;

//--------------------------------------------
void SysTick_Handler(void)
{
	counter++;
}

//--------------------------------------------
void delay_ms(uint32_t delay_ms)
{
	uint32_t start;
	start = counter;
	while ((counter - start) < delay_ms);
}

//--------------------------------------------
uint32_t get_platform_counter(void)
{
	return counter;
}

//--------------------------------------------
void delay_us(uint32_t delay_us)
{
	uint32_t start;
	start = DWT->CYCCNT;
	delay_us *= (SystemCoreClock / 1000000);
	while ((DWT->CYCCNT - start) < delay_us);
}

//--------------------------------------------
static void SetSysClock(void)
{
#ifdef NRF5340_XXAA_APPLICATION
	// Start HFCLK64M source
	NRF_CLOCK_S->EVENTS_HFCLKSTARTED = 0;
	NRF_CLOCK_S->TASKS_HFCLKSTART = 1;

	// Wait for HFCLK64M source to start up
	while (!NRF_CLOCK_S->EVENTS_HFCLKSTARTED);

#ifdef APP_CORE_128MHZ
	NRF_CLOCK_S->HFCLKCTRL = 0;
#endif

#else
	// Start HFCLK64M source
	NRF_CLOCK_NS->EVENTS_HFCLKSTARTED = 0;
	NRF_CLOCK_NS->TASKS_HFCLKSTART = 1;

	// Wait for HFCLK64M source to start up
	while (!NRF_CLOCK_NS->EVENTS_HFCLKSTARTED);

#endif
}

//--------------------------------------------
static void DWTInit(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; 
}

//--------------------------------------------
void platform_init(void)
{
	SetSysClock();
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock / 1000);
	NVIC_SetPriority(SysTick_IRQn, SYSTICK_IRQ_PRIORITY);
	DWTInit();
}
