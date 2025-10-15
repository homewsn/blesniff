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
#include "dppi_channels.h"


//--------------------------------------------
void hal_dppi_init(void)
{
	NRF_DPPIC_NS->CHENSET = (1UL << DPPI_TIMER0_COMPARE_CH_TO_RADIO_RXEN);
	NRF_DPPIC_NS->CHENSET = (1UL << DPPI_TIMER0_COMPARE_CH_TO_RADIO_DISABLE);
	NRF_DPPIC_NS->CHENSET = (1UL << DPPI_RADIO_ADDRESS_TO_TIMER0_CAPTURE_CH);
	NRF_DPPIC_NS->CHENSET = (1UL << DPPI_RADIO_END_TO_TIMER0_CAPTURE_CH);
	NRF_DPPIC_NS->CHENSET = (1UL << DPPI_RADIO_RXREADY_TO_TIMER0_CAPTURE_CH);
	NRF_DPPIC_NS->CHENSET = (1UL << DPPI_RADIO_DISABLED_TO_TIMER0_CAPTURE_CH);
}
