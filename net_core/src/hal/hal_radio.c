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
#include <stdbool.h>   /* bool */
#include <assert.h>    /* assert */
#include <math.h>      /* round, pow */
#include <nrf.h>
#include <nrf53_erratas.h>
#include "irq_priorities.h"
#include "dppi_channels.h"
#include "radio_packet.h"
#include "hal_radio.h"
#include "hal_rt.h"
#include "ble.h"

//--------------------------------------------
#if 0
#define __debugbreak() __asm("bkpt 0")
#else
#define __debugbreak()
#endif

//--------------------------------------------
static _Alignas(32) uint8_t rx_buffer[BLE_PDU_MAX_SIZE];
static radio_irq_callback_t irq_callback;
static bool started;

//--------------------------------------------
void hal_radio_init(void)
{
	// Off and On RADIO
	NRF_RADIO_NS->POWER = 0;
	NRF_RADIO_NS->POWER = 1;

	if (nrf53_errata_158())
	{
		// Copy all the RADIO trim values from FICR into the target addresses
		for (uint32_t index = 0; index < 32ul && NRF_FICR_NS->TRIMCNF[index].ADDR != 0xFFFFFFFFul; index++)
		{
			#if defined ( __ICCARM__ )
			#pragma diag_suppress=Pa082
			#pragma diag_suppress=Pe191
			#endif
			if (((uint32_t)NRF_FICR_NS->TRIMCNF[index].ADDR & 0xFFFFF000ul) == (volatile uint32_t)NRF_RADIO_NS)
			{
				*((volatile uint32_t *)NRF_FICR_NS->TRIMCNF[index].ADDR) = NRF_FICR_NS->TRIMCNF[index].DATA;
			}
			#if defined ( __ICCARM__ )
			#pragma diag_default=Pa082
			#pragma diag_default=Pe191
			#endif
		}
	}

#if HAL_RADIO_FAST_RUMP_UP
	// Fast ramp-up (tRXEN,FAST and tTXEN,FAST)
	NRF_RADIO_NS->MODECNF0 |= (RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos);
#endif

	// General BLE packet settings independent of PHY
	NRF_RADIO_NS->PCNF0 = (8 << RADIO_PCNF0_LFLEN_Pos)   // Length on air of LENGTH field in number of bits = 8
		| (1 << RADIO_PCNF0_S0LEN_Pos)                   // Length on air of S0 field in number of bytes = 1 (PDU header)
		| (0 << RADIO_PCNF0_S1LEN_Pos)                   // Length on air of S1 field in number of bits = 0 (No S1 field)
		| (0 << RADIO_PCNF0_CRCINC_Pos);                 // LENGTH does not contain CRC

	NRF_RADIO_NS->PCNF1 = (BLE_PDU_MAX_SIZE << RADIO_PCNF1_MAXLEN_Pos)  // Maximum length of packet payload = 255
		| (0 << RADIO_PCNF1_STATLEN_Pos)                                // Static length in number of bytes = 0
		| (3 << RADIO_PCNF1_BALEN_Pos)                                  // Base address length in bytes = 4 - 1 = 3
		| (RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos)         // On-air endianness of packet = Little
		| (RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos);     // Enable or disable packet whitening = Enable

	// Setting up CRC (polynomial fixed for BLE)
	NRF_RADIO_NS->CRCCNF = (RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos)  // CRC length in number of bytes = 3
		| (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos);         // Include or exclude packet address field out of CRC calculation = Exclude
	NRF_RADIO_NS->CRCPOLY = 0x65B;

	// Packet pointer
	NRF_RADIO_NS->PACKETPTR = (uint32_t)rx_buffer;

	// Enable interrupt on DISABLED state
	NRF_RADIO_NS->INTENSET = RADIO_INTENSET_DISABLED_Msk;

	// Enable DPPI tasks
	NRF_RADIO_NS->SUBSCRIBE_RXEN = (RADIO_SUBSCRIBE_RXEN_EN_Enabled << RADIO_SUBSCRIBE_RXEN_EN_Pos) | DPPI_TIMER0_COMPARE_CH_TO_RADIO_RXEN;
	NRF_RADIO_NS->SUBSCRIBE_DISABLE = (RADIO_SUBSCRIBE_DISABLE_EN_Enabled << RADIO_SUBSCRIBE_DISABLE_EN_Pos) | DPPI_TIMER0_COMPARE_CH_TO_RADIO_DISABLE;
	NRF_RADIO_NS->PUBLISH_ADDRESS = (RADIO_PUBLISH_ADDRESS_EN_Enabled << RADIO_PUBLISH_ADDRESS_EN_Pos) | DPPI_RADIO_ADDRESS_TO_TIMER0_CAPTURE_CH;
	NRF_RADIO_NS->PUBLISH_END = (RADIO_PUBLISH_END_EN_Enabled << RADIO_PUBLISH_END_EN_Pos) | DPPI_RADIO_END_TO_TIMER0_CAPTURE_CH;
	NRF_RADIO_NS->PUBLISH_RXREADY = (RADIO_PUBLISH_RXREADY_EN_Enabled << RADIO_PUBLISH_RXREADY_EN_Pos) | DPPI_RADIO_RXREADY_TO_TIMER0_CAPTURE_CH;
	NRF_RADIO_NS->PUBLISH_DISABLED = (RADIO_PUBLISH_DISABLED_EN_Enabled << RADIO_PUBLISH_DISABLED_EN_Pos) | DPPI_RADIO_DISABLED_TO_TIMER0_CAPTURE_CH;

	// Enable shortcuts
	NRF_RADIO_NS->SHORTS = RADIO_SHORTS_READY_START_Msk
		| RADIO_SHORTS_END_DISABLE_Msk
		| RADIO_SHORTS_ADDRESS_RSSISTART_Msk
		| RADIO_SHORTS_DISABLED_RSSISTOP_Msk;

	// Enable RADIO interrupt
	NVIC_SetPriority(RADIO_IRQn, RADIO_IRQ_PRIORITY);
	NVIC_EnableIRQ(RADIO_IRQn);
}

//--------------------------------------------
void hal_radio_set_params(radio_params_t *params)
{
	if (params->channel > 39)
	{
		__debugbreak();
		return;
	}

	// Setting the PHY mode and corresponding packet parameters
	uint32_t pcnf0 = NRF_RADIO_NS->PCNF0 & ~((RADIO_PCNF0_PLEN_Msk) | (RADIO_PCNF0_CILEN_Msk) | (RADIO_PCNF0_TERMLEN_Msk));

	switch (params->phy)
	{
	case BLE_PHY_1M:
		NRF_RADIO_NS->MODE = RADIO_MODE_MODE_Ble_1Mbit;
		if (nrf53_errata_117())
		{
			*((volatile uint32_t *)0x41008588) = *((volatile uint32_t *)0x01FF0080);
		}
		pcnf0 |= (0 << RADIO_PCNF0_PLEN_Pos)    // 8-bit preamble
			| (0 << RADIO_PCNF0_CILEN_Pos)      // No CI field
			| (0 << RADIO_PCNF0_TERMLEN_Pos);   // No TERM1 field
		break;
	case BLE_PHY_2M:
		NRF_RADIO_NS->MODE = RADIO_MODE_MODE_Ble_2Mbit;
		if (nrf53_errata_117())
		{
			*((volatile uint32_t *)0x41008588) = *((volatile uint32_t *)0x01FF0084);
		}
		pcnf0 |= (2 << RADIO_PCNF0_PLEN_Pos)    // 16-bit preamble
			| (0 << RADIO_PCNF0_CILEN_Pos)      // No CI field
			| (0 << RADIO_PCNF0_TERMLEN_Pos);   // No TERM1 field
		break;
	case BLE_PHY_CODED_S8:
		NRF_RADIO_NS->MODE = RADIO_MODE_MODE_Ble_LR125Kbit;
		if (nrf53_errata_117())
		{
			*((volatile uint32_t *)0x41008588) = *((volatile uint32_t *)0x01FF0080);
		}
		pcnf0 |= (RADIO_PCNF0_PLEN_LongRange << RADIO_PCNF0_PLEN_Pos) // Preamble - used for Bluetooth LE Long Range
			| (2 << RADIO_PCNF0_CILEN_Pos)                            // 2-bit Coding Indicator (CI)
			| (3 << RADIO_PCNF0_TERMLEN_Pos);                         // 3-bit TERM1 field
		break;
	case BLE_PHY_CODED_S2:
		NRF_RADIO_NS->MODE = RADIO_MODE_MODE_Ble_LR500Kbit;
		if (nrf53_errata_117())
		{
	        *((volatile uint32_t *)0x41008588) = *((volatile uint32_t *)0x01FF0080);
		}
		pcnf0 |= (RADIO_PCNF0_PLEN_LongRange << RADIO_PCNF0_PLEN_Pos) // Preamble - used for Bluetooth LE Long Range
			| (2 << RADIO_PCNF0_CILEN_Pos)                            // 2-bit Coding Indicator (CI)
			| (3 << RADIO_PCNF0_TERMLEN_Pos);                         // 3-bit TERM1 field
		break;
    }

	NRF_RADIO_NS->PCNF0 = pcnf0;

	// Enable reception on logical address 0
	NRF_RADIO_NS->PREFIX0 = params->address >> 24;
	NRF_RADIO_NS->BASE0 = params->address << 8;
	NRF_RADIO_NS->RXADDRESSES = ((RADIO_RXADDRESSES_ADDR0_Enabled) << RADIO_RXADDRESSES_ADDR0_Pos);

	// Setting up CRC init
	NRF_RADIO_NS->CRCINIT = params->crc_init;

	// Frequency channel setting
	NRF_RADIO_NS->FREQUENCY = ble_channel_to_radio(params->channel);
	NRF_RADIO_NS->DATAWHITEIV = params->channel;  // 0x40 is added automatically
}

//--------------------------------------------
void hal_radio_stop(void)
{
	NRF_RADIO_NS->TASKS_DISABLE = 1;
#if 1
	while (NRF_RADIO_NS->STATE != RADIO_STATE_STATE_Disabled);
#else
	size_t delay = 1000000;
	for (delay = 1000000;; delay--)
	{
		if (NRF_RADIO_NS->STATE == RADIO_STATE_STATE_Disabled)
		{
			break;
		}
	}
	if (!delay)
	{
		NRF_RADIO_NS->TASKS_DISABLE = 1;
		while (NRF_RADIO_NS->STATE != RADIO_STATE_STATE_Disabled);
	}
#endif
	started = false;
}

//--------------------------------------------
void hal_radio_start(void)
{
	NRF_RADIO_NS->TASKS_RXEN = 1;
	started = true;
}

//--------------------------------------------
void hal_radio_set_irq_callback(radio_irq_callback_t callback)
{
	irq_callback = callback;
}

//--------------------------------------------
static uint8_t nrf53_errata_87_workaround(uint8_t rssi_sample)
{
#if 0
	static volatile uint32_t time1;
	static volatile uint32_t time2;
	static volatile uint32_t time3;
	time1 = radio_tmr_get_radio_time();
#endif
	NRF_TEMP_NS->EVENTS_DATARDY = 0;
	NRF_TEMP_NS->TASKS_START = 1;
	while (!NRF_TEMP_NS->EVENTS_DATARDY);
	// => +37us
#if 0
	time2 = radio_tmr_get_radio_time();
#endif
	int32_t temp = NRF_TEMP_NS->TEMP;
	uint8_t rssi = (uint8_t)roundf(((1.56f * rssi_sample) +
		(4.9e-5f * rssi_sample * rssi_sample * rssi_sample) -
		(9.9e-3f * rssi_sample * rssi_sample) -
		(0.05f * (temp * 0.25f)) - 7.2f));
	// IAR => +23us, GCC => +35us
#if 0
	time3 = radio_tmr_get_radio_time();
#endif
	return rssi;
}

//--------------------------------------------
int8_t hal_radio_get_rssi(void)
{
	uint8_t rssi;
	rssi = (uint8_t)NRF_RADIO_NS->RSSISAMPLE;
#if 1
	if (nrf53_errata_87())
	{
		rssi = nrf53_errata_87_workaround(rssi);
	}
#endif
	return (int8_t)(-rssi);
}

//--------------------------------------------
bool hal_radio_crc_is_valid(void)
{
	return (NRF_RADIO_NS->CRCSTATUS != 0);
}

//--------------------------------------------
bool hal_radio_is_started(void)
{
	return started;
}

//--------------------------------------------
void RADIO_IRQHandler(void)
{
	bool rx_done = (NRF_RADIO_NS->EVENTS_END != 0);
	started = false;

	// Only EVENTS_* registers read (checked) by software needs reset between Radio IRQs.
	// In PPI use, irrespective of stored EVENT_* register value, PPI task will be triggered.
	// Hence, other EVENT_* registers are not reset to save code and CPU time.
	NRF_RADIO_NS->EVENTS_READY = 0;
	NRF_RADIO_NS->EVENTS_ADDRESS = 0;
	NRF_RADIO_NS->EVENTS_END = 0;
	NRF_RADIO_NS->EVENTS_DISABLED = 0;
	NRF_RADIO_NS->EVENTS_RSSIEND = 0;

	if (irq_callback)
	{
		irq_callback(rx_buffer, rx_done);
	}
}
