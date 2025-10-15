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

#include <stdint.h>     /* uint8_t ... uint64_t */
#include <stdlib.h>     /* size_t */
#include <stdbool.h>    /* bool */
#include "radio_packet.h"
#include "ble.h"

//--------------------------------------------
uint32_t ble_get_access_address_transmission_time_us(ble_phy_t phy)
{
	switch (phy)
	{
	case BLE_PHY_1M:
		// 1 Mbit/s
		return ((1 + ACCESS_ADDRESS_LENGTH) * 8);
	case BLE_PHY_2M:
		// 2 Mbit/s
		return ((2 + ACCESS_ADDRESS_LENGTH) * 8 / 2);
	case BLE_PHY_CODED_S2:
		// S = 2, 500 kbit/s
	case BLE_PHY_CODED_S8:
		// S = 8, 125 kbit/s
		return (80 + 256 + 16 + 24);
	}
	return 0;
}

//--------------------------------------------
uint32_t ble_get_access_address_transmission_time_rt(ble_phy_t phy)
{
	return ble_get_access_address_transmission_time_us(phy) * RT_PER_US;
}

//--------------------------------------------
uint32_t ble_get_packet_transmission_time_us(ble_phy_t phy, size_t size)
{
	if (!size)
	{
		return 0;
	}
	switch (phy)
	{
	case BLE_PHY_1M:
		// 1 Mbit/s
		return ((1 + size) * 8);
	case BLE_PHY_2M:
		// 2 Mbit/s
		return ((2 + size) * 8 / 2);
	case BLE_PHY_CODED_S8:
		// S = 8, 125 kbit/s
		return (80 + 256 + 16 + 24 + ((size - ACCESS_ADDRESS_LENGTH) * 8 + 3) * CODED_PHY_CODING_SCHEME_S8);
	case BLE_PHY_CODED_S2:
		// S = 2, 500 kbit/s
		return (80 + 256 + 16 + 24 + ((size - ACCESS_ADDRESS_LENGTH) * 8 + 3) * CODED_PHY_CODING_SCHEME_S2);
	}
	return 0;
}

//--------------------------------------------
uint32_t ble_get_packet_transmission_time_rt(ble_phy_t phy, size_t size)
{
	return ble_get_packet_transmission_time_us(phy, size) * RT_PER_US;
}

//--------------------------------------------
uint32_t ble_get_transmit_window_delay_us(ble_phy_t phy, uint8_t radio_channel)
{
	// BLUETOOTH CORE SPECIFICATION Version 6.0 | Vol 6, Part B
	// 4.5.3 Connection event transmit window
	// Page 3097
	if (radio_channel >= 37)
	{
		// CONNECT_IND
		return 1250;
	}
	else
	{
		// AUX_CONNECT_REQ
		switch (phy)
		{
		case BLE_PHY_1M:
		case BLE_PHY_2M:
			return 2500;
		case BLE_PHY_CODED_S8:
		case BLE_PHY_CODED_S2:
			return 3750;
		}
	}
	return 0;
}

//--------------------------------------------
uint32_t ble_get_transmit_window_delay_rt(ble_phy_t phy, uint8_t radio_channel)
{
	return ble_get_transmit_window_delay_us(phy, radio_channel);
}

//--------------------------------------------
bool ble_bit_number_to_phy(uint8_t bit_number, ble_phy_t *phy)
{
	switch (bit_number)
	{
	case 0x1:
		*phy = BLE_PHY_1M;
		break;
	case 0x2:
		*phy = BLE_PHY_2M;
		break;
	case 0x4:
		*phy = BLE_PHY_CODED_S8;
		break;
	default:
		return false;
	}
	return true;
}

//--------------------------------------------
bool ble_value_to_phy(uint8_t value, ble_phy_t *phy)
{
	switch (value & 0x7)
	{
	case 0x0:
		*phy = BLE_PHY_1M;
		break;
	case 0x1:
		*phy = BLE_PHY_2M;
		break;
	case 0x2:
		*phy = BLE_PHY_CODED_S8;
		break;
	default:
		return false;
	}
	return true;
}

//--------------------------------------------
static uint8_t count_set_bits(uint8_t byte)
{
	uint8_t count = 0;
	while (byte)
	{
		count += byte & 1;
		byte >>= 1;
	}
	return count;
}

//--------------------------------------------
uint8_t count_used_channels(uint8_t *channel_map)
{
	uint8_t total_used_channels = 0;

	for (size_t cnt = 0; cnt < 5; cnt++)
	{
		total_used_channels += count_set_bits(channel_map[cnt]);
	}

	return total_used_channels;
}

//--------------------------------------------
uint8_t ble_channel_to_radio(uint8_t ble_channel)
{
	if (ble_channel > 39)
	{
		return 0xFF;
	}
	if (ble_channel == 37)
	{
		return 2;
	}
	if (ble_channel == 38)
	{
		return 26;
	}
	if (ble_channel == 39)
	{
		return 80;
	}
	if (ble_channel <= 10)
	{
		return (ble_channel << 1) + 4;
	}
	return (ble_channel << 1) + 6;
}

//--------------------------------------------
uint32_t ble_bis_aa_from_seed_aa_and_bis_number(uint32_t seed_aa, uint8_t n)
{
	uint8_t d;
	uint32_t dw;
	uint32_t bis_aa;

	// 2.1.2 Access Address
	// The Access Address for each BIS and for the BIG Control logical link (see
	// Section 4.4.6.7) in a BIG shall be derived from the SAA for that BIG.
	// For each BIS logical transport, the Access Address shall be equal to the SAA bit-wise
	// XORed with a diversifier word(DW) for that logical transport derived from a Diversifier(D)
	// as follows:

	// D = ((35 x n) + 42) mod 128 where n is the BIS number, or 0 for the BIG Control logical link.
	d = ((35 * n) + 42) & 0x7F;

	// DW = 0bD0 D0 D0 D0 D0 D0 D1 D6 _ D1 0 D5 D4 0 D3 D2 0_00000000_00000000
	// D0
	if (d & 0x01)
	{
		dw = 0xFC000000;
	}
	else
	{
		dw = 0x00000000;
	}
	// D1, D6, D1 D5 
	// D4, D3, D2
	dw |= ((d & 0x02) >> 1 << 25) | ((d & 0x40) >> 6 << 24) | ((d & 0x02) >> 1 << 23) | ((d & 0x20) >> 5 << 21)
		| ((d & 0x10) >> 4 << 20) | ((d & 0x08) >> 3 << 18) | ((d & 0x04) >> 2 << 17);

	bis_aa = seed_aa ^ dw;

	return bis_aa;
}
