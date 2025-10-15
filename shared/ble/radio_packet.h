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

#ifndef RADIO_PACKET_H_
#define RADIO_PACKET_H_

//--------------------------------------------
#define BLE_PDU_MAX_SIZE       255

//--------------------------------------------
typedef enum
{
	BLE_PHY_1M,
	BLE_PHY_2M,
	BLE_PHY_CODED_S8,
	BLE_PHY_CODED_S2
} ble_phy_t;

//--------------------------------------------
typedef enum
{
	BLE_DIR_C2P,
	BLE_DIR_P2C
} pkt_dir_t;

//--------------------------------------------
typedef enum
{
	ENC_UNENCRYPTED,
	ENC_ENCRYPTED,
	ENC_DECRYPTED
} pkt_enc_t;

//--------------------------------------------
typedef enum
{
	MIC_UNKNOWN,
	MIC_VALID
} pkt_mic_t;

//--------------------------------------------
typedef enum
{
	CRC_FAIL,
	CRC_VALID
} pkt_crc_t;

//--------------------------------------------
typedef enum
{
	PDU_ADV,
	PDU_AUX,
	PDU_ACL,
	PDU_ISO_CIG,
	PDU_ISO_BIG
} pkt_pdu_t;

//--------------------------------------------
typedef enum
{
	PKT_AUX_ADV_IND,
	PKT_AUX_CHAIN_IND,
	PKT_AUX_SYNC_IND,
	PKT_AUX_SCAN_RSP,
	PKT_AUX_SYNC_SUBEVENT_IND,
	PKT_AUX_SYNC_SUBEVENT_RSP
} pkt_aux_t;

//--------------------------------------------
typedef struct
{
	ble_phy_t phy;
	uint8_t channel;
	uint32_t address;
	uint32_t crc_init;
} radio_params_t;

//--------------------------------------------
#pragma pack(1)
typedef struct
{
	uint32_t timestamp;
	uint16_t event_counter;
	uint8_t channel;
	int8_t rssi;
	uint8_t phy : 2;
	uint8_t direction : 2;
	uint8_t enc_status : 2;
	uint8_t mic_status : 1;
	uint8_t crc_status : 1;
	uint8_t pdu_type : 4;
	uint8_t aux_pdu_type : 4;
	uint32_t address;
	uint32_t crc_init;
	uint16_t size;
	uint8_t data[BLE_PDU_MAX_SIZE];
} radio_packet_t;
#pragma pack()

#endif /* RADIO_PACKET_H_ */

