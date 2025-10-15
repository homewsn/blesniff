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

#include <stdint.h>    /* uint8_t ... uint64_t */
#include <stddef.h>    /* size_t, NULL */
#include <stdbool.h>   /* bool */
#include <string.h>    /* memcpy */
#include <assert.h>    /* assert */
#include "hal_ecb.h"
#include "hal_ccm.h"
#include "crypto.h"
#include "radio_packet.h"
#include "ble.h"
#include "msg_queue_rx2decrypt.h"

//--------------------------------------------
static ble_crypto_t ble_crypto;
static uint8_t enc_buf[MAX_PACKET_SIZE];
static uint8_t dec_buf[MAX_PACKET_SIZE];
static bool decryption_in_progress;

void acl_packet_processing_from_queue(void);
//--------------------------------------------
static void crypto_irq_callback(bool mic_status)
{
	rx2decrypt_item_t *item;
	decryption_in_progress = false;

	assert(msg_queue_rx2decrypt_get_first_encrypted_item_ptr(&item));
	item->need_to_decrypt = false;
	item->mic_status = mic_status;

	if (mic_status)
	{
		item->data[1] -= 4;
		memcpy(item->data + 2, dec_buf + 3, item->data[1]);
	}
	acl_packet_processing_from_queue();
}

//--------------------------------------------
void crypto_set_ltk(uint8_t *buf)
{
	memcpy(ble_crypto.ltk, buf, 16);
	hal_ccm_init();
	hal_ccm_set_irq_callback(crypto_irq_callback);
}

//--------------------------------------------
void crypto_set_skd_iv_c(uint8_t *buf)
{
	for (size_t cnt = 0; cnt < 8; cnt++)
	{
		ble_crypto.skd_c[cnt] = buf[7 - cnt];
	}
	memcpy(ble_crypto.iv_c, buf + 8, 4);
	hal_ccm_set_iv_c(ble_crypto.iv_c);
}

//--------------------------------------------
void crypto_set_skd_iv_p(uint8_t *buf)
{
	for (size_t cnt = 0; cnt < 8; cnt++)
	{
		ble_crypto.skd_p[cnt] = buf[7 - cnt];
	}
	memcpy(ble_crypto.iv_p, buf + 8, 4);
	hal_ccm_set_iv_p(ble_crypto.iv_p);
}

//--------------------------------------------
bool crypto_generate_session_key(void)
{
	ecb_data_t ecb_data;

	memcpy(ecb_data.key, ble_crypto.ltk, 16);
	memcpy(ecb_data.cleartext, ble_crypto.skd_p, 8);
	memcpy(ecb_data.cleartext + 8, ble_crypto.skd_c, 8);

	for (size_t cnt = 0; cnt < 100; cnt++)
	{
		if (hal_ecb_encrypt(&ecb_data))
		{
			memcpy(ble_crypto.session_key, ecb_data.ciphertext, 16);
			hal_ccm_set_key(ble_crypto.session_key);
			return true;
		}
	}
	return false;
}

//--------------------------------------------
void crypto_reset_encrypted_packet_counters(void)
{
	ble_crypto.c2p_packet_counter = 0;
	ble_crypto.p2c_packet_counter = 0;
}

//--------------------------------------------
void crypto_decrypt(void)
{
	rx2decrypt_item_t *item;

	if (decryption_in_progress)
	{
		return;
	}
	if (!msg_queue_rx2decrypt_get_first_encrypted_item_ptr(&item))
	{
		return;
	}
	if (item->data[0] & CP_MASK)
	{
		memcpy(enc_buf, item->data, item->data[1] + 3);
	}
	else
	{
		enc_buf[0] = item->data[0];
		enc_buf[1] = item->data[1];
		enc_buf[2] = 0;
		memcpy(enc_buf + 3, item->data + 2, item->data[1]);
	}
	if (!item->direction)
	{
		hal_ccm_set_pktctr(ble_crypto.c2p_packet_counter, item->direction);
		ble_crypto.c2p_packet_counter++;
	}
	else
	{
		hal_ccm_set_pktctr(ble_crypto.p2c_packet_counter, item->direction);
		ble_crypto.p2c_packet_counter++;
	}
	item->mic_status = 0;
	decryption_in_progress = true;
	hal_ccm_decrypt(enc_buf, dec_buf);
}
