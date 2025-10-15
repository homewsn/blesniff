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

#ifndef CRYPTO_H_
#define CRYPTO_H_

//--------------------------------------------
typedef struct
{
	uint8_t ltk[16];
	uint8_t session_key[16];
	uint64_t c2p_packet_counter;
	uint64_t p2c_packet_counter;
	uint8_t skd_c[8];
	uint8_t skd_p[8];
	uint8_t iv_c[4];
	uint8_t iv_p[4];
} ble_crypto_t;

//--------------------------------------------
void crypto_set_ltk(uint8_t *buf);
void crypto_set_skd_iv_c(uint8_t *buf);
void crypto_set_skd_iv_p(uint8_t *buf);
bool crypto_generate_session_key(void);
void crypto_reset_encrypted_packet_counters(void);
void crypto_decrypt(void);

#endif /* CRYPTO_H_ */