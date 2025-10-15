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

#ifndef HAL_CCM_H_
#define HAL_CCM_H_

//--------------------------------------------
#define MAX_PACKET_SIZE     251

//--------------------------------------------
typedef void (*ccm_irq_callback_t)(bool mic_status);

//--------------------------------------------
#pragma pack(push,1)
typedef struct
{
	uint8_t key[16];
	uint8_t pktctr[9];
	uint8_t iv[8];
} ccm_cnf_t;
#pragma pack(pop)

//--------------------------------------------
void hal_ccm_set_key(uint8_t *key);
void hal_ccm_set_pktctr(uint64_t pktctr, uint8_t dir);
void hal_ccm_set_iv_c(uint8_t *iv);
void hal_ccm_set_iv_p(uint8_t *iv);
void hal_ccm_init(void);
void hal_ccm_set_irq_callback(ccm_irq_callback_t callback);
void hal_ccm_decrypt(uint8_t *enc_buf, uint8_t *dec_buf);

#endif /* HAL_CCM_H_ */