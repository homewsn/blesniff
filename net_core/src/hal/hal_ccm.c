/*
* Copyright (c) 2025, 2026 Vladimir Alemasov
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

#include <stdbool.h>   /* bool */
#include <string.h>    /* memcpy */
#include <nrf.h>
#include "irq_priorities.h"
#include "hal_ccm.h"


//--------------------------------------------
#define SCRATCH_DATA_SIZE   (MAX_PACKET_SIZE + 16)

//--------------------------------------------
static ccm_cnf_t ccm_cnf;
static uint8_t scratch_data[SCRATCH_DATA_SIZE];
static ccm_irq_callback_t irq_callback;

//--------------------------------------------
void hal_ccm_set_key(uint8_t *key)
{
	memcpy(ccm_cnf.key, key, sizeof(ccm_cnf.key));
}

//--------------------------------------------
void hal_ccm_set_pktctr(uint64_t pktctr, uint8_t dir)
{
	// dir = 0, central -> peripheral
	// dir = 1, peripheral -> central
	ccm_cnf.pktctr[0] = (uint8_t)pktctr;
	ccm_cnf.pktctr[1] = (uint8_t)(pktctr >> 8);
	ccm_cnf.pktctr[2] = (uint8_t)(pktctr >> 16);
	ccm_cnf.pktctr[3] = (uint8_t)(pktctr >> 24);
	ccm_cnf.pktctr[4] = (uint8_t)(pktctr >> 32) & 0x7f;
	ccm_cnf.pktctr[8] = dir == 0 ? 0x01 : 0x00;
}

//--------------------------------------------
void hal_ccm_set_iv_c(uint8_t *iv)
{
	memcpy(ccm_cnf.iv, iv, sizeof(ccm_cnf.iv) / 2);
}

//--------------------------------------------
void hal_ccm_set_iv_p(uint8_t *iv)
{
	memcpy(ccm_cnf.iv + sizeof(ccm_cnf.iv) / 2, iv, sizeof(ccm_cnf.iv) / 2);
}

//--------------------------------------------
void hal_ccm_init(void)
{
	NRF_CCM_NS->ENABLE = (CCM_ENABLE_ENABLE_Enabled << CCM_ENABLE_ENABLE_Pos);
	NRF_CCM_NS->MAXPACKETSIZE = MAX_PACKET_SIZE;
	NRF_CCM_NS->SCRATCHPTR = (uint32_t)scratch_data;
	NRF_CCM_NS->MODE = (CCM_MODE_MODE_Decryption << CCM_MODE_MODE_Pos) | (CCM_MODE_LENGTH_Extended << CCM_MODE_LENGTH_Pos);
	NRF_CCM_NS->CNFPTR = (uint32_t)&ccm_cnf;

	// Clear events
	NRF_CCM_NS->EVENTS_ERROR = 0;
	NRF_CCM_NS->EVENTS_ENDKSGEN = 0;
	NRF_CCM_NS->EVENTS_ENDCRYPT = 0;

	// Enable interrupt on ENDCRYPT state
	NRF_CCM_NS->INTENSET = CCM_INTENSET_ENDCRYPT_Msk;

	// Enable shortcuts
	NRF_CCM_NS->SHORTS = (CCM_SHORTS_ENDKSGEN_CRYPT_Enabled << CCM_SHORTS_ENDKSGEN_CRYPT_Pos);

	// Enable AAR_CCM interrupt
	NVIC_SetPriority(AAR_CCM_IRQn, AAR_CCM_IRQ_PRIORITY);
	NVIC_EnableIRQ(AAR_CCM_IRQn);
}

//--------------------------------------------
void hal_ccm_set_irq_callback(ccm_irq_callback_t callback)
{
	irq_callback = callback;
}

//--------------------------------------------
void hal_ccm_decrypt(uint8_t *enc_buf, uint8_t *dec_buf)
{
	NRF_CCM_NS->INPTR = (uint32_t)enc_buf;
	NRF_CCM_NS->OUTPTR = (uint32_t)dec_buf;

	// Start decryption
	NRF_CCM_NS->TASKS_KSGEN = 1;
}

//--------------------------------------------
void AAR_CCM_IRQHandler(void)
{
	// Clear events
	NRF_CCM_NS->EVENTS_ENDKSGEN = 0;
	NRF_CCM_NS->EVENTS_ENDCRYPT = 0;

	if (irq_callback)
	{
		irq_callback(NRF_CCM_NS->MICSTATUS);
	}
}
