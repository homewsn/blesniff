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

#ifndef HAL_ECB_H_
#define HAL_ECB_H_

//--------------------------------------------
typedef struct
{
	uint8_t key[16];
	uint8_t cleartext[16];
	uint8_t ciphertext[16];
} ecb_data_t;

//--------------------------------------------
bool hal_ecb_encrypt(ecb_data_t *ecb_data);

#endif /* HAL_ECB_H_ */