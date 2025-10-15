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

#ifndef ADV_HEADER_CACHE_H
#define ADV_HEADER_CACHE_H

//--------------------------------------------
void adv_cache_store(const uint8_t *mac, uint8_t header);
uint8_t adv_cache_fetch(const uint8_t *mac);

#endif /* ADV_HEADER_CACHE_H */
