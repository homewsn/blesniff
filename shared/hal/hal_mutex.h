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

#ifndef HAL_MUTEX_H_
#define HAL_MUTEX_H_

//--------------------------------------------
#ifdef NRF5340_XXAA_APPLICATION
void hal_mutex_init(void);
#endif
void hal_mutex_lock(uint8_t number);
void hal_mutex_unlock(uint8_t number);

#endif /* HAL_MUTEX_H_ */