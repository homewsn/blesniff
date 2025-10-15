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
#include <stddef.h>    /* size_t */
#include <stdbool.h>   /* bool */
#include <assert.h>    /* assert */
#include "platform.h"
#include "radio.h"

//--------------------------------------------
void main(void)
{
	platform_init();
	radio_init();

	while (1);
}
