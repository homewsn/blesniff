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

#include <stdint.h>    /* uint32_t */
#include <stdbool.h>   /* bool */
#include "platform.h"
#include "hal_reset.h"
#include "usbdev.h"

//--------------------------------------------
void main(void)
{
	// Stop network core
	hal_reset_network_force_off(true);
	platform_init();
	usbdev_init();
	// Run network core
	hal_reset_network_force_off(false);

	while (1)
	{
		usbdev_loop();
	}
}
