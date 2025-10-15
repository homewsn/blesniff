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
#include <string.h>    /* memcpy */
#include <stdbool.h>   /* bool */
#include <assert.h>    /* assert */
#include "cmsis_compiler.h"  /* __disable_irq, __enable_irq */
#include "tusb.h"
#include "hal_perm.h"
#include "hal_usbd.h"
#include "hal_mutex.h"
#include "hal_ipc.h"
#include "ipc_msg_types.h"
#include "radio_packet.h"
#include "msg_queue_app2net.h"
#include "msg_queue_net2app.h"
#include "serial.h"
#include "msg_queue_ipc2serial.h"
#include "msg_queue_serial2ipc.h"

//--------------------------------------------
bool serial_enabled;

//--------------------------------------------
void tusb_hal_nrf_power_event(uint32_t event);

//--------------------------------------------
// Get USB Serial number string from unique ID if available. Return number of character.
// Input is string descriptor from index 1 (index 0 is type + len)
size_t board_usb_get_serial(uint16_t desc_str1[], size_t max_chars)
{
	uint8_t uid[16] TU_ATTR_ALIGNED(4);
	size_t uid_len;

	uid_len = 8;
	hal_usbd_get_eight_bytes_id(uid);

	if (uid_len > max_chars / 2)
	{
		uid_len = max_chars / 2;
	}

	for (size_t i = 0; i < uid_len; i++)
	{
		for (size_t j = 0; j < 2; j++)
		{
			const char nibble_to_hex[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
			uint8_t const nibble = (uid[i] >> (j * 4)) & 0xf;
			desc_str1[i * 2 + (1 - j)] = nibble_to_hex[nibble]; // UTF-16-LE
		}
	}
	return 2 * uid_len;
}

//--------------------------------------------
static void cdc_task(void)
{
	static msg_ipc2serial_t msg;
	static bool res;
	static uint32_t fpos;

	if (!fpos)
	{
		__disable_irq();
		res = msg_queue_ipc2serial_get(&msg);
		__enable_irq();
	}

	serial_enabled = tud_cdc_connected();
	if (serial_enabled)
	{
		uint32_t aval = tud_cdc_write_available();
		if (res && aval)
		{
			if (msg.size - fpos <= aval)
			{
				tud_cdc_write(msg.data + fpos, msg.size - fpos);
				fpos = 0;
			}
			else
			{
				tud_cdc_write(msg.data + fpos, aval);
				fpos += aval;
			}
			tud_cdc_write_flush();
		}
	}
}

//--------------------------------------------
void usbdev_init(void)
{
	// Init hardware
	hal_perm_init();
	hal_ipc_init();
	hal_ipc_set_mem_callback(mem_callback);
	hal_mutex_init();
	hal_usbd_init();
	hal_usbd_set_irq_callback(tud_int_handler);
	hal_usbregulator_set_irq_callback(tusb_hal_nrf_power_event);
	// Init queues
	msg_queue_app2net_init();
	msg_queue_net2app_init();
	msg_queue_ipc2serial_init();
	msg_queue_serial2ipc_init();

	// init device stack on configured roothub port
	tusb_rhport_init_t dev_init =
	{
		.role = TUSB_ROLE_DEVICE,
		.speed = TUSB_SPEED_AUTO
	};
	tusb_init(0, &dev_init);
}

//--------------------------------------------
void usbdev_loop(void)
{
	tud_task();
	cdc_task();
	serial_rx();
}

//--------------------------------------------
// Device callbacks

//--------------------------------------------
// Invoked when device is mounted
void tud_mount_cb(void)
{
}

//--------------------------------------------
// Invoked when device is unmounted
void tud_umount_cb(void)
{
}

//--------------------------------------------
// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
	(void) remote_wakeup_en;
}

//--------------------------------------------
// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
}

//--------------------------------------------
// Invoked when cdc when line state changed e.g connected/disconnected
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
	(void) itf;
	(void) rts;

	// TODO set some indicator
	if (dtr)
	{
		// Terminal connected
	}
	else
	{
		// Terminal disconnected
	}
}

//--------------------------------------------
// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf)
{
	static uint8_t buf[64];
	uint32_t count;
	if (serial_enabled)
	{
		if (tud_cdc_available())
		{
			count = tud_cdc_n_read(itf, buf, sizeof(buf));
			for (size_t cnt = 0; cnt < count; cnt++)
			{
				serial_rx_callback(buf[cnt]);
			}
		}
	}
}
