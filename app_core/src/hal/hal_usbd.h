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

#ifndef HAL_USBD_H_
#define HAL_USBD_H_

//--------------------------------------------
enum
{
	USB_EVT_DETECTED = 0,
	USB_EVT_REMOVED = 1,
	USB_EVT_READY = 2
};

//--------------------------------------------
typedef void (*usbd_irq_callback_t)(uint8_t port);
typedef void (*usbregulator_irq_callback_t)(uint32_t event);

//--------------------------------------------
void hal_usbd_init(void);
void hal_usbd_get_eight_bytes_id(uint8_t *id);
void hal_usbd_set_irq_callback(usbd_irq_callback_t callback);
void hal_usbregulator_set_irq_callback(usbregulator_irq_callback_t callback);

#endif /* HAL_USBD_H_ */