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

#include <stdbool.h>   /* bool */
#include <string.h>    /* memcpy */
#include <nrf.h>
#include "irq_priorities.h"
#include "hal_usbd.h"

//--------------------------------------------
static usbd_irq_callback_t usbd_irq_callback;
static usbregulator_irq_callback_t usbregulator_irq_callback;

//--------------------------------------------
void hal_usbd_init(void)
{
	NVIC_SetPriority(USBD_IRQn, USBD_IRQ_PRIORITY);

	NVIC_SetPriority(USBREGULATOR_IRQn, USBREGULATOR_IRQ_PRIORITY);
    NVIC_EnableIRQ(USBREGULATOR_IRQn);

	NRF_USBREGULATOR_S->EVENTS_USBDETECTED = 0;
	NRF_USBREGULATOR_S->EVENTS_USBPWRRDY = 0;
	NRF_USBREGULATOR_S->EVENTS_USBREMOVED = 0;

	NRF_USBREGULATOR_S->INTENSET =
		USBREG_INTENSET_USBDETECTED_Msk |
		USBREG_INTENSET_USBREMOVED_Msk |
		USBREG_INTENSET_USBPWRRDY_Msk;
}

//--------------------------------------------
void hal_usbd_get_eight_bytes_id(uint8_t *id)
{
	memcpy(id, (uint8_t *)NRF_FICR_S->INFO.DEVICEID, 8);
}

//--------------------------------------------
void hal_usbd_set_irq_callback(usbd_irq_callback_t callback)
{
	usbd_irq_callback = callback;
}

//--------------------------------------------
void hal_usbregulator_set_irq_callback(usbregulator_irq_callback_t callback)
{
	usbregulator_irq_callback = callback;

	uint32_t usb_reg = NRF_USBREGULATOR_S->USBREGSTATUS;

	if (usb_reg & USBREG_USBREGSTATUS_VBUSDETECT_Msk)
	{
		usbregulator_irq_callback(USB_EVT_DETECTED);
	}
	if (usb_reg & USBREG_USBREGSTATUS_OUTPUTRDY_Msk)
	{
		usbregulator_irq_callback(USB_EVT_READY);
	}
}

//--------------------------------------------
void USBD_IRQHandler(void)
{
	usbd_irq_callback(0);
}

//--------------------------------------------
void USBREGULATOR_IRQHandler(void)
{
	if (NRF_USBREGULATOR_S->EVENTS_USBDETECTED)
	{
		NRF_USBREGULATOR_S->EVENTS_USBDETECTED = 0;
		usbregulator_irq_callback(USB_EVT_DETECTED);
	}
	if (NRF_USBREGULATOR_S->EVENTS_USBREMOVED)
	{
		NRF_USBREGULATOR_S->EVENTS_USBREMOVED = 0;
		usbregulator_irq_callback(USB_EVT_REMOVED);
	}
	if (NRF_USBREGULATOR_S->EVENTS_USBPWRRDY)
	{
		NRF_USBREGULATOR_S->EVENTS_USBPWRRDY = 0;
		usbregulator_irq_callback(USB_EVT_READY);
	}
}
