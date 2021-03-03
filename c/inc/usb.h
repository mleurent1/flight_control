#ifndef __USB_H
#define __USB_H

#include "usbd_def.h"
#include "usbd_cdc_if.h"

/* Public defines -----------------*/

/* Public types -----------------*/

/* Public variables -----------------*/

extern USBD_HandleTypeDef USBD_device_handler;

/* Public functions -----------------*/

void usb_init(void);
	
#endif
