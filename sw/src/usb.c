#include "usbd_cdc_if.h"
#include "usbd_desc.h"
#ifdef STM32F3
	#include "stm32f3xx.h"  // USB->
	#include "utils.h"
#endif

USBD_HandleTypeDef USBD_device_handler;

void usb_init(void)
{
	#ifdef STM32F3
		USB->CNTR &= ~USB_CNTR_PDWN;
		wait_ms(1);
	#endif
	
	USBD_Init(&USBD_device_handler, &USBD_VCP_descriptor, 0);
	USBD_RegisterClass(&USBD_device_handler, &USBD_CDC);
	USBD_CDC_RegisterInterface(&USBD_device_handler, &USBD_CDC_IF_fops);
	USBD_Start(&USBD_device_handler);
}

