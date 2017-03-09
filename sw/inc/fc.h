#include "usbd_conf.h"
#include "usbd_desc.h"
#include "usbd_cdc_if.h"
#include "fc_reg.h"
#include "mpu_reg.h"

/* Public variables -----------------*/

extern USBD_HandleTypeDef USBD_device_handler;

/* Public functions -----------------*/

void HostCommand(uint8_t* buffer);
