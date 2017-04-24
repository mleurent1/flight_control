#include <stdint.h>
#include "usbd_def.h"

/* Public defines -----------------*/

#define FLAG__MPU 0x01
#define FLAG__RADIO 0x02
#define FLAG__MOTOR 0x04
#define FLAG__HOST 0x08
#define FLAG__REG 0x10
#define FLAG__MPU_HOST_READ 0x20
#define FLAG__RADIO_SYNCH 0x40
#define FLAG__MPU_CAL 0x80
#define FLAG__VBAT 0x100
#define FLAG__ERROR_MPU 0x200
#define FLAG__ERROR_RADIO 0x400

/* Public variables -----------------*/

extern volatile _Bool flag_host;
extern uint8_t host_rx_buffer[6];
extern USBD_HandleTypeDef USBD_device_handler;

/* Public functions -----------------*/
