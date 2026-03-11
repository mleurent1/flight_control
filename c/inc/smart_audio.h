#ifndef __SMART_AUDIO_H
#define __SMART_AUDIO_H

#include <stdint.h>
#include <stdbool.h>

/* Public types --------------------------------------*/

enum sma_cmd_e {SMA_GET_SETTINGS, SMA_SET_POWER, SMA_SET_CHANNEL};

/* Exported variables -----------------*/

/* Public functions -----------------*/

void sma_send_cmd(enum sma_cmd_e sma_cmd, uint8_t data);
bool sma_process_resp(void);

#endif
