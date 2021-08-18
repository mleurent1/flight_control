#ifndef __SMART_AUDIO_H
#define __SMART_AUDIO_H

#include <stdint.h>

/* Public types --------------------------------------*/

enum sma_cmd_e {SMA_GET_SETTINGS=1, SMA_SET_POWER, SMA_SET_CHANNEL};

/* Exported variables -----------------*/

extern volatile uint8_t sma_data_received[16];
extern volatile uint8_t sma_nbytes_to_receive;
extern volatile uint8_t vtx_current_chan;
extern volatile uint8_t vtx_current_pwr;

/* Public functions -----------------*/

void sma_send_cmd(enum sma_cmd_e sma_cmd, uint8_t data);
void sma_process_resp(void);


#endif
