#include "radio.h"
#include "sx1276_reg.h"
#include "reg.h" // filter_alpha
#include "utils.h" // EXPO, SIN,...
#include "board.h" // rf_write()
#include "fc.h" // flag

/* Private defines --------------------------------------*/

/* Private macros --------------------------------------*/

#define RF_WRITE(addr,data) rf_data_w[0] = data; rf_write(addr, rf_data_w, 1); wait_ms(1);

/* Global variables -----------------------*/

uint8_t rf_data_w[6];

/* Private Functions -----------------------*/

/* Public Functions -----------------------*/

// TODO: verify checksum

_Bool radio_decode(radio_frame_t * radio_frame, struct radio_raw_s * radio_raw, struct radio_s * radio)
{
	if (radio_frame->frame.header == 0x4020) {
		radio_raw->throttle = radio_frame->frame.chan[2];
		radio_raw->aileron  = radio_frame->frame.chan[0];
		radio_raw->elevator = radio_frame->frame.chan[1];
		radio_raw->rudder   = radio_frame->frame.chan[3];
		radio_raw->aux[0]   = radio_frame->frame.chan[4];
		radio_raw->aux[1]   = radio_frame->frame.chan[5];
		radio_raw->aux[2]   = radio_frame->frame.chan[6];
		radio_raw->aux[3]   = radio_frame->frame.chan[7];
		
		radio->throttle = (float)((int32_t)radio_raw->throttle - (int32_t)REG_THROTTLE__IDLE) / (float)REG_THROTTLE__RANGE;
		radio->pitch = (float)((int32_t)radio_raw->elevator - (int32_t)REG_ELEVATOR__IDLE) / (float)REG_ELEVATOR__RANGE;
		radio->roll = (float)((int32_t)radio_raw->aileron - (int32_t)REG_AILERON__IDLE) / (float)REG_AILERON__RANGE;
		radio->yaw = (float)((int32_t)radio_raw->rudder - (int32_t)REG_RUDDER__IDLE) / (float)REG_RUDDER__RANGE;
		radio->aux[0] = (float)((int32_t)radio_raw->aux[0] - (int32_t)REG_AUX__IDLE) / (float)REG_AUX__RANGE;
		radio->aux[1] = (float)((int32_t)radio_raw->aux[1] - (int32_t)REG_AUX__IDLE) / (float)REG_AUX__RANGE;
		radio->aux[2] = (float)((int32_t)radio_raw->aux[2] - (int32_t)REG_AUX__IDLE) / (float)REG_AUX__RANGE;
		radio->aux[3] = (float)((int32_t)radio_raw->aux[3] - (int32_t)REG_AUX__IDLE) / (float)REG_AUX__RANGE;
		
		return 0;
	}
	else
		return 1;
}

void radio_expo(struct radio_s * radio, _Bool acro_mode)
{
	if (acro_mode) {
		if (radio->pitch >= 0) radio->pitch =  (EXPONENTIAL( radio->pitch * REG_EXPO_PITCH_ROLL) - 1.0f) / (EXPONENTIAL(REG_EXPO_PITCH_ROLL) - 1.0f);
		else                   radio->pitch = -(EXPONENTIAL(-radio->pitch * REG_EXPO_PITCH_ROLL) - 1.0f) / (EXPONENTIAL(REG_EXPO_PITCH_ROLL) - 1.0f);
		if (radio->roll >= 0) radio->roll =  (EXPONENTIAL( radio->roll * REG_EXPO_PITCH_ROLL) - 1.0f) / (EXPONENTIAL(REG_EXPO_PITCH_ROLL) - 1.0f);
		else                  radio->roll = -(EXPONENTIAL(-radio->roll * REG_EXPO_PITCH_ROLL) - 1.0f) / (EXPONENTIAL(REG_EXPO_PITCH_ROLL) - 1.0f);
	}
	if (radio->yaw >= 0) radio->yaw =  (EXPONENTIAL( radio->yaw  * REG_EXPO_YAW) - 1.0f) / (EXPONENTIAL(REG_EXPO_YAW) - 1.0f);
	else                 radio->yaw = -(EXPONENTIAL(-radio->yaw  * REG_EXPO_YAW) - 1.0f) / (EXPONENTIAL(REG_EXPO_YAW) - 1.0f);
}

#ifdef RF
	void sx1276_init(void)
	{
		RF_WRITE(SX1276_OP_MODE, 0);
		RF_WRITE(SX1276_OP_MODE, SX1276_OP_MODE__LONG_RANGE_MODE);
		RF_WRITE(SX1276_OP_MODE, SX1276_OP_MODE__LONG_RANGE_MODE | SX1276_OP_MODE__MODE(1));
		RF_WRITE(SX1276_FR_MSB, 216);
		RF_WRITE(SX1276_FR_MID, 64);
		RF_WRITE(SX1276_FR_LSB, 0);
		RF_WRITE(SX1276_PA_CONFIG, SX1276_PA_CONFIG__OUTPUT_POWER(0) | SX1276_PA_CONFIG__PA_SELECT);
		RF_WRITE(SX1276_PA_RAMP, 3);
		RF_WRITE(SX1276_LNA, SX1276_LNA__LNA_BOOST_HF(2) | SX1276_LNA__LNA_GAIN(1));
		RF_WRITE(SX1276_MODEM_CONFIG_1, SX1276_MODEM_CONFIG_1__IMPLICIT_HEADER_MODE_ON | SX1276_MODEM_CONFIG_1__CODING_RATE(1) | SX1276_MODEM_CONFIG_1__BW(7));
		RF_WRITE(SX1276_MODEM_CONFIG_2, SX1276_MODEM_CONFIG_2__RX_PAYLOAD_CRC_ON | SX1276_MODEM_CONFIG_2__SPREADING_FACTOR(7));
		RF_WRITE(SX1276_PAYLOAD_LENGTH, 6);
		RF_WRITE(SX1276_MODEM_CONFIG_3, SX1276_MODEM_CONFIG_3__AGC_AUTO_ON);
		RF_WRITE(SX1276_OP_MODE, SX1276_OP_MODE__LONG_RANGE_MODE | SX1276_OP_MODE__MODE(5));
	}
#endif
