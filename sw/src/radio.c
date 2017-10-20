#include "radio.h"
#include "radio_reg.h"
#include "reg.h" // filter_alpha
#include "utils.h" // EXPO, SIN,...
#include "board.h" // rf_write
#include "fc.h" // flag

/* Private macros --------------------------------------*/

#define RF_WRITE(addr,data) rf_data_w[0] = data; rf_write(addr, rf_data_w, 1); wait_ms(1);

/* Global variables -----------------------*/

uint8_t rf_data_w[6];

/* Functions -----------------------*/

// TODO: verify checksum

_Bool radio_decode(radio_frame_t * radio_frame, struct radio_raw_s * radio_raw, struct radio_s * radio)
{
#if (RADIO_TYPE == IBUS)
	if (radio_frame->frame.header == 0x4020) {
		radio_raw->throttle = radio_frame->frame.chan[2];
		radio_raw->aileron  = radio_frame->frame.chan[0];
		radio_raw->elevator = radio_frame->frame.chan[1];
		radio_raw->rudder   = radio_frame->frame.chan[3];
		radio_raw->aux[0]   = radio_frame->frame.chan[4];
		radio_raw->aux[1]   = radio_frame->frame.chan[5];
		radio_raw->aux[2]   = radio_frame->frame.chan[6];
		radio_raw->aux[3]   = radio_frame->frame.chan[7];
#elif (RADIO_TYPE == SUMD)
	if ((radio_frame->frame.vendor_id == 0xA8) && ((radio_frame->frame.status == 0x01) || (radio_frame->frame.status == 0x81))) {
		radio_raw->throttle = radio_frame->frame.chan[0];
		radio_raw->aileron  = radio_frame->frame.chan[1];
		radio_raw->elevator = radio_frame->frame.chan[2];
		radio_raw->rudder   = radio_frame->frame.chan[3];
		radio_raw->aux[0]   = radio_frame->frame.chan[4];
		radio_raw->aux[1]   = radio_frame->frame.chan[5];
		radio_raw->aux[2]   = radio_frame->frame.chan[6];
		radio_raw->aux[3]   = radio_frame->frame.chan[7];
#elif (RADIO_TYPE == SBUS)
	//radio_frame->frame.header = (uint8_t)(__RBIT((uint32_t)radio_frame->frame.header) >> 24);
	//if (((radio_frame->frame.header == 0xF0) || (radio_frame->frame.header == 0xF1)) && (radio_frame->frame.end_byte == 0x00)) {
	if (((radio_frame->frame.header == 0x0F) || (radio_frame->frame.header == 0x8F)) && (radio_frame->frame.end_byte == 0x00)) {
		radio_raw->throttle = radio_frame->frame.chan2;
		radio_raw->aileron  = radio_frame->frame.chan0;
		radio_raw->elevator = radio_frame->frame.chan1;
		radio_raw->rudder   = radio_frame->frame.chan3;
		radio_raw->aux[0]   = radio_frame->frame.chan4;
		radio_raw->aux[1]   = radio_frame->frame.chan5;
		radio_raw->aux[2]   = radio_frame->frame.chan6;
		radio_raw->aux[3]   = radio_frame->frame.chan7;
#endif
		
		radio->throttle = (float)((int32_t)radio_raw->throttle - (int32_t)REG_THROTTLE__IDLE) / (float)REG_THROTTLE__RANGE;
		radio->pitch = (float)((int32_t)radio_raw->elevator - (int32_t)REG_ELEVATOR__IDLE) / (float)REG_ELEVATOR__RANGE;
		radio->roll = (float)((int32_t)radio_raw->aileron - (int32_t)REG_AILERON__IDLE) / (float)REG_AILERON__RANGE;
		radio->yaw = (float)((int32_t)radio_raw->rudder - (int32_t)REG_RUDDER__IDLE) / (float)REG_RUDDER__RANGE;
		radio->aux[0] = (float)((int32_t)radio_raw->aux[0] - (int32_t)AUX_IDLE_DEFAULT) / (float)AUX_RANGE_DEFAULT;
		radio->aux[1] = (float)((int32_t)radio_raw->aux[1] - (int32_t)AUX_IDLE_DEFAULT) / (float)AUX_RANGE_DEFAULT;
		radio->aux[2] = (float)((int32_t)radio_raw->aux[2] - (int32_t)AUX_IDLE_DEFAULT) / (float)AUX_RANGE_DEFAULT;
		radio->aux[3] = (float)((int32_t)radio_raw->aux[3] - (int32_t)AUX_IDLE_DEFAULT) / (float)AUX_RANGE_DEFAULT;
		
		return 0;
	}
	else
		return 1;
}

void radio_cal_idle(radio_frame_t * radio_frame)
{
	uint16_t radio_sample_count = 0;
	struct radio_raw_s radio_raw1;
	struct radio_s radio1;
	
	float aileron_idle = 0;
	float elevator_idle = 0;
	float rudder_idle = 0;
	
	while (radio_sample_count < 100)
	{
		if (flag_radio)
		{
			flag_radio = 0;
			
			radio_decode(radio_frame, &radio_raw1, &radio1);
			
			aileron_idle += (float)radio_raw1.aileron;
			elevator_idle += (float)radio_raw1.elevator;
			rudder_idle += (float)radio_raw1.rudder;
			
			if ((radio_sample_count & 0x03) == 0)
				toggle_led_sensor();
			
			radio_sample_count++;
		}
		__wfi();
	}
	
	REG_AILERON &= ~REG_AILERON__IDLE_Msk;
	REG_AILERON |= ((uint32_t)(aileron_idle / 100.0f) << REG_AILERON__IDLE_Pos) & REG_AILERON__IDLE_Msk;
	REG_ELEVATOR &= ~REG_ELEVATOR__IDLE_Msk;
	REG_ELEVATOR |= ((uint32_t)(elevator_idle / 100.0f) << REG_ELEVATOR__IDLE_Pos) & REG_ELEVATOR__IDLE_Msk;
	REG_RUDDER &= ~REG_RUDDER__IDLE_Msk;
	REG_RUDDER |= ((uint32_t)(rudder_idle / 100.0f) << REG_RUDDER__IDLE_Pos) & REG_RUDDER__IDLE_Msk;
}

void radio_cal_range(radio_frame_t * radio_frame)
{
	uint16_t radio_sample_count = 0;
	struct radio_raw_s radio_raw1;
	struct radio_s radio1;
	
	uint16_t throttle_min = 0xFFFF;
	uint16_t throttle_max = 0;
	uint16_t aileron_min = 0xFFFF;
	uint16_t aileron_max = 0;
	uint16_t elevator_min = 0xFFFF;
	uint16_t elevator_max = 0;
	uint16_t rudder_min = 0xFFFF;
	uint16_t rudder_max = 0;
	
	while (radio_sample_count < 1000)
	{
		if (flag_radio)
		{
			flag_radio = 0;
			
			radio_decode(radio_frame, &radio_raw1, &radio1);
			
			if (radio_raw1.throttle > throttle_max)
				throttle_max = radio_raw1.throttle;
			if (radio_raw1.throttle < throttle_min)
				throttle_min = radio_raw1.throttle;
			if (radio_raw1.aileron > aileron_max)
				aileron_max = radio_raw1.aileron;
			if (radio_raw1.aileron < aileron_min)
				aileron_min = radio_raw1.aileron;
			if (radio_raw1.elevator > elevator_max)
				elevator_max = radio_raw1.elevator;
			if (radio_raw1.elevator < elevator_min)
				elevator_min = radio_raw1.elevator;
			if (radio_raw1.rudder > rudder_max)
				rudder_max = radio_raw1.rudder;
			if (radio_raw1.rudder < rudder_min)
				rudder_min = radio_raw1.rudder;
			
			if ((radio_sample_count & 0x03) == 0)
				toggle_led_sensor();
			
			radio_sample_count++;
		}
		__wfi();
	}
	
	REG_THROTTLE = (((uint32_t)throttle_min << REG_THROTTLE__IDLE_Pos) & REG_THROTTLE__IDLE_Msk) | (((uint32_t)(throttle_max - throttle_min) << REG_THROTTLE__RANGE_Pos) & REG_THROTTLE__RANGE_Msk);
	
	REG_AILERON &= ~REG_AILERON__RANGE_Msk;
	REG_AILERON |= ((uint32_t)((aileron_max - aileron_min)>>1) << REG_AILERON__RANGE_Pos) & REG_AILERON__RANGE_Msk;
	REG_ELEVATOR &= ~REG_ELEVATOR__RANGE_Msk;
	REG_ELEVATOR |= ((uint32_t)((elevator_max - elevator_min)>>1) << REG_ELEVATOR__RANGE_Pos) & REG_ELEVATOR__RANGE_Msk;
	REG_RUDDER &= ~REG_RUDDER__RANGE_Msk;
	REG_RUDDER |= ((uint32_t)((rudder_max - rudder_min)>>1) << REG_RUDDER__RANGE_Pos) & REG_RUDDER__RANGE_Msk;
}

void radio_expo(struct radio_s * radio, _Bool acro_mode)
{
	if (acro_mode) {
		if (radio->pitch >= 0) radio->pitch =  (EXPONENTIAL( radio->pitch * REG_EXPO_PITCH_ROLL) - 1) / expo_scale_pitch_roll;
		else                   radio->pitch = -(EXPONENTIAL(-radio->pitch * REG_EXPO_PITCH_ROLL) - 1) / expo_scale_pitch_roll;
		if (radio->roll >= 0) radio->roll =  (EXPONENTIAL( radio->roll * REG_EXPO_PITCH_ROLL) - 1) / expo_scale_pitch_roll;
		else                  radio->roll = -(EXPONENTIAL(-radio->roll * REG_EXPO_PITCH_ROLL) - 1) / expo_scale_pitch_roll;
	}
	if (radio->yaw >= 0) radio->yaw =  (EXPONENTIAL( radio->yaw  * REG_EXPO_YAW) - 1) / expo_scale_yaw;
	else                 radio->yaw = -(EXPONENTIAL(-radio->yaw  * REG_EXPO_YAW) - 1) / expo_scale_yaw;
}

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
