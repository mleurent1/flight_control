#include "reg.h"
#include "fc.h" // flags, sensor_raw, radio_raw
#include "board.h" // CMSIS
#include "sensor.h" // mpu_cal()
#include "radio.h" // default idle/range

uint32_t reg[NB_REG];
float regf[NB_REG];
reg_properties_t reg_properties[NB_REG] = 
{
	{1, 1, 0, 28}, // VERSION
	{0, 0, 0, 0}, // CTRL
	{0, 0, 0, 0}, // MOTOR_TEST
	{0, 0, 0, 32512}, // DEBUG
	{1, 0, 0, 0}, // ERROR
	{1, 0, 0, 0}, // TIME
	{1, 0, 1, 0}, // VBAT
	{0, 1, 1, 1097649357}, // VBAT_MIN
	{0, 1, 0, 327682000}, // TIME_CONSTANT
	{0, 1, 0, 100}, // TIME_CONSTANT_RADIO
	{0, 1, 1, 1082130432}, // EXPO_PITCH_ROLL
	{0, 1, 1, 1077936128}, // EXPO_YAW
	{0, 1, 0, 1782758450}, // MOTOR
	{0, 1, 0, 756450000}, // RATE
	{0, 1, 1, 1073741824}, // P_PITCH
	{0, 1, 1, 1017370378}, // I_PITCH
	{0, 1, 1, 0}, // D_PITCH
	{0, 1, 1, 1073741824}, // P_ROLL
	{0, 1, 1, 1017370378}, // I_ROLL
	{0, 1, 1, 0}, // D_ROLL
	{0, 1, 1, 1073741824}, // P_YAW
	{0, 1, 1, 1017370378}, // I_YAW
	{0, 1, 1, 0}, // D_YAW
	{0, 1, 1, 1084227584}, // P_PITCH_ANGLE
	{0, 1, 1, 0}, // I_PITCH_ANGLE
	{0, 1, 1, 1140457472}, // D_PITCH_ANGLE
	{0, 1, 1, 1084227584}, // P_ROLL_ANGLE
	{0, 1, 1, 0}, // I_ROLL_ANGLE
	{0, 1, 1, 1140457472}, // D_ROLL_ANGLE
	{1, 1, 0, 0}, // GYRO_DC_XY
	{1, 1, 0, 0}, // GYRO_DC_Z
	{1, 1, 0, 0}, // ACCEL_DC_XY
	{1, 1, 0, 0}, // ACCEL_DC_Z
	{1, 1, 0, 0}, // THROTTLE
	{1, 1, 0, 0}, // AILERON
	{1, 1, 0, 0}, // ELEVATOR
	{1, 1, 0, 0} // RUDDER
};

#ifdef STM32F3
	uint32_t* flash_r = (uint32_t*)REG_FLASH_ADDR;
	uint16_t* flash_w = (uint16_t*)REG_FLASH_ADDR;
#elif defined(STM32F4)
	uint32_t* flash_r = (uint32_t*)REG_FLASH_ADDR;
	uint32_t* flash_w = (uint32_t*)REG_FLASH_ADDR;
#endif

float expo_scale_pitch_roll;
float expo_scale_yaw;
float filter_alpha_radio;
float filter_alpha_accel;
float filter_alpha_vbat;

void reg_init()
{
	int i;
	
	_Bool reg_flash_valid;
	uint32_t reg_default;
	
	if (flash_r[0] == reg_properties[0].dflt)
		reg_flash_valid = 1;
	else
		reg_flash_valid = 0;
	
	for(i=0; i<NB_REG; i++)
	{
		if (reg_properties[i].flash && reg_flash_valid)
			reg_default = flash_r[i];
		else
			reg_default = reg_properties[i].dflt;
		if (reg_properties[i].is_float)
			regf[i] = uint32_to_float(reg_default);
		else
			reg[i] = reg_default;
	}
	
	REG_VBAT = 15.0f;
	
	if (!reg_flash_valid ) {
		REG_THROTTLE = ((THROTTLE_IDLE_DEFAULT << REG_THROTTLE__IDLE_Pos) & REG_THROTTLE__IDLE_Msk) | ((THROTTLE_RANGE_DEFAULT << REG_THROTTLE__RANGE_Pos) & REG_THROTTLE__RANGE_Msk);
		REG_AILERON = ((AILERON_IDLE_DEFAULT << REG_AILERON__IDLE_Pos) & REG_AILERON__IDLE_Msk) | ((AILERON_RANGE_DEFAULT << REG_AILERON__RANGE_Pos) & REG_AILERON__RANGE_Msk);
		REG_ELEVATOR = ((ELEVATOR_IDLE_DEFAULT << REG_ELEVATOR__IDLE_Pos) & REG_ELEVATOR__IDLE_Msk) | ((ELEVATOR_RANGE_DEFAULT << REG_ELEVATOR__RANGE_Pos) & REG_ELEVATOR__RANGE_Msk);
		REG_RUDDER = ((RUDDER_IDLE_DEFAULT << REG_RUDDER__IDLE_Pos) & REG_RUDDER__IDLE_Msk) | ((RUDDER_RANGE_DEFAULT << REG_RUDDER__RANGE_Pos) & REG_RUDDER__RANGE_Msk);
	}
	
	reg_update_on_write();
}

void reg_update_on_write(void)
{
	set_mpu_host(REG_CTRL__SENSOR_HOST_CTRL == 1);
	
	expo_scale_pitch_roll = EXPONENTIAL(REG_EXPO_PITCH_ROLL) - 1;
	expo_scale_yaw = EXPONENTIAL(REG_EXPO_YAW) - 1;
	
	if (REG_CTRL__BEEP_TEST)
		flag_beep_host = 1;
	else
		flag_beep_host = 0;
	
	if (REG_TIME_CONSTANT_RADIO == 0)
		filter_alpha_radio = 1.0f;
	else
		filter_alpha_radio = 1.0f / (float)REG_TIME_CONSTANT_RADIO;
	if (REG_TIME_CONSTANT__ACCEL == 0)
		filter_alpha_accel = 1.0f;
	else
		filter_alpha_accel = 1.0f / (float)REG_TIME_CONSTANT__ACCEL;
	if (REG_TIME_CONSTANT__VBAT < VBAT_PERIOD) {
		REG_TIME_CONSTANT &= ~REG_TIME_CONSTANT__VBAT_Msk;
		REG_TIME_CONSTANT |= VBAT_PERIOD << REG_TIME_CONSTANT__VBAT_Pos;
	}
	filter_alpha_vbat  = (float)VBAT_PERIOD / (float)REG_TIME_CONSTANT__VBAT;
	
	flag_acro = (REG_CTRL__ARM_TEST == 1);
	
	if (REG_CTRL__SENSOR_CAL) {
		REG_CTRL &= ~REG_CTRL__SENSOR_CAL_Msk;
		mpu_cal(&sensor_raw);
	}
	
	if (REG_CTRL__RADIO_CAL_IDLE) {
		REG_CTRL &= ~REG_CTRL__RADIO_CAL_IDLE_Msk;
		radio_cal_idle(&radio_frame);
	}
	
	if (REG_CTRL__RADIO_CAL_RANGE) {
		REG_CTRL &= ~REG_CTRL__RADIO_CAL_RANGE_Msk;
		radio_cal_range(&radio_frame);
	}
}

void reg_update_on_read(void)
{
	REG_ERROR = ((uint32_t)rf_error_count << 16) | ((uint32_t)radio_error_count << 8) | (uint32_t)sensor_error_count;
	REG_TIME = ((uint32_t)time_process << 16) | (uint32_t)time_sensor;
}

void reg_access(host_buffer_rx_t * host_buffer_rx)
{
	uint8_t addr = host_buffer_rx->addr;
	
	switch (host_buffer_rx->instr)
	{
		case 0: // REG read
		{
			reg_update_on_read();
			if (reg_properties[addr].is_float)
				host_send((uint8_t*)&regf[addr], 4);
			else
				host_send((uint8_t*)&reg[addr], 4);
			break;
		}
		case 1: // REG write
		{
			if (!reg_properties[addr].read_only)
			{
				if (reg_properties[addr].is_float)
					regf[addr] = host_buffer_rx->data.f;
				else
					reg[addr] = host_buffer_rx->data.u32;
				reg_update_on_write();
			}
			break;
		}
		case 2: // SPI read to MPU
		{
			flag_sensor_host_read = 1;
			sensor_read(addr,1);
			break;
		}
		case 3: // SPI write to MPU
		{
			sensor_write(addr, host_buffer_rx->data.u8[3]);
			break;
		}
		case 4: // Flash read
		{
			host_send((uint8_t*)&flash_r[addr], 4);
			break;
		}
		case 5: // Flash write
		{
			if (FLASH->CR & FLASH_CR_LOCK) {
				FLASH->KEYR = 0x45670123;
				FLASH->KEYR = 0xCDEF89AB;
			}
			#ifdef STM32F3
				FLASH->CR |= FLASH_CR_PG;
				flash_w[addr*2] = host_buffer_rx->data.u16[0];
				flash_w[addr*2+1] = host_buffer_rx->data.u16[1];
				while (FLASH->SR & FLASH_SR_BSY) {}
				FLASH->CR &= ~FLASH_CR_PG;
			#elif defined(STM32F4)
				FLASH->CR |= FLASH_CR_PG | (2 << FLASH_CR_PSIZE_Pos);
				flash_w[addr] = host_buffer_rx->data.u32;
				while (FLASH->SR & FLASH_SR_BSY) {}
				FLASH->CR &= ~FLASH_CR_PG;
			#endif
			break;
		}
		case 6: // Flash page erase
		{
			if (FLASH->CR & FLASH_CR_LOCK) {
				FLASH->KEYR = 0x45670123;
				FLASH->KEYR = 0xCDEF89AB;
			}
			#ifdef STM32F3
				FLASH->CR |= FLASH_CR_PER;
				FLASH->AR = REG_FLASH_ADDR;
				FLASH->CR |= FLASH_CR_STRT;
				while (FLASH->SR & FLASH_SR_BSY) {}
				FLASH->CR &= ~FLASH_CR_PER;
			#elif defined(STM32F4)
				FLASH->CR |= FLASH_CR_SER | (11 << FLASH_CR_SNB_Pos);
				FLASH->CR |= FLASH_CR_STRT;
				while (FLASH->SR & FLASH_SR_BSY) {}
				FLASH->CR &= ~FLASH_CR_SER;
			#endif
			break;
		}
		case 7: // SPI read to RF
		{
			flag_rf_host_read = 1;
			//rf_read(addr,1);
			break;
		}
		case 8: // SPI write to RF
		{
			//RF_WRITE_1(addr, host_buffer_rx->data.u8[3]);
			break;
		}
	}
}
