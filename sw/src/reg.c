#include "reg.h"
#include "fc.h" // flags, sensor_raw, radio_raw
#include "sensor.h" // mpu_cal()
#include "sensor_reg.h"
#include "utils.h" // uint32_to_float()
#ifdef STM32F4
	#include "stm32f4xx.h"
#else
	#include "stm32f3xx.h" // FLASH->
#endif

uint32_t reg[NB_REG];
float regf[NB_REG];
reg_properties_t reg_properties[NB_REG] =
{
	{1, 1, 0, 35}, // VERSION
	{1, 0, 0, 0}, // STATUS
	{0, 0, 0, 0}, // CTRL
	{0, 0, 0, 0}, // MOTOR_TEST
	{1, 0, 0, 0}, // ERROR
	{0, 1, 1, 1088841318}, // VBAT_MIN
	{0, 1, 0, 327682000}, // TIME_CONSTANT
	{0, 1, 0, 100}, // TIME_CONSTANT_RADIO
	{0, 1, 1, 1077936128}, // EXPO_PITCH_ROLL
	{0, 1, 1, 1073741824}, // EXPO_YAW
	{0, 1, 0, 1782784050}, // MOTOR
	{0, 1, 0, 757187460}, // RATE
	{0, 1, 1, 1073741824}, // P_PITCH
	{0, 1, 1, 1000593162}, // I_PITCH
	{0, 1, 1, 0}, // D_PITCH
	{0, 1, 1, 1073741824}, // P_ROLL
	{0, 1, 1, 1000593162}, // I_ROLL
	{0, 1, 1, 0}, // D_ROLL
	{0, 1, 1, 1082130432}, // P_YAW
	{0, 1, 1, 1008981770}, // I_YAW
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
	{0, 1, 0, 65537000}, // THROTTLE
	{0, 1, 0, 32769500}, // AILERON
	{0, 1, 0, 32769500}, // ELEVATOR
	{0, 1, 0, 32769500}, // RUDDER
	{0, 1, 0, 65537000}, // AUX
	{0, 1, 0, 1}, // MPU_CFG
	{0, 1, 0, 1}, // FC_CFG
	{0, 0, 0, 0}, // DEBUG_INT
	{0, 0, 1, 0} // DEBUG_FLOAT
};

#ifdef STM32F3
	#define REG_FLASH_ADDR 0x0800F800
	uint32_t* flash_r = (uint32_t*)REG_FLASH_ADDR;
	uint16_t* flash_w = (uint16_t*)REG_FLASH_ADDR;
#elif defined(STM32F4)
	//#define REG_FLASH_ADDR 0x080E0000
	#define REG_FLASH_ADDR 0x08060000
	uint32_t* flash_r = (uint32_t*)REG_FLASH_ADDR;
	uint32_t* flash_w = (uint32_t*)REG_FLASH_ADDR;
#endif

float sensor_rate;
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
		else {
			reg[i] = reg_default;
		}
	}

	sensor_rate = 1000.0f / (float)(REG_MPU_CFG__RATE+1);
	if (REG_TIME_CONSTANT_RADIO == 0)
		filter_alpha_radio = 1.0f;
	else
		filter_alpha_radio = 1.0f / (float)REG_TIME_CONSTANT_RADIO;
	if (REG_TIME_CONSTANT__ACCEL == 0)
		filter_alpha_accel = 1.0f;
	else
		filter_alpha_accel = 1.0f / (float)REG_TIME_CONSTANT__ACCEL;
	if (REG_TIME_CONSTANT__VBAT < VBAT_PERIOD)
		filter_alpha_vbat  = 1.0f;
	else
		filter_alpha_vbat  = (float)VBAT_PERIOD / (float)REG_TIME_CONSTANT__VBAT;
}

void reg_access(host_buffer_rx_t * host_buffer_rx)
{
	uint8_t addr = host_buffer_rx->addr;
	uint32_t old_data;

	switch (host_buffer_rx->instr)
	{
		case 0: // REG read
		{
			if (reg_properties[addr].is_float)
				host_send((uint8_t*)&regf[addr], 4);
			else {
				if (addr == REG_ERROR_Addr)
					REG_ERROR = ((uint32_t)rf_error_count << 16) | ((uint32_t)radio_error_count << 8) | (uint32_t)sensor_error_count;
				else if (addr == REG_DEBUG_INT_Addr) {
					/* REG_DEBUG_INT = Put your debug value here */
				}
				else if (addr == REG_DEBUG_FLOAT_Addr) {
					/* REG_DEBUG_FLOAT = Put your debug value here */
				}
				host_send((uint8_t*)&reg[addr], 4);
			}
			break;
		}
		case 1: // REG write
		{
			if (!reg_properties[addr].read_only)
			{
				if (reg_properties[addr].is_float)
					regf[addr] = host_buffer_rx->data.f[0];
				else {
					old_data = reg[addr];
					reg[addr] = host_buffer_rx->data.u32[0];
					if (addr == REG_CTRL_Addr) {
						if (REG_CTRL__SENSOR_HOST_CTRL != (old_data & REG_CTRL__SENSOR_HOST_CTRL_Msk) >> REG_CTRL__SENSOR_HOST_CTRL_Pos) {
							set_mpu_host(REG_CTRL__SENSOR_HOST_CTRL == 1);
						}
						if (REG_CTRL__SENSOR_CAL) {
							mpu_cal(&sensor_raw);
							REG_CTRL &= ~REG_CTRL__SENSOR_CAL_Msk;
						}
					}
					else if (addr == REG_MPU_CFG_Addr) {
						if (REG_MPU_CFG != old_data) {
							set_mpu_host(1);
							sensor_write(MPU_CFG, MPU_CFG__DLPF_CFG(REG_MPU_CFG__FILT));
							sensor_write(MPU_SMPLRT_DIV, REG_MPU_CFG__RATE);
							set_mpu_host(REG_CTRL__SENSOR_HOST_CTRL == 1);
							sensor_rate = 1000.0f / (float)(REG_MPU_CFG__RATE+1);
						}
					}
					else if (addr == REG_TIME_CONSTANT_RADIO_Addr) {
						if (REG_TIME_CONSTANT_RADIO == 0)
							filter_alpha_radio = 1.0f;
						else
							filter_alpha_radio = 1.0f / (float)REG_TIME_CONSTANT_RADIO;
					}
					else if (addr == REG_TIME_CONSTANT_Addr) {
						if (REG_TIME_CONSTANT__ACCEL == 0)
							filter_alpha_accel = 1.0f;
						else
							filter_alpha_accel = 1.0f / (float)REG_TIME_CONSTANT__ACCEL;
						if (REG_TIME_CONSTANT__VBAT < VBAT_PERIOD)
							filter_alpha_vbat  = 1.0f;
						else
							filter_alpha_vbat  = (float)VBAT_PERIOD / (float)REG_TIME_CONSTANT__VBAT;
					}
					else if (addr == REG_DEBUG_INT_Addr) {
						/* Put your debug command here */
					}
					else if (addr == REG_DEBUG_FLOAT_Addr) {
						/* Put your debug command here */
					}
				}
			}
			break;
		}
		case 2: // SPI read to MPU
		{
			if (REG_CTRL__SENSOR_HOST_CTRL == 1)
				sensor_read(addr,1);
			break;
		}
		case 3: // SPI write to MPU
		{
			if (REG_CTRL__SENSOR_HOST_CTRL == 1)
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
				flash_w[addr] = host_buffer_rx->data.u32[0];
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
				FLASH->CR |= FLASH_CR_SER | (7 << FLASH_CR_SNB_Pos);
				FLASH->CR |= FLASH_CR_STRT;
				while (FLASH->SR & FLASH_SR_BSY) {}
				FLASH->CR &= ~FLASH_CR_SER;
			#endif
			break;
		}
	#ifdef RF
		case 7: // SPI read to RF
		{
			flag_rf_host_read = 1;
			rf_read(addr,1);
			break;
		}
		case 8: // SPI write to RF
		{
			RF_WRITE_1(addr, host_buffer_rx->data.u8[3]);
			break;
		}
	#endif
	#ifdef RUNCAM
		case 9: // UART send to Runcam
		{
			runcam_send(host_buffer_rx->addr);
			break;
		}
	#endif
	}
}
