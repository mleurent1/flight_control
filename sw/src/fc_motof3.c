/* Includes ----------------------*/

#include "stm32f3xx.h"
#include "fc.h"

/* Definitions ------------------------------------*/

#define MOTOR_MAX 2000 // us
#define MOTOR_MIN 1000 // us
#define BEEPER_PERIOD 250 // ms
#define TIMEOUT_COMMAND 500 // ms
#define TIMEOUT_SENSOR 10000 // us
#define I2C_DURATION_MAX 300 // us
#define ADC_SCALE 0.0089f
#define VBAT_ALPHA 0.001f
#define VBAT_PERIOD 10 // ms
#define VBAT_THRESHOLD 8.0f
#define INTEGRAL_MAX 200.0f
#define REG_FLASH_ADDR 0x0803F800

#define FLAG__SENSOR 0x01
#define FLAG__COMMAND 0x02
#define FLAG__DEBUG 0x04
#define FLAG__REG 0x08
#define FLAG__I2C_HOST 0x10

#define DEBUG

/* Pivate variables --------------------------------------*/

uint8_t FLAG;

uint32_t tick;

USBD_HandleTypeDef USBD_device_handler;

volatile uint8_t uart2_rx_buffer[16];
volatile uint8_t i2c2_rx_buffer[15];
volatile uint8_t i2c2_tx_buffer[2];

uint16_t sensor_sample_count;
int16_t gyro_x_raw;
int16_t gyro_y_raw;
int16_t gyro_z_raw;
int16_t accel_x_raw;
int16_t accel_y_raw;
int16_t accel_z_raw;
int16_t temperature_raw;
uint16_t sensor_error_count;

uint16_t command_frame_count;
uint8_t command_fades;
uint8_t command_system;
uint16_t throttle_raw;
uint16_t aileron_raw;
uint16_t elevator_raw;
uint16_t rudder_raw;
uint16_t armed_raw;
uint16_t chan6_raw;
uint16_t command_error_count;

uint16_t time_i2c;

/* Private macros ---------------------------------------------------*/

#define I2C2_TX_DMA_EN(size) \
	DMA1_Channel4->CNDTR = size; \
	DMA1_Channel4->CCR |= DMA_CCR_EN; \
	I2C2->CR2 &= ~(I2C_CR2_RD_WRN | I2C_CR2_NBYTES_Msk); \
	I2C2->CR2 |= (I2C_CR2_NBYTES_Msk & (size << I2C_CR2_NBYTES_Pos)) | I2C_CR2_START;

#define UART2_RX_DMA_EN \
	DMA1_Channel6->CNDTR = 16; \
	DMA1_Channel6->CCR |= DMA_CCR_EN;

#define USB_TX(size) \
	USBD_CDC_SetTxBuffer(&USBD_device_handler, USBD_CDC_IF_tx_buffer, size); \
	USBD_CDC_TransmitPacket(&USBD_device_handler);

/* Private function declarations -----------------------------------*/

void uint32_to_float(uint32_t* b, float* f);
void float_to_uint32(float* f, uint32_t* b);
void wait_ms(uint32_t t);

/* Public functions ---------------------------------------------*/

void HAL_Delay(__IO uint32_t Delay)
{
	wait_ms(Delay);
}

void HostCommand(uint8_t* buffer)
{
	uint8_t cmd;
	uint8_t addr;
	uint32_t val;
	uint32_t* flash_r = (uint32_t*)REG_FLASH_ADDR;
	uint16_t* flash_w = (uint16_t*)REG_FLASH_ADDR;
	
	cmd = buffer[0];
	addr = buffer[1];
	
	switch (cmd)
	{
		case 0: // REG read
		{
			if (reg_properties[addr].is_float)
				float_to_uint32(&regf[addr], &val);
			else
				val = reg[addr];
			USBD_CDC_IF_tx_buffer[0] = (uint8_t)((val >> 24) & 0xFF);
			USBD_CDC_IF_tx_buffer[1] = (uint8_t)((val >> 16) & 0xFF);
			USBD_CDC_IF_tx_buffer[2] = (uint8_t)((val >>  8) & 0xFF);
			USBD_CDC_IF_tx_buffer[3] = (uint8_t)((val >>  0) & 0xFF);
			USB_TX(4)
			break;
		}
		case 1: // REG write
		{
			if (!reg_properties[addr].read_only)
			{
				val = ((uint32_t)buffer[2] << 24) | ((uint32_t)buffer[3] << 16) | ((uint32_t)buffer[4] << 8) | (uint32_t)buffer[5];
				if (reg_properties[addr].is_float)
					uint32_to_float(&val, &regf[addr]);
				else
					reg[addr] = val;
				FLAG |= FLAG__REG;
			}
			break;
		}
		case 2: // I2C read to MPU
		{
			FLAG |= FLAG__I2C_HOST;
			i2c2_tx_buffer[0] = addr;
			DMA1_Channel5->CNDTR = 1;
			I2C2_TX_DMA_EN(1);
			break;
		}
		case 3: // I2C write to MPU
		{
			i2c2_tx_buffer[0] = addr;
			i2c2_tx_buffer[1] = buffer[5];
			I2C2_TX_DMA_EN(2);
			break;
		}
		case 4: // Flash read
		{
			val = flash_r[addr];
			USBD_CDC_IF_tx_buffer[0] = (uint8_t)((val >> 24) & 0xFF);
			USBD_CDC_IF_tx_buffer[1] = (uint8_t)((val >> 16) & 0xFF);
			USBD_CDC_IF_tx_buffer[2] = (uint8_t)((val >>  8) & 0xFF);
			USBD_CDC_IF_tx_buffer[3] = (uint8_t)((val >>  0) & 0xFF);
			USB_TX(4)
			break;
		}
		case 5: // Flash write
		{
			if (FLASH->CR & FLASH_CR_LOCK)
			{
				FLASH->KEYR = 0x45670123;
				FLASH->KEYR = 0xCDEF89AB;
			}
			FLASH->CR |= FLASH_CR_PG;
			flash_w[addr*2] = ((uint16_t)buffer[4] << 8) | (uint16_t)buffer[5];
			flash_w[addr*2+1] = ((uint16_t)buffer[2] << 8) | (uint16_t)buffer[3];
			while (FLASH->SR & FLASH_SR_BSY) {}
			FLASH->CR &= ~FLASH_CR_PG;
			break;
		}
		case 6: // Flash page erase
		{
			if (FLASH->CR & FLASH_CR_LOCK)
			{
				FLASH->KEYR = 0x45670123;
				FLASH->KEYR = 0xCDEF89AB;
			}
			FLASH->CR |= FLASH_CR_PER;
			FLASH->AR = REG_FLASH_ADDR;
			FLASH->CR |= FLASH_CR_STRT;
			while (FLASH->SR & FLASH_SR_BSY) {}
			FLASH->CR &= ~FLASH_CR_PER;
			break;
		}
	}
}

/* Private functions ------------------------------------------------*/

void wait_ms(uint32_t t)
{
	uint32_t next_tick;
	next_tick = tick + t;
	while (tick != next_tick)
		__WFI();
}

void I2cWrite(uint8_t addr, uint8_t data)
{
	i2c2_tx_buffer[0] = addr;
	i2c2_tx_buffer[1] = data;
	I2C2_TX_DMA_EN(2);
	wait_ms(1);
}

uint8_t I2cRead(uint8_t addr)
{
	i2c2_tx_buffer[0] = addr;
	DMA1_Channel5->CNDTR = 1;
	I2C2_TX_DMA_EN(1);
	wait_ms(1);
	return i2c2_rx_buffer[0];
}

void uint32_to_float(uint32_t* b, float* f)
{
	union {
		float f;
		uint32_t b;
	} f2b;
	f2b.b = *b;
	*f = f2b.f;
}

void float_to_bytes(float* f, uint8_t* b)
{
	union {
		float f;
		uint8_t b[4];
	} f2b;
	f2b.f = *f;
	b[0] = f2b.b[0];
	b[1] = f2b.b[1];
	b[2] = f2b.b[2];
	b[3] = f2b.b[3];
}

void float_to_uint32(float* f, uint32_t* b)
{
	union {
		float f;
		uint32_t b;
	} f2b;
	f2b.f = *f;
	*b = f2b.b;
}

float expo(float linear, float scale)
{
	float x;
	float y;
	x = linear * REG_EXPO;
	y = x;
	x = x * linear * REG_EXPO;
	y += x / 2.0f;
	x = x * linear * REG_EXPO;
	y += x / 6.0f;
	x = x * linear * REG_EXPO;
	y += x / 24.0f;
	x = x * linear * REG_EXPO;
	y += x / 120.0f;
	x = x * linear * REG_EXPO;
	y += x / 720.0f;
	return (y / scale);
}

/* Interrupt routines --------------------------------*/

void SysTick_Handler()
{
	tick++;
}

void EXTI15_10_IRQHandler()
{
	EXTI->PR = EXTI_PR_PIF15; // Clear pending request
	
	if ((REG_CTRL__MPU_HOST_CTRL == 0) && (TIM15->CNT > I2C_DURATION_MAX))
	{
		i2c2_tx_buffer[0] = 58;
		DMA1_Channel5->CNDTR = 15;
		I2C2_TX_DMA_EN(1);
	}
	
	// Reset I2C transaction time
	TIM15->CNT = 0;
	#ifdef DEBUG
		GPIOA->BSRR = GPIO_BSRR_BS_3;
	#endif
}

void I2C2_EV_IRQHandler()
{
	_Bool error = 0;
	
	// Check I2C error
	if (I2C2->ISR & (I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_NACKF))// | I2C_ISR_STOPF)) ?
		error = 1;
		
	// Check DMA transfer error
	if (DMA1->ISR & DMA_ISR_TEIF4)
		error = 1;
	
	if (error)
	{
		// End I2C transfer
		I2C2->CR2 |= I2C_CR2_STOP;
		while (I2C2->ISR & I2C_ISR_BUSY) {}
		
		// Reset I2C
		I2C2->CR1 &= ~I2C_CR1_PE;
		while (I2C2->CR1 & I2C_CR1_PE) {}
		I2C2->CR1 |= I2C_CR1_PE;
			
		// Clear nb of transfer of DMA Rx
		DMA1_Channel5->CNDTR = 0;
	}
	else if (DMA1_Channel5->CNDTR > 0)
	{
		// Enable DMA for Rx
		DMA1_Channel5->CCR |= DMA_CCR_EN;
		
		// Restart I2C transfer
		I2C2->CR2 &= ~I2C_CR2_NBYTES_Msk;
		I2C2->CR2 |= (I2C_CR2_NBYTES_Msk & (DMA1_Channel5->CNDTR << I2C_CR2_NBYTES_Pos)) | I2C_CR2_RD_WRN | I2C_CR2_START;
	}
	else
	{
		// End I2C transfer
		I2C2->CR2 |= I2C_CR2_STOP;
		while (I2C2->ISR & I2C_ISR_BUSY) {}
	}
	
	// Disable DMA for Tx
	DMA1->IFCR = DMA_IFCR_CGIF4; // clear all flags
	DMA1_Channel4->CCR &= ~DMA_CCR_EN;
	
	if (error)
	{
		sensor_error_count++;
		REG_ERROR &= 0xFFFF0000;
		REG_ERROR |= (uint32_t)sensor_error_count & 0x0000FFFF;
	}
}

void DMA1_Channel5_IRQHandler()
{
	_Bool error = 0;
	
	// Check DMA transfer error
	if (DMA1->ISR & DMA_ISR_TEIF5)
		error = 1;
	
	// Disable DMA
	DMA1->IFCR = DMA_IFCR_CGIF5; // clear all flags
	DMA1_Channel5->CCR &= ~DMA_CCR_EN;
	
	if (error)
	{
		sensor_error_count++;
		REG_ERROR &= 0xFFFF0000;
		REG_ERROR |= (uint32_t)sensor_error_count & 0x0000FFFF;
	}
	else if (REG_CTRL__MPU_HOST_CTRL == 0)
	{
		if (i2c2_rx_buffer[0] & MPU_INT_STATUS__DATA_RDY_INT)
		{
			sensor_sample_count++;
			
			accel_x_raw     = ((int16_t)i2c2_rx_buffer[ 1] << 8) | (int16_t)i2c2_rx_buffer[ 2];
			accel_y_raw     = ((int16_t)i2c2_rx_buffer[ 3] << 8) | (int16_t)i2c2_rx_buffer[ 4];
			accel_z_raw     = ((int16_t)i2c2_rx_buffer[ 5] << 8) | (int16_t)i2c2_rx_buffer[ 6];
			temperature_raw = ((int16_t)i2c2_rx_buffer[ 7] << 8) | (int16_t)i2c2_rx_buffer[ 8];
			gyro_x_raw      = ((int16_t)i2c2_rx_buffer[ 9] << 8) | (int16_t)i2c2_rx_buffer[10];
			gyro_y_raw      = ((int16_t)i2c2_rx_buffer[11] << 8) | (int16_t)i2c2_rx_buffer[12];
			gyro_z_raw      = ((int16_t)i2c2_rx_buffer[13] << 8) | (int16_t)i2c2_rx_buffer[14];
			
			FLAG |= FLAG__SENSOR;
		}
		
		// Record I2C transaction time
		time_i2c = TIM15->CNT;
		if ((REG_CTRL__TIME_MAXHOLD == 0) || ((time_i2c > REG_TIME__SPI) && REG_CTRL__TIME_MAXHOLD))
		{
			REG_TIME &= 0xFFFF0000;
			REG_TIME |= (uint32_t)time_i2c & 0x0000FFFF;
		}
		#ifdef DEBUG
			GPIOA->BSRR = GPIO_BSRR_BR_3;
		#endif
	}
	else if (FLAG & FLAG__I2C_HOST)
	{
		FLAG &= ~FLAG__I2C_HOST;
		USBD_CDC_IF_tx_buffer[0] = i2c2_rx_buffer[0];
		USB_TX(1)
	}
}

void DMA1_Channel6_IRQHandler()
{
	int i;
	uint8_t chan_id;
	uint16_t servo;
	_Bool error = 0;
	
	// Check DMA transfer error
	if (DMA1->ISR & DMA_ISR_TEIF6)
		error = 1;
	
	// Disable DMA
	DMA1->IFCR = DMA_IFCR_CGIF6; // clear all flags
	DMA1_Channel6->CCR &= ~DMA_CCR_EN;
	
	// Check UART error
	if (USART2->ISR & (USART_ISR_ORE | USART_ISR_NE))
	{
		error = 1;
		USART2->ICR = USART_ICR_ORECF | USART_ICR_NCF;
	}
	
	if (error)
	{
		command_error_count++;
		REG_ERROR &= 0x0000FFFF;
		REG_ERROR |= ((uint32_t)command_fades + (uint32_t)command_error_count) << 16;
	}
	else
	{
		command_fades = uart2_rx_buffer[0];
		command_system = uart2_rx_buffer[1];
		for (i=0; i<7; i++)
		{
			servo = ((uint16_t)uart2_rx_buffer[2+2*i] << 8) | (uint16_t)uart2_rx_buffer[3+2*i];
			chan_id = (uint8_t)((servo & 0x7800) >> 11);
			switch (chan_id)
			{
				case 0:
					throttle_raw = servo & 0x07FF;
					break;
				case 1:
					aileron_raw = servo & 0x07FF;
					break;
				case 2:
					elevator_raw = servo & 0x07FF;
					break;
				case 3:
					rudder_raw = servo & 0x07FF;
					break;
				case 4:
					armed_raw = servo & 0x07FF;
					break;
				case 5:
					chan6_raw = servo & 0x07FF;
					break;
			}
		}
		
		command_frame_count++;
		FLAG |= FLAG__COMMAND;
	}
	
	UART2_RX_DMA_EN
}

void USB_LP_CAN_RX0_IRQHandler()
{
	HAL_PCD_IRQHandler(&PCD_handler);
}

/*----------------------------------------------------------
-----------------------------------------------------------*/

int main()
{
	int i;
	
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float gyro_x_dc;
	float gyro_y_dc;
	float gyro_z_dc;
	float gyro_x_scale;
	float gyro_y_scale;
	float gyro_z_scale;
	float accel_x;
	float accel_y;
	float accel_z;
	float accel_x_scale;
	float accel_y_scale;
	float accel_z_scale;
	float temperature;
	
	float throttle;
	float aileron;
	float elevator;
	float rudder;
	float expo_scale;
	
	float error_pitch;
	float error_roll;
	float error_yaw;
	float error_pitch_z;
	float error_roll_z;
	float error_yaw_z;
	float error_pitch_int;
	float error_roll_int;
	float error_yaw_int;
	float pitch_i;
	float roll_i;
	float yaw_i;
	float pitch_before_tpa;
	float roll_before_tpa;
	float yaw_before_tpa;
	float pitch;
	float roll;
	float yaw;
	
	float motor[4];
	int32_t motor_clip[4];
	uint32_t motor_raw[4];
	
	float vbat_acc;
	float vbat;
	uint16_t vbat_sample_count;
	
	float x;
	
	_Bool reg_flash_valid;
	uint32_t* flash_r;
	uint32_t reg_default;
	
	uint16_t time_process;
	
	/* Variable init ---------------------------------------*/
	
	FLAG = FLAG__REG;
	
	command_frame_count = 0;
	command_error_count = 0;
	throttle_raw = 0;
	aileron_raw = 0;
	elevator_raw = 0;
	rudder_raw = 0;
	armed_raw = 0;
	chan6_raw = 0;
	
	sensor_sample_count = 0;
	sensor_error_count = 0;
	gyro_x_raw = 0;
	gyro_y_raw = 0;
	gyro_z_raw = 0;
	accel_x_raw = 0;
	accel_y_raw = 0;
	accel_z_raw = 0;
	temperature_raw = 0;
	
	gyro_x_dc = 0;
	gyro_y_dc = 0;
	gyro_z_dc = 0;
	
	throttle = 0;
	aileron = 0;
	elevator = 0;
	rudder = 0;
	
	expo_scale = 1.0;
	
	error_pitch_int = 0;
	error_roll_int = 0;
	error_yaw_int = 0;
	
	vbat_acc = 15.0f / VBAT_ALPHA;
	vbat = 15.0f;
	vbat_sample_count = 0;
	
	
	/* Register init -----------------------------------*/
	
	flash_r = (uint32_t*)REG_FLASH_ADDR;
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
			uint32_to_float(&reg_default, &regf[i]);
		else
			reg[i] = reg_default;
	}
	
	/* Clock enable --------------------------------------------------*/
	
	// UART clock enable
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	// SPI clock enable
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	// Timer clock enable
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN | RCC_APB1ENR_TIM6EN | RCC_APB1ENR_TIM7EN;
	RCC->APB2ENR |= RCC_APB2ENR_TIM15EN | RCC_APB2ENR_TIM16EN;
	// System configuration controller clock enable (to manage external interrupt line connection to GPIOs)
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	// DMA clock enable
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	// GPIO clock enable 
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;
	// ADC clock enable
	RCC->AHBENR |= RCC_AHBENR_ADC12EN;
	// USB clock enable
	RCC->APB1ENR |= RCC_APB1ENR_USBEN;
	
	/* Receiver bind -------------------------------------------*/
	
	if (REG_RECEIVER_BIND)
	{
		wait_ms(100);
		
		GPIOB->BSRR = GPIO_BSRR_BS_4;
		GPIOB->MODER &= ~GPIO_MODER_MODER4_Msk;
		GPIOB->MODER |= GPIO_MODER_MODER4_0; // OUT
		GPIOB->OTYPER &= ~GPIO_OTYPER_OT_4; // PP
		GPIOB->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR4_Msk; // Low speed
		GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR4_Msk; // No PU/PD
		
		for (i=0; i<9; i++)
		{
			GPIOB->BSRR = GPIO_BSRR_BR_4;
			wait_ms(1);
			GPIOB->BSRR = GPIO_BSRR_BS_4;
			wait_ms(1);
		}
	}
	
	/* GPIO ------------------------------------------------*/
	
	// MODER: 00:IN, 01:OUT, 10:AF, 11:analog
	// PUPDR: 00:float 01:PU, 10:PD
	// OTYPER: 0:PP, 1:OD
	// OSPEEDR: x0:low 01:mid 11:high
	
	// Reset non-zero registers
	GPIOA->MODER = 0;
	GPIOA->PUPDR = 0;
	GPIOA->OSPEEDR = 0;
	GPIOB->MODER = 0;
	GPIOB->PUPDR = 0;
	GPIOB->OSPEEDR = 0;
	
	// A0 : Beeper
	#ifndef DEBUG
		GPIOA->MODER |= GPIO_MODER_MODER0_0;
	#endif
	// A1 : Motor 5, to TIM2_CH2, AF1, NOT USED
	// A2 : Motor 6, to TIM2_CH3, AF1, NOT USED
	// A3 : PWM7
	#ifdef DEBUG
		GPIOA->MODER |= GPIO_MODER_MODER3_0;
	#endif
	// A4 : Motor 2, to TIM3_CH2, AF2
	// A6 : Motor 1, to TIM3_CH1, AF2
	GPIOA->MODER |= GPIO_MODER_MODER4_1 | GPIO_MODER_MODER6_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4 | GPIO_OSPEEDER_OSPEEDR6;
	GPIOA->AFR[0] |= (2 << GPIO_AFRL_AFRL4_Pos) | (2 << GPIO_AFRL_AFRL6_Pos);
	// A5 : VBAT/10
	GPIOA->MODER |= GPIO_MODER_MODER5;
	// A7 : PPM
	// A8 : PWM8
	#ifdef DEBUG
		GPIOA->MODER |= GPIO_MODER_MODER8_0;
	#endif
	// A9 : I2C2 SCL, AF4
	// A10: I2C2 SDA, AF4
	GPIOA->MODER |= GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9 | GPIO_OSPEEDER_OSPEEDR10;
	GPIOA->OTYPER |= GPIO_OTYPER_OT_9 | GPIO_OTYPER_OT_10;
	GPIOA->AFR[1] |= (4 << GPIO_AFRH_AFRH1_Pos) | (4 << GPIO_AFRH_AFRH2_Pos);
	// A11: USB_DM, AF14
	// A12: USB_DP, AF14
	GPIOA->MODER |= GPIO_MODER_MODER11_1 | GPIO_MODER_MODER12_1;
	GPIOA->AFR[1] |= (14 << GPIO_AFRH_AFRH3_Pos) | (14 << GPIO_AFRH_AFRH4_Pos);
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR11 | GPIO_OSPEEDER_OSPEEDR12;
	// A15: MPU interrupt
	// B0 : Motor 3, to TIM3, AF2
	// B1 : Motor 4, to TIM3, AF2
	GPIOB->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0 | GPIO_OSPEEDER_OSPEEDR1;
	GPIOB->AFR[0] |= (2 << GPIO_AFRL_AFRL0_Pos) | (2 << GPIO_AFRL_AFRL1_Pos);
	// B3 : UART2 Tx, AF7, NOT USED
	// B4 : UART2 Rx, AF7
	GPIOB->MODER |= GPIO_MODER_MODER4_1;
	GPIOB->AFR[0] |= 7 << GPIO_AFRL_AFRL4_Pos;
	// B5 : Red LED, need open-drain (external pull-up)
	GPIOB->MODER |= GPIO_MODER_MODER5_0;
	GPIOB->OTYPER |= GPIO_OTYPER_OT_5;
	// B6 : UART1 Tx, NOT USED
	// B7 : UART1 Rx, NOT USED
	// B10: UART3 Tx, AF7, NOT USED
	// B11: UART3 Rx, AF7, NOT USED
	
	/* DMA ---------------------------------------------------*/
	
	// I2C2 Tx
	DMA1_Channel4->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_PL_0;
	DMA1_Channel4->CMAR = (uint32_t)i2c2_tx_buffer;
	DMA1_Channel4->CPAR = (uint32_t)&(I2C2->TXDR);

	// I2C2 Rx
	DMA1_Channel5->CCR = DMA_CCR_TCIE | DMA_CCR_MINC | DMA_CCR_PL;
	DMA1_Channel5->CMAR = (uint32_t)i2c2_rx_buffer;
	DMA1_Channel5->CPAR = (uint32_t)&(I2C2->RXDR);

	// UART2 Rx
	DMA1_Channel6->CCR = DMA_CCR_TCIE | DMA_CCR_MINC;
	DMA1_Channel6->CMAR = (uint32_t)uart2_rx_buffer;
	DMA1_Channel6->CPAR = (uint32_t)&(USART2->RDR);
	
	/* Timers --------------------------------------------------*/
	
	// One-pulse mode for OneShot125 
	TIM3->CR1 = TIM_CR1_OPM;
	TIM3->PSC = 8; // 0.125us
	TIM3->ARR = MOTOR_MAX + 1;
	TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
	TIM3->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_0;
	TIM3->CCMR2 = TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_0 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_0;
	
	// Beeper
	TIM4->PSC = 35999; // 0.5ms
	TIM4->ARR = BEEPER_PERIOD*4;
	TIM4->CR1 = TIM_CR1_CEN;
	
	// Receiver timeout
	TIM6->PSC = 35999; // 0.5ms
	TIM6->ARR = 65535;
	TIM6->CR1 = TIM_CR1_CEN;
	
	// Sensor to motor processing time
	TIM7->PSC = 71; // 1us
	TIM7->ARR = 65535;
	TIM7->CR1 = TIM_CR1_CEN;
	
	// SPI transaction time
	TIM15->PSC = 71; // 1us
	TIM15->ARR = 65535;
	TIM15->CR1 = TIM_CR1_CEN;
	
	// VBAT
	TIM16->PSC = 35999; // 0.5ms
	TIM16->ARR = 65535;
	TIM16->CR1 = TIM_CR1_CEN;
	
	/* UART ---------------------------------------------------*/
	
	USART2->BRR = 0x271; // 115200 bps
	USART2->CR1 = USART_CR1_UE | USART_CR1_RE;
	USART2->CR3 = USART_CR3_DMAR;
	UART2_RX_DMA_EN
	
	/* I2C ------------------------------------------------------*/
	
	I2C2->CR1 = I2C_CR1_TCIE | I2C_CR1_TXDMAEN | I2C_CR1_RXDMAEN;
	I2C2->CR2 = 104 << (1+I2C_CR2_SADD_Pos);
	I2C2->TIMINGR = (5 << I2C_TIMINGR_PRESC_Pos) | (9 << I2C_TIMINGR_SCLL_Pos) | (3 << I2C_TIMINGR_SCLH_Pos) | (3 << I2C_TIMINGR_SDADEL_Pos) | (3 << I2C_TIMINGR_SCLDEL_Pos); // 400kHz (Fast-mode)
	I2C2->CR1 |= I2C_CR1_PE;
	
	/* ADC -----------------------------------------------------*/
	
	// Select AHB clock as ADC clock 
	ADC12_COMMON->CCR = ADC12_CCR_CKMODE_0;
	
	// Regulator startup
	ADC2->CR = 0;
	ADC2->CR = ADC_CR_ADVREGEN_0;
	wait_ms(1);
	
	// Calibration
	ADC2->CR |= ADC_CR_ADCAL;
	while (ADC2->CR & ADC_CR_ADCAL) {}
	
	// Enable
	ADC2->CR |= ADC_CR_ADEN;
	while ((ADC2->ISR & ADC_ISR_ADRDY) == 0) {}
	
	ADC2->SMPR1 = 4 << ADC_SMPR1_SMP2_Pos;
	ADC2->SQR1 = 2 << ADC_SQR1_SQ1_Pos;
	
	/* Interrupts ---------------------------------------------------*/
	
	// MPU interrrupt
	SYSCFG->EXTICR[3] = SYSCFG_EXTICR4_EXTI15_PA;
	EXTI->RTSR |= EXTI_RTSR_TR15;
	EXTI->IMR = EXTI_IMR_MR15;
	
	NVIC_EnableIRQ(EXTI15_10_IRQn);
	NVIC_EnableIRQ(I2C2_EV_IRQn);
	NVIC_EnableIRQ(DMA1_Channel5_IRQn);
	NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	NVIC_EnableIRQ(USB_LP_CAN_RX0_IRQn);
	
	NVIC_SetPriority(EXTI15_10_IRQn,2);
	NVIC_SetPriority(I2C2_EV_IRQn,1);
	NVIC_SetPriority(DMA1_Channel5_IRQn,3);
	NVIC_SetPriority(DMA1_Channel6_IRQn,4);
	NVIC_SetPriority(USB_LP_CAN_RX0_IRQn,5);

	/* USB ----------------------------------------------------------*/
	
	USB->CNTR &= ~USB_CNTR_PDWN;
	wait_ms(1);
	
	USBD_Init(&USBD_device_handler, &USBD_VCP_descriptor, 0);
	USBD_RegisterClass(&USBD_device_handler, &USBD_CDC);
	USBD_CDC_RegisterInterface(&USBD_device_handler, &USBD_CDC_IF_fops);
	USBD_Start(&USBD_device_handler);
	
	/* MPU init ----------------------------------------------------*/
	
	I2cWrite(MPU_PWR_MGMT_1, MPU_PWR_MGMT_1__DEVICE_RST);
	wait_ms(100);
	I2cWrite(MPU_PWR_MGMT_1, MPU_PWR_MGMT_1__CLKSEL(1)); // Get MPU out of sleep, set CLK = gyro X clock
	wait_ms(100);
	I2cWrite(MPU_SMPLRT_DIV, 7); // Sample rate = Fs/(x+1)
	//I2cWrite(MPU_CFG, MPU_CFG__DLPF_CFG(1)); // Filter ON => Fs=1kHz, else 8kHz
	I2cWrite(MPU_GYRO_CFG, MPU_GYRO_CFG__FS_SEL(3)); // Full scale = +/-2000 deg/s
	I2cWrite(MPU_ACCEL_CFG, MPU_ACCEL_CFG__AFS_SEL(3)); // Full scale = +/- 16g
	wait_ms(100); // wait for filter to settle
	I2cWrite(MPU_INT_EN, MPU_INT_EN__DATA_RDY_EN);
	
	/* Gyro calibration ---------------------------------------*/
	
	while (sensor_sample_count < 1000)
	{
		if (FLAG & FLAG__SENSOR)
		{
			FLAG &= ~FLAG__SENSOR;
			
			gyro_x_dc += (float)gyro_x_raw;
			gyro_y_dc += (float)gyro_y_raw;
			gyro_z_dc += (float)gyro_z_raw;
		}
		__WFI();
	}
	
	gyro_x_dc = gyro_x_dc / 1000.0f;
	gyro_y_dc = gyro_y_dc / 1000.0f;
	gyro_z_dc = gyro_z_dc / 1000.0f;
	
	gyro_x_scale = 0.061035f;
	gyro_y_scale = 0.061035f;
	gyro_z_scale = 0.061035f;
	
	accel_x_scale = 0.00048828f;
	accel_y_scale = 0.00048828f;
	accel_z_scale = 0.00048828f;
	
	/* Main loop -----------------------------------------------*/
	
	while (1)
	{
		// Reset processing time
		TIM7->CNT = 0;
		#ifdef DEBUG
			GPIOA->BSRR = GPIO_BSRR_BS_8;
		#endif
		
		// Actions only needed on a register write
		if (FLAG & FLAG__REG)
		{
			FLAG &= ~FLAG__REG;
			
			// Adapt SPI clock frequency
			if (REG_CTRL__MPU_HOST_CTRL)
				SPI2->CR1 |= (5 << SPI_CR1_BR_Pos); // 562.5kHz
			else
				SPI2->CR1 &= ~SPI_CR1_BR_Msk; // 18MHz
			
			// Manual contol of LED
			if (REG_CTRL__LED_SELECT == 0)
				GPIOB->BSRR = GPIO_BSRR_BS_5;
			else if (REG_CTRL__LED_SELECT == 1)
				GPIOB->BSRR = GPIO_BSRR_BR_5;
			
			// Expo
			x = REG_EXPO;
			expo_scale = x;
			x = x * REG_EXPO;
			expo_scale += x / 2.0f;
			x = x * REG_EXPO;
			expo_scale += x / 6.0f;
			x = x * REG_EXPO;
			expo_scale += x / 24.0f;
			x = x * REG_EXPO;
			expo_scale += x / 120.0f;
			x = x * REG_EXPO;
			expo_scale += x / 720.0f;
		}
		
		// Process commands
		if (FLAG & FLAG__COMMAND)
		{
			FLAG &= ~FLAG__COMMAND;
			
			TIM6->CNT = 0; // Reset timer
			
			if (REG_CTRL__LED_SELECT == 3)
			{
				if ((command_frame_count & 0xFF) == 0)
					GPIOB->BSRR = GPIO_BSRR_BR_5;
				else if ((command_frame_count & 0xFF) == 128)
					GPIOB->BSRR = GPIO_BSRR_BS_5;
			}
			
			throttle = (float)throttle_raw / 2048.0f;
			aileron = (float)((int16_t)aileron_raw - 1024) / 1024.0f;
			elevator = (float)((int16_t)elevator_raw - 1024) / 1024.0f;
			rudder = (float)((int16_t)rudder_raw - 1024) / 1024.0f;
			
			if (REG_EXPO > 0.0f)
			{
				if (aileron >= 0)
					aileron = expo(aileron, expo_scale);
				else
					aileron = -expo(-aileron, expo_scale);
				
				if (elevator >= 0)
					elevator = expo(elevator, expo_scale);
				else
					elevator = -expo(-elevator, expo_scale);
				
				if (rudder >= 0)
					rudder = expo(rudder, expo_scale);
				else
					rudder = -expo(-rudder, expo_scale);
			}
			
			// Set flag to indicate data is ready for debug
			if (REG_DEBUG)
				FLAG |= FLAG__DEBUG; 
		}
		
		// Disarm if receiver timeout
		if (TIM6->CNT > TIMEOUT_COMMAND*2)
			armed_raw = 0;
		
		// Process sensors
		if (FLAG & FLAG__SENSOR)
		{
			FLAG &= ~FLAG__SENSOR;
			
			if (REG_CTRL__LED_SELECT == 2)
			{
				if ((sensor_sample_count & 0x3FF) == 0)
					GPIOB->BSRR = GPIO_BSRR_BR_5;
				else if ((sensor_sample_count & 0x3FF) == 512)
					GPIOB->BSRR = GPIO_BSRR_BS_5;
			}
			
			// Remove DC and scale
			gyro_x = ((float)gyro_x_raw - gyro_x_dc) * gyro_x_scale;
			gyro_y = ((float)gyro_y_raw - gyro_y_dc) * gyro_y_scale;
			gyro_z = ((float)gyro_z_raw - gyro_z_dc) * gyro_z_scale;
			accel_x = (float)accel_x_raw * accel_x_scale;
			accel_y = (float)accel_y_raw * accel_y_scale;
			accel_z = (float)accel_z_raw * accel_z_scale;
			temperature = (float)temperature_raw / 340.0f + 36.53f;
			
			// Previous error
			error_pitch_z = error_pitch;
			error_roll_z = error_roll;
			error_yaw_z = error_yaw;
			
			// Current error
			error_pitch = (elevator * (float)REG_COMMAND_RATE) - gyro_y;
			error_roll = (aileron * (float)REG_COMMAND_RATE) - gyro_x;
			error_yaw = (rudder * (float)REG_COMMAND_RATE) - gyro_z;
			
			// Integrate error
			if ((armed_raw < 1024) && REG_CTRL__RESET_INTEGRAL_ON_ARMED)
			{
				error_pitch_int = 0.0f;
				error_roll_int = 0.0f;
				error_yaw_int = 0.0f;
			}
			else
			{
				error_pitch_int += error_pitch;
				error_roll_int += error_roll;
				error_yaw_int += error_yaw;
			}
			
			// Clip the integral
			pitch_i = error_pitch_int * REG_PITCH_I;
			roll_i = error_roll_int * REG_ROLL_I;
			yaw_i = error_yaw_int * REG_YAW_I;
			
			if (pitch_i > INTEGRAL_MAX)
				error_pitch_int = INTEGRAL_MAX / REG_PITCH_I;
			else if (pitch_i < -INTEGRAL_MAX)
				error_pitch_int = -INTEGRAL_MAX / REG_PITCH_I;
			if (roll_i > INTEGRAL_MAX)
				error_roll_int = INTEGRAL_MAX / REG_ROLL_I;
			else if (roll_i < -INTEGRAL_MAX)
				error_roll_int = -INTEGRAL_MAX / REG_ROLL_I;
			if (yaw_i > INTEGRAL_MAX)
				error_yaw_int = INTEGRAL_MAX / REG_YAW_I;
			else if (yaw_i < -INTEGRAL_MAX)
				error_yaw_int = -INTEGRAL_MAX / REG_YAW_I;
			
			// PID
			pitch_before_tpa = error_pitch * REG_PITCH_P + error_pitch_int * REG_PITCH_I - (error_pitch - error_pitch_z) * REG_PITCH_D;
			roll_before_tpa = error_roll * REG_ROLL_P + error_roll_int * REG_ROLL_I - (error_roll - error_roll_z) * REG_ROLL_D;
			yaw_before_tpa = error_yaw * REG_YAW_P + error_yaw_int * REG_YAW_I - (error_yaw - error_yaw_z) * REG_YAW_D;
			
			// Throttle PIDs adjustement
			if (throttle > REG_TPA_THRESHOLD)
			{
				pitch = pitch_before_tpa * (1.0f - REG_TPA_SLOPE * (throttle - REG_TPA_THRESHOLD));
				roll = roll_before_tpa * (1.0f - REG_TPA_SLOPE * (throttle - REG_TPA_THRESHOLD));
				yaw = yaw_before_tpa * (1.0f - REG_TPA_SLOPE * (throttle - REG_TPA_THRESHOLD));
			}
			else
			{
				pitch = pitch_before_tpa;
				roll = roll_before_tpa;
				yaw = yaw_before_tpa;
			}
			
			motor[0] = (throttle * (float)REG_THROTTLE_RANGE) + roll + pitch - yaw;
			motor[1] = (throttle * (float)REG_THROTTLE_RANGE) + roll - pitch + yaw;
			motor[2] = (throttle * (float)REG_THROTTLE_RANGE) - roll - pitch - yaw;
			motor[3] = (throttle * (float)REG_THROTTLE_RANGE) - roll + pitch + yaw;
			
			for (i=0; i<4; i++)
			{
				motor_clip[i] = (int32_t)motor[i] + (int32_t)REG_MOTOR_ARMED;
				
				if (motor_clip[i] < (int32_t)REG_MOTOR_START)
					motor_clip[i] = (int32_t)REG_MOTOR_START;
				else if (motor_clip[i] > (int32_t)MOTOR_MAX)
					motor_clip[i] = (int32_t)MOTOR_MAX;
				
				if (REG_MOTOR_TEST__SELECT)
					motor_raw[i] = (REG_MOTOR_TEST__SELECT & (1 << i)) ? (uint32_t)REG_MOTOR_TEST__VALUE : 0;
				else if (armed_raw < 1024)
					motor_raw[i] = 0;
				else
					motor_raw[i] = (uint32_t)motor_clip[i];
			}
			
			TIM3->CCR1 = MOTOR_MAX + 1 - MOTOR_MIN - (motor_raw[0]>>1);
			TIM3->CCR2 = MOTOR_MAX + 1 - MOTOR_MIN - (motor_raw[1]>>1);
			TIM3->CCR3 = MOTOR_MAX + 1 - MOTOR_MIN - (motor_raw[2]>>1);
			TIM3->CCR4 = MOTOR_MAX + 1 - MOTOR_MIN - (motor_raw[3]>>1);
			TIM3->CR1 |= TIM_CR1_CEN;
			
			// Set flag to indicate data is ready for debug
			if (REG_DEBUG)
				FLAG |= FLAG__DEBUG;
		}
		
		// EMERGENCY: Motor OFF when no sensor samples
		if (TIM15->CNT > TIMEOUT_SENSOR)
		{
			TIM15->CNT = 0; // Reset timer
			
			TIM3->CCR1 = MOTOR_MAX + 1 - MOTOR_MIN;
			TIM3->CCR2 = MOTOR_MAX + 1 - MOTOR_MIN;
			TIM3->CCR3 = MOTOR_MAX + 1 - MOTOR_MIN;
			TIM3->CCR4 = MOTOR_MAX + 1 - MOTOR_MIN;
			TIM3->CR1 |= TIM_CR1_CEN;
		}
		
		// VBAT
		if (TIM16->CNT > VBAT_PERIOD*2)
		{
			TIM16->CNT = 0; // Reset timer
			
			vbat_sample_count++;
			
			if (ADC2->ISR & ADC_ISR_EOC)
				vbat = (float)ADC2->DR * ADC_SCALE;
			
			vbat_acc = vbat_acc * (1.0f - VBAT_ALPHA) + vbat;
			vbat = vbat_acc * VBAT_ALPHA;
			
			REG_VBAT = vbat;
			
			ADC2->CR |= ADC_CR_ADSTART;
			
			if (REG_DEBUG)
				FLAG |= FLAG__DEBUG;
		}
		
		// Beeper
		if (((REG_VBAT < REG_VBAT_MIN) && (REG_VBAT > VBAT_THRESHOLD)) || (chan6_raw > 1024) || (TIM6->CNT > TIMEOUT_COMMAND*2) || REG_CTRL__BEEP_TEST)
		{
			if (TIM4->CNT > BEEPER_PERIOD*2)
				GPIOA->BSRR = GPIO_BSRR_BS_0;
			else
				GPIOA->BSRR = GPIO_BSRR_BR_0;
		}
		else
			GPIOA->BSRR = GPIO_BSRR_BR_0;
		
		// Debug
		if (FLAG & FLAG__DEBUG)
		{
			FLAG &= ~FLAG__DEBUG;
			
			switch (REG_DEBUG)
			{
				case 1:
				{
					USBD_CDC_IF_tx_buffer[ 0] = (uint8_t) sensor_sample_count;
					USBD_CDC_IF_tx_buffer[ 1] = (uint8_t)(sensor_sample_count >> 8);
					USBD_CDC_IF_tx_buffer[ 2] = (uint8_t) gyro_x_raw;
					USBD_CDC_IF_tx_buffer[ 3] = (uint8_t)(gyro_x_raw >> 8);
					USBD_CDC_IF_tx_buffer[ 4] = (uint8_t) gyro_y_raw;
					USBD_CDC_IF_tx_buffer[ 5] = (uint8_t)(gyro_y_raw >> 8);
					USBD_CDC_IF_tx_buffer[ 6] = (uint8_t) gyro_z_raw;
					USBD_CDC_IF_tx_buffer[ 7] = (uint8_t)(gyro_z_raw >> 8);
					USBD_CDC_IF_tx_buffer[ 8] = (uint8_t) accel_x_raw;
					USBD_CDC_IF_tx_buffer[ 9] = (uint8_t)(accel_x_raw >> 8);
					USBD_CDC_IF_tx_buffer[10] = (uint8_t) accel_y_raw;
					USBD_CDC_IF_tx_buffer[11] = (uint8_t)(accel_y_raw >> 8);
					USBD_CDC_IF_tx_buffer[12] = (uint8_t) accel_z_raw;
					USBD_CDC_IF_tx_buffer[13] = (uint8_t)(accel_z_raw >> 8);
					USBD_CDC_IF_tx_buffer[14] = (uint8_t) temperature_raw;
					USBD_CDC_IF_tx_buffer[15] = (uint8_t)(temperature_raw >> 8);
					USB_TX(16)
					break;
				}
				case 2:
				{
					USBD_CDC_IF_tx_buffer[0] = (uint8_t) sensor_sample_count;
					USBD_CDC_IF_tx_buffer[1] = (uint8_t)(sensor_sample_count >> 8);
					float_to_bytes(&gyro_x, &USBD_CDC_IF_tx_buffer[2]);
					float_to_bytes(&gyro_y, &USBD_CDC_IF_tx_buffer[6]);
					float_to_bytes(&gyro_z, &USBD_CDC_IF_tx_buffer[10]);
					float_to_bytes(&accel_x, &USBD_CDC_IF_tx_buffer[14]);
					float_to_bytes(&accel_y, &USBD_CDC_IF_tx_buffer[18]);
					float_to_bytes(&accel_z, &USBD_CDC_IF_tx_buffer[22]);
					float_to_bytes(&temperature, &USBD_CDC_IF_tx_buffer[26]);
					USB_TX(30)
					break;
				}
				case 3:
				{
					USBD_CDC_IF_tx_buffer[0] = (uint8_t) vbat_sample_count;
					USBD_CDC_IF_tx_buffer[1] = (uint8_t)(vbat_sample_count >> 8);
					float_to_bytes(&vbat, &USBD_CDC_IF_tx_buffer[2]);
					USB_TX(6)
					break;
				}
				case 4:
				{
					USBD_CDC_IF_tx_buffer[ 0] = (uint8_t) command_frame_count;
					USBD_CDC_IF_tx_buffer[ 1] = (uint8_t)(command_frame_count >> 8);
					USBD_CDC_IF_tx_buffer[ 2] = (uint8_t) throttle_raw;
					USBD_CDC_IF_tx_buffer[ 3] = (uint8_t)(throttle_raw >> 8);
					USBD_CDC_IF_tx_buffer[ 4] = (uint8_t) aileron_raw;
					USBD_CDC_IF_tx_buffer[ 5] = (uint8_t)(aileron_raw>> 8);
					USBD_CDC_IF_tx_buffer[ 6] = (uint8_t) elevator_raw;
					USBD_CDC_IF_tx_buffer[ 7] = (uint8_t)(elevator_raw>> 8);
					USBD_CDC_IF_tx_buffer[ 8] = (uint8_t) rudder_raw;
					USBD_CDC_IF_tx_buffer[ 9] = (uint8_t)(rudder_raw >> 8);
					USBD_CDC_IF_tx_buffer[10] = (uint8_t) armed_raw;
					USBD_CDC_IF_tx_buffer[11] = (uint8_t)(armed_raw >> 8);
					USBD_CDC_IF_tx_buffer[12] = (uint8_t) chan6_raw;
					USBD_CDC_IF_tx_buffer[13] = (uint8_t)(chan6_raw >> 8);
					USB_TX(14)
					break;
				}
				case 5:
				{
					USBD_CDC_IF_tx_buffer[ 0] = (uint8_t) command_frame_count;
					USBD_CDC_IF_tx_buffer[ 1] = (uint8_t)(command_frame_count >> 8);
					float_to_bytes(&throttle, &USBD_CDC_IF_tx_buffer[2]);
					float_to_bytes(&aileron, &USBD_CDC_IF_tx_buffer[6]);
					float_to_bytes(&elevator, &USBD_CDC_IF_tx_buffer[10]);
					float_to_bytes(&rudder, &USBD_CDC_IF_tx_buffer[14]);
					USB_TX(18)
					break;
				}
				case 6:
				{
					USBD_CDC_IF_tx_buffer[0] = (uint8_t) sensor_sample_count;
					USBD_CDC_IF_tx_buffer[1] = (uint8_t)(sensor_sample_count >> 8);
					float_to_bytes(&pitch, &USBD_CDC_IF_tx_buffer[2]);
					float_to_bytes(&roll, &USBD_CDC_IF_tx_buffer[6]);
					float_to_bytes(&yaw, &USBD_CDC_IF_tx_buffer[10]);
					USB_TX(14)
					break;
				}
				case 7:
				{
					USBD_CDC_IF_tx_buffer[0] = (uint8_t) sensor_sample_count;
					USBD_CDC_IF_tx_buffer[1] = (uint8_t)(sensor_sample_count >> 8);
					for (i=0; i<4; i++)
						float_to_bytes(&motor[i], &USBD_CDC_IF_tx_buffer[i*4+2]);
					USB_TX(18)
					break;
				}
				case 8:
				{
					USBD_CDC_IF_tx_buffer[0] = (uint8_t) sensor_sample_count;
					USBD_CDC_IF_tx_buffer[1] = (uint8_t)(sensor_sample_count >> 8);
					for (i=0; i<4; i++)
					{
						USBD_CDC_IF_tx_buffer[i*2+2] = (uint8_t) motor_raw[i];
						USBD_CDC_IF_tx_buffer[i*2+3] = (uint8_t)(motor_raw[i] >> 8);
					}
					USB_TX(10)
					break;
				}
			}
			
			REG_DEBUG = 0;
		}
		
		// Record processing time
		time_process = TIM7->CNT;
		if ((REG_CTRL__TIME_MAXHOLD == 0) || ((time_process > REG_TIME__PROCESS) && REG_CTRL__TIME_MAXHOLD))
		{
			REG_TIME &= 0x0000FFFF;
			REG_TIME |= (uint32_t)time_process << 16;
		}
		#ifdef DEBUG
			GPIOA->BSRR = GPIO_BSRR_BR_8;
		#endif
		
		// Wait for interrupts
		__WFI();
	}
}
