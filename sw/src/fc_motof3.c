#include "stm32f3xx.h"
#include "usbd_cdc_if.h"
#include "fc_reg.h"
#include "mpu_reg.h"

//######## DEFINTIONS ########

#define MOTOR_PULSE_MAX 2100 // us
#define VBAT_ALPHA 0.0002f
#define BEEPER
#define BEEPER_PERIOD 250 // ms
#define TIMEOUT_COMMAND 220 // ms

#define FLAG_SENSOR 0x01
#define FLAG_SENSOR_DEBUG 0x02
#define FLAG_COMMAND 0x04
#define FLAG_COMMAND_DEBUG 0x08
#define FLAG_I2C_RD_WRN 0x10
#define FLAG_I2C_TRANSACTION_OK 0x20
#define FLAG_I2C_RETURN_ON_USB 0x40

#define REG_FLASH_ADDR 0x0803F800

//######## GLOBAL VARIABLES ########

uint8_t FLAG;

uint32_t tick;

uint8_t uart2_rx_buffer[16];
uint8_t i2c2_rx_buffer[14];
uint8_t i2c2_tx_buffer[2];

uint16_t sensor_sample_count;
int16_t gyro_x_raw;
int16_t gyro_y_raw;
int16_t gyro_z_raw;
int16_t accel_x_raw;
int16_t accel_y_raw;
int16_t accel_z_raw;
//int16_t temp_raw;
uint16_t sensor_error_count;

uint16_t command_frame_count;
uint8_t command_fades;
//uint8_t command_system;
uint16_t throttle_raw;
uint16_t aileron_raw;
uint16_t elevator_raw;
uint16_t rudder_raw;
uint16_t armed_raw;
uint16_t chan6_raw;
uint16_t command_error_count;

//######## PIVATE FUNCTIONS ########

void SetDmaI2c2Tx(uint8_t size)
{
	I2C2->CR1 &= ~I2C_CR1_PE;
	while (I2C2->CR1 & I2C_CR1_PE) {}
	
	DMA1_Channel4->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_PL_1;
	DMA1_Channel4->CMAR = (uint32_t)i2c2_tx_buffer;
	DMA1_Channel4->CPAR = (uint32_t)&(I2C2->TXDR);
	DMA1_Channel4->CNDTR = size;
	DMA1_Channel4->CCR |= DMA_CCR_EN;
	
	I2C2->CR1 |= I2C_CR1_PE;
	I2C2->CR2 &= ~(I2C_CR2_RD_WRN | I2C_CR2_NBYTES_Msk);
	I2C2->CR2 |= (I2C_CR2_NBYTES_Msk & (size << I2C_CR2_NBYTES_Pos)) | I2C_CR2_START;
	
	FLAG &= ~FLAG_I2C_TRANSACTION_OK;
}

void SetDmaI2c2Rx(uint8_t size)
{
	DMA1_Channel5->CCR = DMA_CCR_MINC | DMA_CCR_PL;
	DMA1_Channel5->CMAR = (uint32_t)i2c2_rx_buffer;
	DMA1_Channel5->CPAR = (uint32_t)&(I2C2->RXDR);
	DMA1_Channel5->CNDTR = size;
	DMA1_Channel5->CCR |= DMA_CCR_EN;
}

void SetDmaUart2Rx()
{
	DMA1_Channel6->CCR = DMA_CCR_TCIE | DMA_CCR_MINC | DMA_CCR_PL_0;
	DMA1_Channel6->CMAR = (uint32_t)uart2_rx_buffer;
	DMA1_Channel6->CPAR = (uint32_t)&(USART2->RDR);
	DMA1_Channel6->CNDTR = 16;
	DMA1_Channel6->CCR |= DMA_CCR_EN;
}

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
	SetDmaI2c2Tx(2);
	wait_ms(1);
}

uint8_t I2cRead(uint8_t addr)
{
	FLAG |= FLAG_I2C_RD_WRN;
	SetDmaI2c2Rx(1);
	i2c2_tx_buffer[0] = addr;
	SetDmaI2c2Tx(1);
	wait_ms(1);
	return i2c2_rx_buffer[0];
}

void uint32_to_float(uint32_t * b, float * f)
{
	union {
		float f;
		uint32_t b;
	} f2b;
	f2b.b = *b;
	*f = f2b.f;
}

void float_to_bytes(float * f, uint8_t * b)
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

void float_to_uint32(float * f, uint32_t * b)
{
	union {
		float f;
		uint32_t b;
	} f2b;
	f2b.f = *f;
	*b = f2b.b;
}

//######## System Interrupts ########

void SysTick_Handler()
{
	tick++;
}

//####### External Interrupts ########

void EXTI15_10_IRQHandler()
{
	EXTI->PR = EXTI_PR_PIF15; // Clear pending request
	
	sensor_sample_count++;
	
	if (REG_CTRL__MPU_HOST_CTRL == 0)
	{
		FLAG |= FLAG_I2C_RD_WRN;
		SetDmaI2c2Rx(14);
		i2c2_tx_buffer[0] = 59;
		SetDmaI2c2Tx(1);
	}
}

void I2C2_EV_IRQHandler()
{
	if (FLAG & FLAG_I2C_RD_WRN)
	{
		FLAG &= ~FLAG_I2C_RD_WRN;
		I2C2->CR2 &= ~I2C_CR2_NBYTES_Msk;
		I2C2->CR2 |= (I2C_CR2_NBYTES_Msk & (DMA1_Channel5->CNDTR << I2C_CR2_NBYTES_Pos)) | I2C_CR2_RD_WRN | I2C_CR2_START;
	}
	else
	{
		I2C2->CR2 |= I2C_CR2_STOP;
		
		if (((I2C2->ISR & (I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_NACKF)) == 0)
			&& ((DMA1->ISR & DMA_ISR_TEIF5) == 0)
			&& (DMA1->ISR & DMA_ISR_TCIF5))
		{
			FLAG |= FLAG_I2C_TRANSACTION_OK;
			if (REG_CTRL__MPU_HOST_CTRL == 0)
				FLAG |= FLAG_SENSOR;
		}
		else if (REG_CTRL__MPU_HOST_CTRL == 0)
			sensor_error_count++;
		
		if (FLAG & FLAG_I2C_RETURN_ON_USB)
		{
			FLAG &= ~FLAG_I2C_RETURN_ON_USB;
			
			USBD_CDC_Itf_tx_buffer[0] = i2c2_rx_buffer[0];
			USBD_CDC_SetTxBuffer(&USBD_device_handler, USBD_CDC_Itf_tx_buffer, 1);
			USBD_CDC_TransmitPacket(&USBD_device_handler);
		}
	}
}

void DMA1_Channel6_IRQHandler()
{
	int i;
	uint8_t chan_id;
	uint16_t servo;
	uint8_t error = 0;
	
	DMA1->IFCR = DMA_IFCR_CTCIF6; // clear flag
	
	if (DMA1->ISR & DMA_ISR_TEIF6)
		error = 1;
	if (USART2->ISR & (USART_ISR_ORE | USART_ISR_NE))
	{
		error = 1;
		USART2->ICR = USART_ICR_ORECF | USART_ICR_NCF;
	}
	
	command_frame_count++;
	
	if (error == 0)
	{
		command_fades = uart2_rx_buffer[0];
		//command_system = uart2_rx_buffer[1];
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
		
		FLAG |= FLAG_COMMAND;
	}
	else
		command_error_count++;
	
	SetDmaUart2Rx();
}

void USB_LP_CAN_RX0_IRQHandler(void)
{
	HAL_PCD_IRQHandler(&PCD_handler);
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
			USBD_CDC_Itf_tx_buffer[0] = (uint8_t)((val >> 24) & 0xFF);
			USBD_CDC_Itf_tx_buffer[1] = (uint8_t)((val >> 16) & 0xFF);
			USBD_CDC_Itf_tx_buffer[2] = (uint8_t)((val >>  8) & 0xFF);
			USBD_CDC_Itf_tx_buffer[3] = (uint8_t)((val >>  0) & 0xFF);
			USBD_CDC_SetTxBuffer(&USBD_device_handler, USBD_CDC_Itf_tx_buffer, 4);
			USBD_CDC_TransmitPacket(&USBD_device_handler);
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
			}
			break;
		}
		case 2: // I2C read to MPU
		{
			FLAG |= FLAG_I2C_RD_WRN | FLAG_I2C_RETURN_ON_USB;
			SetDmaI2c2Rx(1);
			i2c2_tx_buffer[0] = addr;
			SetDmaI2c2Tx(1);
			break;
		}
		case 3: // I2C write to MPU
		{
			i2c2_tx_buffer[0] = addr;
			i2c2_tx_buffer[1] = buffer[5];
			SetDmaI2c2Tx(2);
			break;
		}
		case 4: // Flash read
		{
			val = flash_r[addr];
			USBD_CDC_Itf_tx_buffer[0] = (uint8_t)((val >> 24) & 0xFF);
			USBD_CDC_Itf_tx_buffer[1] = (uint8_t)((val >> 16) & 0xFF);
			USBD_CDC_Itf_tx_buffer[2] = (uint8_t)((val >>  8) & 0xFF);
			USBD_CDC_Itf_tx_buffer[3] = (uint8_t)((val >>  0) & 0xFF);
			USBD_CDC_SetTxBuffer(&USBD_device_handler, USBD_CDC_Itf_tx_buffer, 4);
			USBD_CDC_TransmitPacket(&USBD_device_handler);
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

//######## MAIN #########

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
	
	float throttle;
	float aileron;
	float elevator;
	float rudder;
	float expo_a;
	float expo_b;
	float expo_c;
	
	float error_pitch;
	float error_roll;
	float error_yaw;
	float error_pitch_z;
	float error_roll_z;
	float error_yaw_z;
	float error_pitch_int;
	float error_roll_int;
	float error_yaw_int;
	float pitch;
	float roll;
	float yaw;
	
	float motor[4];
	int32_t motor_clip[4];
	uint16_t motor_raw[4];
	
	float vbat_acc;
	float vbat;
	
	uint16_t t;
	float x;
	float y;
	
	uint32_t gpiob_moder;
	uint32_t gpiob_pupdr;
	uint32_t gpiob_ospeedr;
	uint32_t gpiob_otyper;
	
	_Bool reg_flash_valid;
	uint32_t* flash_r;
	uint32_t reg_default;
	
	//####### REG INIT #######
	
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
	
	//####### VAR INIT #######
	
	FLAG = 0;
	
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
	
	gyro_x_dc = 0;
	gyro_y_dc = 0;
	gyro_z_dc = 0;
	
	throttle = 0;
	aileron = 0;
	elevator = 0;
	rudder = 0;
	
	expo_a = 0;
	expo_b = 0;
	expo_c = 0;
	
	error_pitch_int = 0;
	error_roll_int = 0;
	error_yaw_int = 0;
		
	vbat_acc = REG_VBAT_MAX / VBAT_ALPHA;
	vbat = REG_VBAT_MAX;
	
	//######## CLOCK ENABLE ##########
	
	// UART clock enable
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	// SPI clock enable
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	// Timer clock enable
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM6EN | RCC_APB1ENR_TIM7EN;
	// System configuration controller clock enable (to manage external interrupt line connection to GPIOs)
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	// DMA clock enable
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	// GPIO clock enable 
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;
	// ADC clock enable
	RCC->AHBENR |= RCC_AHBENR_ADC12EN;
	
	//####### GPIO ##########
	
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
	
	// A0 : Beeper, to TIM2 (AF1)
	#ifdef BEEPER
		GPIOA->MODER |= GPIO_MODER_MODER0_1;
		GPIOA->AFR[0] |= 1 << GPIO_AFRL_AFRL0_Pos;
	#endif
	// A4 : Motor 2, to TIM3, AF2
	// A6 : Motor 1, to TIM3, AF2
	GPIOA->MODER |= GPIO_MODER_MODER4_1 | GPIO_MODER_MODER6_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4 | GPIO_OSPEEDER_OSPEEDR6;
	GPIOA->AFR[0] |= (2 << GPIO_AFRL_AFRL4_Pos) | (2 << GPIO_AFRL_AFRL6_Pos);
	// A5 : VBAT/10
	GPIOA->MODER |= GPIO_MODER_MODER5;
	// A7 : Binding ON
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR7_1;
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
	// B8 : USB disconnect pin, need open-drain (external pull-up)
	GPIOB->MODER |= GPIO_MODER_MODER8_0;
	GPIOB->OTYPER |= GPIO_OTYPER_OT_8;
	// B10: UART3 Tx, AF7, NOT USED
	// B11: UART3 Rx, AF7, NOT USED
	
	//####### TIM ########
	
	// PWM for beeper
	TIM2->PSC = 35999; // 0.5ms
	TIM2->ARR = BEEPER_PERIOD*4;
	TIM2->CCR1 = BEEPER_PERIOD*2;
	TIM2->CCER = TIM_CCER_CC1E;
	TIM2->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0;
	
	// One-pulse mode for OneShot125 
	TIM3->CR1 = TIM_CR1_OPM;
	TIM3->PSC = 8; // 0.125us
	TIM3->ARR = MOTOR_PULSE_MAX;
	TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
	TIM3->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_0;
	TIM3->CCMR2 = TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_0 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_0;
	
	// Timer for receiver timeout
	TIM6->PSC = 35999; // 0.5ms
	TIM6->ARR = 65535;
	TIM6->CR1 = TIM_CR1_CEN;
	
	// Timer for loop time
	TIM7->PSC = 71; // 1us
	TIM7->ARR = 65535;
	TIM7->CR1 = TIM_CR1_CEN;
	
	//####### ESC CAL and RADIO BIND ########
	
	if (GPIOA->IDR & GPIO_IDR_7)
	{
		t = 0;
		gpiob_moder = GPIOB->MODER;
		gpiob_pupdr = GPIOB->PUPDR;
		gpiob_ospeedr = GPIOB->OSPEEDR;
		gpiob_otyper = GPIOB->OTYPER;
		
		while (GPIOA->IDR & GPIO_IDR_7)
		{
			// ESC calibration of max command value
			TIM3->CCR1 = MOTOR_PULSE_MAX - REG_MOTOR__MAX;
			TIM3->CCR2 = MOTOR_PULSE_MAX - REG_MOTOR__MAX;
			TIM3->CCR3 = MOTOR_PULSE_MAX - REG_MOTOR__MAX;
			TIM3->CCR4 = MOTOR_PULSE_MAX - REG_MOTOR__MAX;
			TIM3->CR1 |= TIM_CR1_CEN;
			
			// Receiver binding sequence
			if (t == 10)
			{
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
				GPIOB->MODER = gpiob_moder;
				GPIOB->PUPDR = gpiob_pupdr;
				GPIOB->OSPEEDR = gpiob_ospeedr;
				GPIOB->OTYPER = gpiob_otyper;
			}
			
			t++;
			wait_ms(10);
		}
		
		
		for (i=0; i<100; i++)
		{
			TIM3->CCR1 = MOTOR_PULSE_MAX - REG_MOTOR__MIN;
			TIM3->CCR2 = MOTOR_PULSE_MAX - REG_MOTOR__MIN;
			TIM3->CCR3 = MOTOR_PULSE_MAX - REG_MOTOR__MIN;
			TIM3->CCR4 = MOTOR_PULSE_MAX - REG_MOTOR__MIN;
			TIM3->CR1 |= TIM_CR1_CEN;
			wait_ms(10);
		}
	}
	
	//######## UART ##########
	
	USART2->BRR = 0x271; // 115200 bps
	USART2->CR1 = USART_CR1_UE | USART_CR1_RE;
	USART2->CR3 = USART_CR3_DMAR;
	SetDmaUart2Rx();
	
	//####### I2C #########
	
	I2C2->CR1 = I2C_CR1_TCIE | I2C_CR1_TXDMAEN | I2C_CR1_RXDMAEN;
	I2C2->CR2 = 104 << (1+I2C_CR2_SADD_Pos);
	I2C2->TIMINGR = (5 << I2C_TIMINGR_PRESC_Pos) | (9 << I2C_TIMINGR_SCLL_Pos) | (3 << I2C_TIMINGR_SCLH_Pos) | (3 << I2C_TIMINGR_SDADEL_Pos) | (3 << I2C_TIMINGR_SCLDEL_Pos); // 400kHz (Fast-mode)
	
	//####### ADC ########
	
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
	
	//####### INTERRUPT #######
	
	// MPU interrrupt
	SYSCFG->EXTICR[3] = SYSCFG_EXTICR4_EXTI15_PA;
	EXTI->RTSR |= EXTI_RTSR_TR15;
	//EXTI->IMR = EXTI_IMR_MR15; // To be enabled after MPU init
	
	NVIC_EnableIRQ(EXTI15_10_IRQn);
	NVIC_EnableIRQ(I2C2_EV_IRQn);
	NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	NVIC_EnableIRQ(USB_LP_CAN_RX0_IRQn);
	
	NVIC_SetPriority(EXTI15_10_IRQn,2);
	NVIC_SetPriority(I2C2_EV_IRQn,1);
	NVIC_SetPriority(DMA1_Channel6_IRQn,3);
	NVIC_SetPriority(USB_LP_CAN_RX0_IRQn,4);
	
	//######## USB ############
	
	USB->CNTR &= ~USB_CNTR_PDWN;
	wait_ms(1);
	
	USBD_Init(&USBD_device_handler, &USBD_VCP_descriptor, 0);
	USBD_RegisterClass(&USBD_device_handler, &USBD_CDC);
	USBD_CDC_RegisterInterface(&USBD_device_handler, &USBD_CDC_Itf_fops);
	USBD_Start(&USBD_device_handler);
	
	//####### MPU INIT #########
	
	I2cWrite(MPU_PWR_MGMT_1, MPU_PWR_MGMT_1__DEVICE_RST);
	wait_ms(100);
	I2cWrite(MPU_PWR_MGMT_1, 0); // Get MPU out of sleep
	wait_ms(100);
	I2cWrite(MPU_PWR_MGMT_1, MPU_PWR_MGMT_1__CLKSEL(1)); // Set CLK = gyro X clock
	wait_ms(100);
	//I2cWrite(MPU_SMPLRT_DIV, 0); // Sample rate = Fs/(x+1)
	I2cWrite(MPU_CFG, MPU_CFG__DLPF_CFG(1)); // Filter ON => Fs=1kHz, else 8kHz
	I2cWrite(MPU_GYRO_CFG, MPU_GYRO_CFG__FS_SEL(3)); // Full scale = +/-2000 deg/s
	I2cWrite(MPU_ACCEL_CFG, MPU_ACCEL_CFG__AFS_SEL(3)); // Full scale = +/- 16g
	wait_ms(100); // wait for filter to settle
	I2cWrite(MPU_INT_EN, MPU_INT_EN__DATA_RDY_EN);
	
	EXTI->IMR = EXTI_IMR_MR15;
	
	//####### GYRO calibration ####### 
	
	while (sensor_sample_count < 1000)
	{
		if (FLAG & FLAG_SENSOR)
		{
			FLAG &= ~FLAG_SENSOR;
			
			gyro_x_dc += (float)gyro_x_raw;
			gyro_y_dc += (float)gyro_y_raw;
			gyro_z_dc += (float)gyro_z_raw;
			
			// Blink LED during calibration
			if ((sensor_sample_count & 0x7F) == 0)
				GPIOB->BSRR = GPIO_BSRR_BS_5;
			else if ((sensor_sample_count & 0x7F) == 64)
				GPIOB->BSRR = GPIO_BSRR_BR_5;
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
	
	//####### END OF INIT TASKS #####
	
	// Disable Systick interrupt, not needed anymore (but can still use COUNTFLAG)
	SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
	
	//######## MAIN LOOP #########
	
	while (1)
	{
		// Reset loop timer
		TIM7->EGR = TIM_EGR_UG;
		
		// Process commands
		if (FLAG & FLAG_COMMAND)
		{
			FLAG &= ~FLAG_COMMAND;
			FLAG |= FLAG_COMMAND_DEBUG; // Set flag to indicate data is ready for debug
			
			TIM6->EGR = TIM_EGR_UG; // Reset timer
			
			if (REG_CTRL__LED_SELECT == 3)
			{
				if ((command_frame_count & 0xFF) == 0)
					GPIOB->BSRR = GPIO_BSRR_BR_5;
				else if ((command_frame_count & 0xFF) == 128)
					GPIOB->BSRR = GPIO_BSRR_BS_5;
			}
			
			REG_ERROR &= 0x0000FFFF;
			REG_ERROR |= ((uint32_t)command_fades + (uint32_t)command_error_count) << 16; 
			
			throttle = (float)throttle_raw / 2048.0f;
			aileron = (float)((int16_t)aileron_raw - 1024) / 1024.0f;
			elevator = (float)((int16_t)elevator_raw - 1024) / 1024.0f;
			rudder = (float)((int16_t)rudder_raw - 1024) / 1024.0f;
			
			if (REG_EXPO > 0.0f)
			{
				// Update expo coefficient if register change
				if (expo_a != REG_EXPO)
				{
					expo_a = REG_EXPO;
					expo_b = expo_a * expo_a;
					expo_c = 1 / ((1.0f + expo_a) * (1.0f + expo_a) - expo_b);
				}
				
				x = aileron;
				if (x >= 0)
				{
					x += expo_a;
					y = x * x - expo_b;
				}
				else
				{
					x -= expo_a;
					y = -(x * x - expo_b);
				}
				aileron = y * expo_c;
				
				x = elevator;
				if (x >= 0)
				{
					x += expo_a;
					y = x * x - expo_b;
				}
				else
				{
					x -= expo_a;
					y = -(x * x - expo_b);
				}
				elevator = y * expo_c;
				
				x = rudder;
				if (x >= 0)
				{
					x += expo_a;
					y = x * x - expo_b;
				}
				else
				{
					x -= expo_a;
					y = -(x * x - expo_b);
				}
				rudder = y * expo_c;
			}
		}
		
		// Disarm if receiver timeout
		if (TIM6->CNT > TIMEOUT_COMMAND*2)
			armed_raw = 0;
		
		// Process sensors
		if (FLAG & FLAG_SENSOR)
		{
			FLAG &= ~FLAG_SENSOR;
			FLAG |= FLAG_SENSOR_DEBUG; // Set flag to indicate data is ready for debug
			
			if (REG_CTRL__LED_SELECT == 2)
			{
				if ((sensor_sample_count & 0xFF) == 0)
					GPIOB->BSRR = GPIO_BSRR_BR_5;
				else if ((sensor_sample_count & 0xFF) == 128)
					GPIOB->BSRR = GPIO_BSRR_BS_5;
			}
			
			REG_ERROR &= 0xFFFF0000;
			REG_ERROR |= (uint32_t)sensor_error_count & 0x0000FFFF;; 
			
			accel_x_raw = ((int16_t)i2c2_rx_buffer[0]  << 8) | (int16_t)i2c2_rx_buffer[1];
			accel_y_raw = ((int16_t)i2c2_rx_buffer[2]  << 8) | (int16_t)i2c2_rx_buffer[3];
			accel_z_raw = ((int16_t)i2c2_rx_buffer[4]  << 8) | (int16_t)i2c2_rx_buffer[5];
			//temp_raw    = ((int16_t)i2c2_rx_buffer[6]  << 8) | (int16_t)i2c2_rx_buffer[7];
			gyro_x_raw  = ((int16_t)i2c2_rx_buffer[8]  << 8) | (int16_t)i2c2_rx_buffer[9];
			gyro_y_raw  = ((int16_t)i2c2_rx_buffer[10] << 8) | (int16_t)i2c2_rx_buffer[11];
			gyro_z_raw  = ((int16_t)i2c2_rx_buffer[12] << 8) | (int16_t)i2c2_rx_buffer[13];
			
			// Remove DC and scale
			gyro_x = ((float)gyro_x_raw - gyro_x_dc) * gyro_x_scale;
			gyro_y = ((float)gyro_y_raw - gyro_y_dc) * gyro_y_scale;
			gyro_z = ((float)gyro_z_raw - gyro_z_dc) * gyro_z_scale;
			accel_x = (float)accel_x_raw * accel_x_scale;
			accel_y = (float)accel_y_raw * accel_y_scale;
			accel_z = (float)accel_z_raw * accel_z_scale;
			
			error_pitch_z = error_pitch;
			error_roll_z = error_roll;
			error_yaw_z = error_yaw;
			
			error_pitch = (elevator * (float)REG_RATE__PITCH_ROLL) - gyro_y;
			error_roll = (aileron * (float)REG_RATE__PITCH_ROLL) - gyro_x;
			error_yaw = (rudder * (float)REG_RATE__YAW) - gyro_z;
			
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
			
			pitch = error_pitch * REG_PITCH_P + error_pitch_int * REG_PITCH_I + (error_pitch - error_pitch_z) * REG_PITCH_D;
			roll = error_roll * REG_ROLL_P + error_roll_int * REG_ROLL_I + (error_roll - error_roll_z) * REG_ROLL_D;
			yaw = error_yaw * REG_YAW_P + error_yaw_int * REG_YAW_I + (error_yaw - error_yaw_z) * REG_YAW_D;
			
			motor[0] = (throttle * (float)REG_THROTTLE__RANGE) + roll + pitch - yaw;
			motor[1] = (throttle * (float)REG_THROTTLE__RANGE) + roll - pitch + yaw;
			motor[2] = (throttle * (float)REG_THROTTLE__RANGE) - roll - pitch - yaw;
			motor[3] = (throttle * (float)REG_THROTTLE__RANGE) - roll + pitch + yaw;
			
			for (i=0; i<4; i++)
			{
				motor_clip[i] = (int32_t)motor[i] + (int32_t)REG_THROTTLE__ARMED;
				
				if (motor_clip[i] < (int32_t)REG_MOTOR__MIN)
					motor_clip[i] = (int32_t)REG_MOTOR__MIN;
				else if (motor_clip[i] > (int32_t)REG_MOTOR__MAX)
					motor_clip[i] = (int32_t)REG_MOTOR__MAX;
				
				motor_raw[i] = (uint16_t)motor_clip[i];
			}
			
			if (armed_raw < 1024)
			{
				TIM3->CCR1 = (REG_CTRL__MOTOR_SELECT == 1) ? (MOTOR_PULSE_MAX - (uint16_t)REG_CTRL__MOTOR_TEST) : (MOTOR_PULSE_MAX - REG_MOTOR__MIN);
				TIM3->CCR2 = (REG_CTRL__MOTOR_SELECT == 2) ? (MOTOR_PULSE_MAX - (uint16_t)REG_CTRL__MOTOR_TEST) : (MOTOR_PULSE_MAX - REG_MOTOR__MIN);
				TIM3->CCR3 = (REG_CTRL__MOTOR_SELECT == 3) ? (MOTOR_PULSE_MAX - (uint16_t)REG_CTRL__MOTOR_TEST) : (MOTOR_PULSE_MAX - REG_MOTOR__MIN);
				TIM3->CCR4 = (REG_CTRL__MOTOR_SELECT == 4) ? (MOTOR_PULSE_MAX - (uint16_t)REG_CTRL__MOTOR_TEST) : (MOTOR_PULSE_MAX - REG_MOTOR__MIN);
			}
			else
			{
				TIM3->CCR1 = MOTOR_PULSE_MAX - motor_raw[0];
				TIM3->CCR2 = MOTOR_PULSE_MAX - motor_raw[1];
				TIM3->CCR3 = MOTOR_PULSE_MAX - motor_raw[2];
				TIM3->CCR4 = MOTOR_PULSE_MAX - motor_raw[3];
			}
			TIM3->CR1 |= TIM_CR1_CEN;
			
			// VBAT
			if (ADC2->ISR & ADC_ISR_EOC)
				vbat = (float)ADC2->DR * 0.0089f;
			
			vbat_acc = vbat_acc * (1.0f - VBAT_ALPHA) + vbat;
			vbat = vbat_acc * VBAT_ALPHA;
			
			REG_VBAT = vbat;
			
			ADC2->CR |= ADC_CR_ADSTART;
		}
		
		// Beeper
		if ((REG_VBAT < REG_VBAT_MIN) || (chan6_raw > 1024) || (TIM6->CNT > TIMEOUT_COMMAND*2) || REG_CTRL__BEEP_TEST)
			TIM2->CR1 |= TIM_CR1_CEN;
		else
		{
			TIM2->CR1 &= ~TIM_CR1_CEN;
			TIM2->EGR |= TIM_EGR_UG;
		}
		
		// Manual contol of LED
		if (REG_CTRL__LED_SELECT == 0)
			GPIOB->BSRR = GPIO_BSRR_BS_5;
		else if (REG_CTRL__LED_SELECT == 1)
			GPIOB->BSRR = GPIO_BSRR_BR_5;
		
		// Record max loop time
		t = TIM7->CNT;
		if (t > REG_LOOP_TIME)
			REG_LOOP_TIME = t;
		
		// Debug
		if (REG_DEBUG)
		{
			switch (REG_DEBUG)
			{
				case 1:
				{
					if (FLAG & FLAG_SENSOR_DEBUG)
					{
						FLAG &= ~FLAG_SENSOR_DEBUG;
						USBD_CDC_Itf_tx_buffer[ 0] = (uint8_t) sensor_sample_count;
						USBD_CDC_Itf_tx_buffer[ 1] = (uint8_t) gyro_x_raw;
						USBD_CDC_Itf_tx_buffer[ 2] = (uint8_t)(gyro_x_raw >> 8);
						USBD_CDC_Itf_tx_buffer[ 3] = (uint8_t) gyro_y_raw;
						USBD_CDC_Itf_tx_buffer[ 4] = (uint8_t)(gyro_y_raw >> 8);
						USBD_CDC_Itf_tx_buffer[ 5] = (uint8_t) gyro_z_raw;
						USBD_CDC_Itf_tx_buffer[ 6] = (uint8_t)(gyro_z_raw >> 8);
						USBD_CDC_Itf_tx_buffer[ 7] = (uint8_t) accel_x_raw;
						USBD_CDC_Itf_tx_buffer[ 8] = (uint8_t)(accel_x_raw >> 8);
						USBD_CDC_Itf_tx_buffer[ 9] = (uint8_t) accel_y_raw;
						USBD_CDC_Itf_tx_buffer[10] = (uint8_t)(accel_y_raw >> 8);
						USBD_CDC_Itf_tx_buffer[11] = (uint8_t) accel_z_raw;
						USBD_CDC_Itf_tx_buffer[12] = (uint8_t)(accel_z_raw >> 8);
						USBD_CDC_SetTxBuffer(&USBD_device_handler, USBD_CDC_Itf_tx_buffer, 13);
						USBD_CDC_TransmitPacket(&USBD_device_handler);
						REG_DEBUG = 0;
					}
					break;
				}
				case 2:
				{
					if (FLAG & FLAG_SENSOR_DEBUG)
					{
						FLAG &= ~FLAG_SENSOR_DEBUG;
						USBD_CDC_Itf_tx_buffer[0] = (uint8_t)sensor_sample_count;
						float_to_bytes(&gyro_x, &USBD_CDC_Itf_tx_buffer[1]);
						float_to_bytes(&gyro_y, &USBD_CDC_Itf_tx_buffer[5]);
						float_to_bytes(&gyro_z, &USBD_CDC_Itf_tx_buffer[9]);
						float_to_bytes(&accel_x, &USBD_CDC_Itf_tx_buffer[13]);
						float_to_bytes(&accel_y, &USBD_CDC_Itf_tx_buffer[17]);
						float_to_bytes(&accel_z, &USBD_CDC_Itf_tx_buffer[21]);
						USBD_CDC_SetTxBuffer(&USBD_device_handler, USBD_CDC_Itf_tx_buffer, 25);
						USBD_CDC_TransmitPacket(&USBD_device_handler);
						REG_DEBUG = 0;
					}
					break;
				}
				case 4:
				{
					if (FLAG & FLAG_COMMAND_DEBUG)
					{
						FLAG &= ~FLAG_COMMAND_DEBUG;
						USBD_CDC_Itf_tx_buffer[ 0] = (uint8_t) command_frame_count;
						USBD_CDC_Itf_tx_buffer[ 1] = (uint8_t) throttle_raw;
						USBD_CDC_Itf_tx_buffer[ 2] = (uint8_t)(throttle_raw >> 8);
						USBD_CDC_Itf_tx_buffer[ 3] = (uint8_t) aileron_raw;
						USBD_CDC_Itf_tx_buffer[ 4] = (uint8_t)(aileron_raw>> 8);
						USBD_CDC_Itf_tx_buffer[ 5] = (uint8_t) elevator_raw;
						USBD_CDC_Itf_tx_buffer[ 6] = (uint8_t)(elevator_raw>> 8);
						USBD_CDC_Itf_tx_buffer[ 7] = (uint8_t) rudder_raw;
						USBD_CDC_Itf_tx_buffer[ 8] = (uint8_t)(rudder_raw >> 8);
						USBD_CDC_Itf_tx_buffer[ 9] = (uint8_t) armed_raw;
						USBD_CDC_Itf_tx_buffer[10] = (uint8_t)(armed_raw >> 8);
						USBD_CDC_Itf_tx_buffer[11] = (uint8_t) chan6_raw;
						USBD_CDC_Itf_tx_buffer[12] = (uint8_t)(chan6_raw >> 8);
						USBD_CDC_SetTxBuffer(&USBD_device_handler, USBD_CDC_Itf_tx_buffer, 13);
						USBD_CDC_TransmitPacket(&USBD_device_handler);
						REG_DEBUG = 0;
					}
					break;
				}
				case 5:
				{
					if (FLAG & FLAG_COMMAND_DEBUG)
					{
						FLAG &= ~FLAG_COMMAND_DEBUG;
						USBD_CDC_Itf_tx_buffer[0] = (uint8_t)command_frame_count;
						float_to_bytes(&throttle, &USBD_CDC_Itf_tx_buffer[1]);
						float_to_bytes(&aileron, &USBD_CDC_Itf_tx_buffer[5]);
						float_to_bytes(&elevator, &USBD_CDC_Itf_tx_buffer[9]);
						float_to_bytes(&rudder, &USBD_CDC_Itf_tx_buffer[13]);
						USBD_CDC_SetTxBuffer(&USBD_device_handler, USBD_CDC_Itf_tx_buffer, 17);
						USBD_CDC_TransmitPacket(&USBD_device_handler);
						REG_DEBUG = 0;
					}
					break;
				}
				case 6:
				{
					if (FLAG & FLAG_SENSOR_DEBUG)
					{
						FLAG &= ~FLAG_SENSOR_DEBUG;
						USBD_CDC_Itf_tx_buffer[0] = (uint8_t)sensor_sample_count;
						float_to_bytes(&pitch, &USBD_CDC_Itf_tx_buffer[1]);
						float_to_bytes(&roll, &USBD_CDC_Itf_tx_buffer[5]);
						float_to_bytes(&yaw, &USBD_CDC_Itf_tx_buffer[9]);
						USBD_CDC_SetTxBuffer(&USBD_device_handler, USBD_CDC_Itf_tx_buffer, 13);
						USBD_CDC_TransmitPacket(&USBD_device_handler);
						REG_DEBUG = 0;
					}
					break;
				}
				case 7:
				{
					if (FLAG & FLAG_SENSOR_DEBUG)
					{
						FLAG &= ~FLAG_SENSOR_DEBUG;
						USBD_CDC_Itf_tx_buffer[0] = (uint8_t)sensor_sample_count;
						for (i=0; i<4; i++)
							float_to_bytes(&motor[i], &USBD_CDC_Itf_tx_buffer[i*4+1]);
						USBD_CDC_SetTxBuffer(&USBD_device_handler, USBD_CDC_Itf_tx_buffer, 17);
						USBD_CDC_TransmitPacket(&USBD_device_handler);
						REG_DEBUG = 0;
					}
					break;
				}
				case 8:
				{
					if (FLAG & FLAG_SENSOR_DEBUG)
					{
						FLAG &= ~FLAG_SENSOR_DEBUG;
						USBD_CDC_Itf_tx_buffer[0] = (uint8_t)sensor_sample_count;
						for (i=0; i<4; i++)
						{
							USBD_CDC_Itf_tx_buffer[i*2+1] = (uint8_t) motor_raw[i];
							USBD_CDC_Itf_tx_buffer[i*2+2] = (uint8_t)(motor_raw[i] >> 8);
						}
						USBD_CDC_SetTxBuffer(&USBD_device_handler, USBD_CDC_Itf_tx_buffer, 9);
						USBD_CDC_TransmitPacket(&USBD_device_handler);
						REG_DEBUG = 0;
					}
					break;
				}
			}
		}
		
		// Wait for interrupts
		__WFI();
	}
}
