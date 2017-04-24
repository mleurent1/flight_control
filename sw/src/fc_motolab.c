#include "stm32f3xx.h"
#include "usbd_cdc_if.h"
#include "usbd_desc.h"
#include "fc.h"
#include "fc_reg.h"
#include "mpu_reg.h"

/* Private defines ------------------------------------*/

#define VERSION 20

#define MPU_SPI
#define ESC_DSHOT
#define RADIO_SYNCH_AT_INIT

#define RADIO_TYPE 0 // 0:IBUS, 1:SUMD

#define MOTOR_MAX 2000 // us
#define MOTOR_MIN 1000 // us
#define BEEPER_PERIOD 250 // ms
#define TIMEOUT_RADIO 500 // ms
#define TIMEOUT_MPU 10000 // us
#define ADC_SCALE 0.0089f
#define VBAT_ALPHA 0.001f
#define VBAT_PERIOD 10 // ms
#define VBAT_THRESHOLD 8.0f
#define INTEGRAL_MAX 200.0f
#define COMMAND_ALPHA 0.5f
#define REG_FLASH_ADDR 0x0803F800

//#define DEBUG
//#define DISABLE_BEEPER_ON_PA7

/* Private variables --------------------------------------*/

volatile uint32_t tick;
USBD_HandleTypeDef USBD_device_handler;
uint8_t host_rx_buffer[6];
volatile uint8_t spi2_rx_buffer[16];
volatile uint8_t spi2_tx_buffer[16];
volatile uint8_t i2c2_rx_buffer[15];
volatile uint8_t i2c2_tx_buffer[2];
volatile uint8_t uart2_rx_buffer[32];
volatile uint16_t time[4];
volatile float armed;
uint32_t motor_raw[4];

volatile _Bool flag_mpu;
volatile _Bool flag_radio;
volatile _Bool flag_motor;
volatile _Bool flag_host;
volatile _Bool flag_reg;
volatile _Bool flag_mpu_host_read;
volatile _Bool flag_radio_synch;
volatile _Bool flag_mpu_cal;
volatile _Bool flag_vbat;
volatile _Bool flag_error_mpu;
volatile _Bool flag_error_radio;

volatile _Bool flag_beep_user;
volatile _Bool flag_beep_radio;
volatile _Bool flag_beep_mpu;
volatile _Bool flag_beep_host;
volatile _Bool flag_beep_vbat;

/* Private macros ---------------------------------------------------*/

#if (RADIO_TYPE == 0)
	#define RADIO_NB_BYTES 32
#else
	#define RADIO_NB_BYTES 29
#endif

#define UART2_RX_DMA_EN \
	DMA1_Channel6->CNDTR = RADIO_NB_BYTES; \
	DMA1_Channel6->CCR |= DMA_CCR_EN;

#define TIM_DMA_EN \
	TIM2->CR1 = 0; \
	TIM3->CR1 = 0; \
	TIM2->CNT = 0; \
	TIM3->CNT = 0; \
	DMA1_Channel1->CCR &= ~DMA_CCR_EN; \
	DMA1_Channel2->CCR &= ~DMA_CCR_EN; \
	DMA1_Channel3->CCR &= ~DMA_CCR_EN; \
	DMA1_Channel7->CCR &= ~DMA_CCR_EN; \
	DMA1_Channel1->CNDTR = 17; \
	DMA1_Channel2->CNDTR = 17; \
	DMA1_Channel3->CNDTR = 17; \
	DMA1_Channel7->CNDTR = 17; \
	DMA1_Channel1->CCR |= DMA_CCR_EN; \
	DMA1_Channel2->CCR |= DMA_CCR_EN; \
	DMA1_Channel3->CCR |= DMA_CCR_EN; \
	DMA1_Channel7->CCR |= DMA_CCR_EN;

#define USB_TX(size) \
	USBD_CDC_SetTxBuffer(&USBD_device_handler, USBD_CDC_IF_tx_buffer, size); \
	USBD_CDC_TransmitPacket(&USBD_device_handler);

#define ABORT_MPU_DMA \
	DMA1_Channel4->CCR &= ~DMA_CCR_EN; \
	DMA1_Channel5->CCR &= ~DMA_CCR_EN; \
	DMA1_Channel5->CNDTR = 0; \
	DMA1->IFCR = DMA_IFCR_CGIF4 | DMA_IFCR_CGIF5; \
	flag_mpu_host_read = 0;;

#define ABORT_MPU_SPI \
	SPI2->CR1 &= ~SPI_CR1_SPE; \
	while (SPI2->SR & SPI_SR_FRLVL) \
		SPI2->DR; \
	SPI2->CR1 |= SPI_CR1_MSTR;

#define ABORT_MPU_I2C \
	I2C2->CR2 |= I2C_CR2_STOP; \
	while (I2C2->ISR & I2C_ISR_BUSY) {} \
	I2C2->CR1 &= ~I2C_CR1_PE; \
	while (I2C2->CR1 & I2C_CR1_PE) {} \
	I2C2->CR1 |= I2C_CR1_PE;

/* Private function declarations -----------------------------------*/

void uint32_to_float(uint32_t* b, float* f);
void float_to_uint32(float* f, uint32_t* b);
void wait_ms(uint32_t t);
void MpuWrite(uint8_t addr, uint8_t data);
void MpuRead(uint8_t addr, uint8_t size);
	
/* Public functions ---------------------------------------------*/

void HAL_Delay(__IO uint32_t Delay)
{
	wait_ms(Delay);
}

/* Private functions ------------------------------------------------*/

void wait_ms(uint32_t t)
{
	uint32_t next_tick;
	next_tick = tick + t;
	while (tick != next_tick)
		__WFI();
}

#ifdef MPU_SPI

	void MpuWrite(uint8_t addr, uint8_t data)
	{
		spi2_tx_buffer[0] = addr & 0x7F;
		spi2_tx_buffer[1] = data;
		DMA1_Channel4->CNDTR = 2;
		DMA1_Channel5->CNDTR = 2;
		DMA1_Channel4->CCR |= DMA_CCR_EN;
		DMA1_Channel5->CCR |= DMA_CCR_EN;
		SPI2->CR1 |= SPI_CR1_SPE;
	}

	void MpuRead(uint8_t addr, uint8_t size)
	{
		spi2_tx_buffer[0] = 0x80 | (addr & 0x7F);
		DMA1_Channel4->CNDTR = size+1;
		DMA1_Channel5->CNDTR = size+1;
		DMA1_Channel4->CCR |= DMA_CCR_EN;
		DMA1_Channel5->CCR |= DMA_CCR_EN;
		SPI2->CR1 |= SPI_CR1_SPE;
	}

#else

	void MpuWrite(uint8_t addr, uint8_t data)
	{
		i2c2_tx_buffer[0] = addr;
		i2c2_tx_buffer[1] = data;
		DMA1_Channel5->CNDTR = 0;
		DMA1_Channel4->CNDTR = 2;
		DMA1_Channel4->CCR |= DMA_CCR_EN;
		I2C2->CR2 &= ~(I2C_CR2_RD_WRN | I2C_CR2_NBYTES_Msk);
		I2C2->CR2 |= (I2C_CR2_NBYTES_Msk & (2 << I2C_CR2_NBYTES_Pos)) | I2C_CR2_START;
	}

	void MpuRead(uint8_t addr, uint8_t size)
	{
		i2c2_tx_buffer[0] = addr;
		DMA1_Channel5->CNDTR = size;
		DMA1_Channel4->CNDTR = 1;
		DMA1_Channel4->CCR |= DMA_CCR_EN;
		I2C2->CR2 &= ~(I2C_CR2_RD_WRN | I2C_CR2_NBYTES_Msk);
		I2C2->CR2 |= (I2C_CR2_NBYTES_Msk & (1 << I2C_CR2_NBYTES_Pos)) | I2C_CR2_START;
	}

#endif

void dshot_encode(uint32_t* val, volatile uint32_t buf[17])
{
	int i;
	uint8_t bit[11];
	for (i=0; i<11; i++)
	{
		buf[i] = (*val & (1 << (10-i))) ? 90 : 45;
		bit[i] = (*val & (1 << i)) ? 1 : 0;
	}
	buf[11] = 45;
	buf[12] = (bit[10]^bit[6]^bit[2]) ? 90 : 45;
    buf[13] = (bit[ 9]^bit[5]^bit[1]) ? 90 : 45;
    buf[14] = (bit[ 8]^bit[4]^bit[0]) ? 90 : 45;
    buf[15] = (bit[ 7]^bit[3])        ? 90 : 45;
	buf[16] = 0;
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

/* System timer 1ms ---*/
void SysTick_Handler()
{
	tick++;
}

/* Sample valid from MPU ---*/
void EXTI15_10_IRQHandler() 
{
	EXTI->PR = EXTI_PR_PIF15; // Clear pending request

#ifdef MPU_SPI
	if ((REG_CTRL__MPU_HOST_CTRL == 0) && ((SPI2->SR & SPI_SR_BSY) == 0))
#else
	if ((REG_CTRL__MPU_HOST_CTRL == 0) && ((I2C2->ISR & I2C_ISR_BUSY) == 0))
#endif
	{
		MpuRead(58,15);

		// SPI transaction time
		time[0] = TIM7->CNT;
		#ifdef DEBUG
			GPIOA->BSRR = GPIO_BSRR_BS_8;
		#endif
	}
}

/* I2C error ---*/
void I2C2_ER_IRQHandler()
{
	ABORT_MPU_DMA
	ABORT_MPU_I2C
	flag_error_mpu = 1;
}

/* I2C R/W management ---*/
void I2C2_EV_IRQHandler()
{
	// Disable DMA for Tx
	if (DMA1_Channel4->CCR & DMA_CCR_EN)
	{
		DMA1->IFCR = DMA_IFCR_CGIF4; // clear all flags
		DMA1_Channel4->CCR &= ~DMA_CCR_EN;
	}
	
	if (DMA1_Channel5->CNDTR > 0)
	{
		// Enable DMA for Rx
		DMA1_Channel5->CCR |= DMA_CCR_EN;
		
		// Restart I2C transfer for read
		I2C2->CR2 &= ~I2C_CR2_NBYTES_Msk;
		I2C2->CR2 |= (I2C_CR2_NBYTES_Msk & (DMA1_Channel5->CNDTR << I2C_CR2_NBYTES_Pos)) | I2C_CR2_RD_WRN | I2C_CR2_START;
	}
	else
	{
		// End I2C transfer
		I2C2->CR2 |= I2C_CR2_STOP;
		while (I2C2->ISR & I2C_ISR_BUSY) {}
	}
}

/* SPI error ---*/
void SPI2_IRQHandler() 
{
	ABORT_MPU_DMA
	ABORT_MPU_SPI
	flag_error_mpu = 1;
}

#ifdef MPU_SPI

	/* End of SPI receive ---*/
	void DMA1_Channel4_IRQHandler() 
	{
		// Check DMA transfer error
		if (DMA1->ISR & DMA_ISR_TEIF4)
			flag_error_mpu = 1;
		
		// Disable DMA
		DMA1_Channel4->CCR &= ~DMA_CCR_EN;
		DMA1_Channel5->CCR &= ~DMA_CCR_EN;
		DMA1->IFCR = DMA_IFCR_CGIF4 | DMA_IFCR_CGIF5; // clear all flags
		
		// Disable SPI
		SPI2->CR1 &= ~SPI_CR1_SPE;
		
		// Raise flag for sample ready
		flag_mpu = 1;
		
		// SPI transaction time
		time[1] = TIM7->CNT;
		#ifdef DEBUG
			GPIOA->BSRR = GPIO_BSRR_BR_8;
		#endif
	}

	/* DMA Transfer error ---*/
	void DMA1_Channel5_IRQHandler() 
	{
		ABORT_MPU_DMA
		ABORT_MPU_SPI
		flag_error_mpu = 1;
	} 

#else

	/* DMA Transfer error ---*/
	void DMA1_Channel4_IRQHandler() 
	{
		ABORT_MPU_DMA
		ABORT_MPU_I2C
		flag_error_mpu = 1;
	}

	/* End of I2C receive ---*/
	void DMA1_Channel5_IRQHandler() 
	{
		// Check DMA transfer error
		if (DMA1->ISR & DMA_ISR_TEIF5)
			flag_error_mpu = 1;
		
		// Disable DMA
		DMA1->IFCR = DMA_IFCR_CGIF5; // clear all flags
		DMA1_Channel5->CCR &= ~DMA_CCR_EN;
		
		// Raise flag for sample ready
		flag_mpu = 1;
		
		// I2C transaction time
		time_mpu_transaction = TIM15->CNT;
		
		#ifdef DEBUG
			GPIOA->BSRR = GPIO_BSRR_BR_8;
		#endif
	}

#endif

/* UART error ---*/
void USART2_IRQHandler()
{
	// Disable DMA
	DMA1_Channel6->CCR &= ~DMA_CCR_EN;
	DMA1->IFCR = DMA_IFCR_CGIF6; // clear all flags
	
	// Reset UART and disable DMA request for next resynch action
	USART2->CR1 &= ~USART_CR1_UE;
	USART2->CR3 &= ~USART_CR3_DMAR;
	USART2->CR1 |= USART_CR1_UE;
	
	// Request a resynch and signal an error
	flag_radio_synch = 1;
	flag_error_radio = 1;
	
	UART2_RX_DMA_EN
}

/* End of UART receive ---*/
void DMA1_Channel6_IRQHandler()
{
	// Check DMA transfer error
	if (DMA1->ISR & DMA_ISR_TEIF6)
		flag_error_radio = 1;
	
	// Disable DMA
	DMA1_Channel6->CCR &= ~DMA_CCR_EN;
	DMA1->IFCR = DMA_IFCR_CGIF6; // clear all flags
	
	flag_radio = 1;
	
	UART2_RX_DMA_EN
}

/* Beeper period ---*/
void TIM4_IRQHandler()
{
	TIM4->SR &= ~TIM_SR_UIF;
	
	if (flag_beep_user | flag_beep_radio | flag_beep_mpu | flag_beep_host | flag_beep_vbat)
	{
		if (GPIOA->ODR & GPIO_ODR_0)
			GPIOA->ODR &= ~GPIO_ODR_0;
		else
			GPIOA->ODR |= GPIO_ODR_0;
	}
	else
		GPIOA->ODR &= ~GPIO_ODR_0;
}

/* Radio timeout ---*/
void TIM6_DAC_IRQHandler()
{
	TIM6->SR &= ~TIM_SR_UIF;
	
	// Disarm motors!
	armed = 0;
	
	// Beep!
	flag_beep_radio = 1;
}

/* MPU timeout ---*/
void TIM1_BRK_TIM15_IRQHandler()
{
	int i;
	
	TIM15->SR &= ~TIM_SR_UIF;
	
	// Stop motors!
	for (i=0; i<4; i++)
		motor_raw[i] = 0;
	flag_motor = 1;
	
	// Beep!
	flag_beep_mpu = 1;
}

/* VBAT sampling time ---*/
void TIM1_UP_TIM16_IRQHandler()
{
	TIM16->SR &= ~TIM_SR_UIF;
	flag_vbat = 1;
}

void USB_LP_CAN_RX0_IRQHandler()
{
	HAL_PCD_IRQHandler(&PCD_handler);
}

/*-----------------------------------------------------------------------
-----------------------------------------------------------------------*/

int main()
{
	int i;
	float x;
	
	uint16_t mpu_sample_count;
	int16_t gyro_x_raw;
	int16_t gyro_y_raw;
	int16_t gyro_z_raw;
	int16_t accel_x_raw;
	int16_t accel_y_raw;
	int16_t accel_z_raw;
	int16_t temperature_raw;
	uint16_t mpu_error_count;
	float gyro_x;
	float gyro_y;
	float gyro_z;
	volatile float gyro_x_dc;
	volatile float gyro_y_dc;
	volatile float gyro_z_dc;
	volatile float gyro_x_scale;
	volatile float gyro_y_scale;
	volatile float gyro_z_scale;
	float accel_x;
	float accel_y;
	float accel_z;
	volatile float accel_x_scale;
	volatile float accel_y_scale;
	volatile float accel_z_scale;
	float temperature;
	
	uint16_t radio_frame_count;
	uint16_t throttle_raw;
	uint16_t aileron_raw;
	uint16_t elevator_raw;
	uint16_t rudder_raw;
	uint16_t armed_raw;
	uint16_t chan6_raw;
	uint16_t radio_error_count;
	volatile float throttle;
	volatile float aileron;
	volatile float elevator;
	volatile float rudder;
	//volatile float armed; // Moved to global because shared by interrupt
	volatile float chan6;
	volatile float expo_scale;
	float throttle_rate;
	float pitch_rate;
	float roll_rate;
	float yaw_rate;
	float pid_gain;
	float pitch_gain;
	float roll_gain;
	float yaw_gain;
	volatile float throttle_acc;
	volatile float aileron_acc;
	volatile float elevator_acc;
	volatile float rudder_acc;
	float throttle_s;
	float aileron_s;
	float elevator_s;
	float rudder_s;
	
	volatile float error_pitch;
	volatile float error_roll;
	volatile float error_yaw;
	float error_pitch_z;
	float error_roll_z;
	float error_yaw_z;
	volatile float error_pitch_i;
	volatile float error_roll_i;
	volatile float error_yaw_i;
	volatile float error_pitch_i_max;
	volatile float error_roll_i_max;
	volatile float error_yaw_i_max;
	float pitch;
	float roll;
	float yaw;
	
	float motor[4];
	int32_t motor_clip[4];
	//uint32_t motor_raw[4]; // Moved to global because shared by interrupt
	volatile uint32_t motor1_dshot[17];
	volatile uint32_t motor2_dshot[17];
	volatile uint32_t motor3_dshot[17];
	volatile uint32_t motor4_dshot[17];
	
	volatile float vbat_acc;
	float vbat;
	uint16_t vbat_sample_count;
	
	uint8_t reg_addr;
	uint32_t val;
	
	
	_Bool reg_flash_valid;
	uint32_t* flash_r;
	uint16_t* flash_w;
	uint32_t reg_default;
	
	int32_t time_mpu;
	int32_t time_process;
	
	/* Variable init ---------------------------------------*/
	
	flag_mpu = 0;
	flag_radio = 0;
	flag_motor = 0;
	flag_host = 0;
	flag_reg = 1;
	flag_mpu_host_read = 0;
#ifdef RADIO_SYNCH_AT_INIT
	flag_radio_synch = 1;
#else
	flag_radio_synch = 0;
#endif
	flag_mpu_cal = 1;
	flag_vbat = 0;
	flag_error_mpu = 0;
	flag_error_radio = 0;

	flag_beep_user = 0;
	flag_beep_radio = 0;
	flag_beep_mpu = 0;
	flag_beep_host = 0;
	flag_beep_vbat = 0;
	
	radio_frame_count = 0;
	radio_error_count = 0;
	throttle_raw = 0;
	aileron_raw = 0;
	elevator_raw = 0;
	rudder_raw = 0;
	armed_raw = 0;
	chan6_raw = 0;
	
	mpu_sample_count = 0;
	mpu_error_count = 0;
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
	
	throttle_rate = 0;
	pitch_rate = 0;
	roll_rate = 0;
	yaw_rate = 0;
	pid_gain = 1.0f;
	pitch_gain = 1.0f;
	roll_gain = 1.0f;
	yaw_gain = 1.0f;	
	throttle_acc = 0;
	aileron_acc = 0;
	elevator_acc = 0;
	rudder_acc = 0;
	
	expo_scale = 1.0f;
	
	error_pitch_i = 0;
	error_roll_i = 0;
	error_yaw_i = 0;
	
	vbat_acc = 15.0f / VBAT_ALPHA;
	vbat = 15.0f;
	vbat_sample_count = 0;
	
	/* Register init -----------------------------------*/
	
	flash_r = (uint32_t*)REG_FLASH_ADDR;
	flash_w = (uint16_t*)REG_FLASH_ADDR;
	if (flash_r[0] == VERSION)
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
	
	REG_VERSION = VERSION;
	
	/* Clock enable --------------------------------------------------*/
	
	// UART clock enable
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
#ifdef MPU_SPI
	// SPI clock enable
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
#else
	// I2C clock enable
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
#endif
	// Timer clock enable
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN | RCC_APB1ENR_TIM6EN | RCC_APB1ENR_TIM7EN;
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
#ifdef DISABLE_BEEPER_ON_PA7
	if ((GPIOA->IDR & GPIO_IDR_7) == 0)
		GPIOA->MODER |= GPIO_MODER_MODER0_0;
#else
	GPIOA->MODER |= GPIO_MODER_MODER0_0;
#endif
	// A1 : Motor 5, to TIM2_CH2, AF1
	// A2 : Motor 6, to TIM2_CH3, AF1
	GPIOA->MODER |= GPIO_MODER_MODER1_1 | GPIO_MODER_MODER2_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR1 | GPIO_OSPEEDER_OSPEEDR2;
	GPIOA->AFR[0] |= (1 << GPIO_AFRL_AFRL1_Pos) | (1 << GPIO_AFRL_AFRL2_Pos);
	// A3 : Motor 7
#ifdef DEBUG
	GPIOA->MODER |= GPIO_MODER_MODER3_0;
#endif
	// A4 : Motor 1, to TIM3_CH2, AF2, NOT USED
	// A5 : VBAT/10
	GPIOA->MODER |= GPIO_MODER_MODER5;
	// A6 : Motor 2, to TIM3_CH1, AF2, NOT USED
	// A7 : PPM
	// A8 : Motor 8
#ifdef DEBUG
	GPIOA->MODER |= GPIO_MODER_MODER8_0;
#endif
	// A9 : I2C2 SCL, AF4
	// A10: I2C2 SDA, AF4
#ifndef MPU_SPI 
	GPIOA->MODER |= GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9 | GPIO_OSPEEDER_OSPEEDR10;
	GPIOA->OTYPER |= GPIO_OTYPER_OT_9 | GPIO_OTYPER_OT_10;
	GPIOA->AFR[1] |= (4 << GPIO_AFRH_AFRH1_Pos) | (4 << GPIO_AFRH_AFRH2_Pos);
#endif
	// A11: USB_DM, AF14
	// A12: USB_DP, AF14
	GPIOA->MODER |= GPIO_MODER_MODER11_1 | GPIO_MODER_MODER12_1;
	GPIOA->AFR[1] |= (14 << GPIO_AFRH_AFRH3_Pos) | (14 << GPIO_AFRH_AFRH4_Pos);
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR11 | GPIO_OSPEEDER_OSPEEDR12;
	// A15: MPU interrupt
	// B0 : Motor 3, to TIM3_CH3, AF2
	// B1 : Motor 4, to TIM3_CH4, AF2
	GPIOB->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0 | GPIO_OSPEEDER_OSPEEDR1;
	GPIOB->AFR[0] |= (2 << GPIO_AFRL_AFRL0_Pos) | (2 << GPIO_AFRL_AFRL1_Pos);
	// B3 : UART2 Tx, AF7, NOT USED
	// B4 : UART2 Rx, AF7, pull-up for IDLE
	GPIOB->MODER |= GPIO_MODER_MODER4_1;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR4_0;
	GPIOB->AFR[0] |= 7 << GPIO_AFRL_AFRL4_Pos;
	// B5 : Red LED, need open-drain
	GPIOB->MODER |= GPIO_MODER_MODER5_0;
	GPIOB->OTYPER |= GPIO_OTYPER_OT_5;
	// B6 : UART1 Tx, NOT USED
	// B7 : UART1 Rx, NOT USED
	// B10: UART3 Tx, AF7, NOT USED
	// B11: UART3 Rx, AF7, NOT USED
	// B12: SPI2 CS, AF5, need pull-up
	// B13: SPI2 CLK, AF5, need pull-up
	// B14: SPI2 MISO, AF5
	// B15: SPI2 MOSI, AF5
#ifdef MPU_SPI
	GPIOB->MODER |= GPIO_MODER_MODER12_1 | GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR12 | GPIO_OSPEEDER_OSPEEDR13 | GPIO_OSPEEDER_OSPEEDR15;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR12_0 | GPIO_PUPDR_PUPDR13_0;
	GPIOB->AFR[1] |= (5 << GPIO_AFRH_AFRH4_Pos) | (5 << GPIO_AFRH_AFRH5_Pos) | (5 << GPIO_AFRH_AFRH6_Pos) | (5 << GPIO_AFRH_AFRH7_Pos);
#endif
	
	/* DMA --------------------------------------------------------------------------*/
	
#ifdef MPU_SPI
	// SPI2 Rx
	DMA1_Channel4->CCR = DMA_CCR_MINC | DMA_CCR_PL_0 | DMA_CCR_TCIE | DMA_CCR_TEIE;
	DMA1_Channel4->CMAR = (uint32_t)spi2_rx_buffer;
	DMA1_Channel4->CPAR = (uint32_t)&(SPI2->DR);

	// SPI2 Tx
	DMA1_Channel5->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_PL | DMA_CCR_TEIE;
	DMA1_Channel5->CMAR = (uint32_t)spi2_tx_buffer;
	DMA1_Channel5->CPAR = (uint32_t)&(SPI2->DR);
#else
	// I2C2 Tx
	DMA1_Channel4->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_PL_0 | DMA_CCR_TEIE;
	DMA1_Channel4->CMAR = (uint32_t)i2c2_tx_buffer;
	DMA1_Channel4->CPAR = (uint32_t)&(I2C2->TXDR);

	// I2C2 Rx
	DMA1_Channel5->CCR = DMA_CCR_MINC | DMA_CCR_PL | DMA_CCR_TCIE | DMA_CCR_TEIE;
	DMA1_Channel5->CMAR = (uint32_t)i2c2_rx_buffer;
	DMA1_Channel5->CPAR = (uint32_t)&(I2C2->RXDR);
#endif
	
	// UART2 Rx
	DMA1_Channel6->CCR = DMA_CCR_TCIE | DMA_CCR_MINC;
	DMA1_Channel6->CMAR = (uint32_t)uart2_rx_buffer;
	DMA1_Channel6->CPAR = (uint32_t)&(USART2->RDR);

	// Timers for DSHOT
	DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_PL_1 | DMA_CCR_PSIZE_1 | DMA_CCR_MSIZE_1;
	DMA1_Channel1->CMAR = (uint32_t)motor2_dshot;
	DMA1_Channel1->CPAR = (uint32_t)&(TIM2->CCR3);
	
	DMA1_Channel2->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_PL_1 | DMA_CCR_PSIZE_1 | DMA_CCR_MSIZE_1;
	DMA1_Channel2->CMAR = (uint32_t)motor3_dshot;
	DMA1_Channel2->CPAR = (uint32_t)&(TIM3->CCR3);
	
	DMA1_Channel3->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_PL_1 | DMA_CCR_PSIZE_1 | DMA_CCR_MSIZE_1;
	DMA1_Channel3->CMAR = (uint32_t)motor4_dshot;
	DMA1_Channel3->CPAR = (uint32_t)&(TIM3->CCR4);
	
	DMA1_Channel7->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_PL_1 | DMA_CCR_PSIZE_1 | DMA_CCR_MSIZE_1 | DMA_CCR_TCIE;
	DMA1_Channel7->CMAR = (uint32_t)motor1_dshot;
	DMA1_Channel7->CPAR = (uint32_t)&(TIM2->CCR2);
	
	/* Timers --------------------------------------------------------------------------*/
	
#ifdef ESC_DSHOT
	// DMA driven timer for DShot600, 24Mhz, 0:15, 1:30, T:40
	TIM2->PSC = 0;
	TIM2->ARR = 120;
	TIM2->DIER = TIM_DIER_CC2DE | TIM_DIER_CC3DE;
	TIM2->CCER = TIM_CCER_CC2E | TIM_CCER_CC3E;
	TIM2->CCMR1 = (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
	TIM2->CCMR2 = (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
	
	TIM3->PSC = 0;
	TIM3->ARR = 120;
	TIM3->DIER = TIM_DIER_CC3DE | TIM_DIER_CC4DE;
	TIM3->CCER = TIM_CCER_CC3E | TIM_CCER_CC4E;
	TIM3->CCMR2 = (6 << TIM_CCMR2_OC3M_Pos) | (6 << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;
#else	
	// One-pulse mode for OneShot125
	TIM2->CR1 = TIM_CR1_OPM;
	TIM2->PSC = 8; // 0.125us (2 for OneShot42, 0.042us)
	TIM2->ARR = MOTOR_MAX + 1;
	TIM2->CCER = TIM_CCER_CC2E | TIM_CCER_CC3E;
	TIM2->CCMR1 = (7 << TIM_CCMR1_OC2M_Pos);
	TIM2->CCMR2 = (7 << TIM_CCMR2_OC3M_Pos);
	
	TIM3->CR1 = TIM_CR1_OPM;
	TIM3->PSC = 8;
	TIM3->ARR = MOTOR_MAX + 1;
	TIM3->CCER = TIM_CCER_CC3E | TIM_CCER_CC4E;
	TIM3->CCMR2 = (7 << TIM_CCMR2_OC3M_Pos) | (7 << TIM_CCMR2_OC4M_Pos);
#endif

	// Beeper
	TIM4->PSC = 35999; // 0.5ms
	TIM4->ARR = BEEPER_PERIOD*2;
	TIM4->DIER = TIM_DIER_UIE;
	TIM4->CR1 = TIM_CR1_CEN;
	
	// Receiver timeout
	TIM6->PSC = 35999; // 0.5ms
	TIM6->ARR = TIMEOUT_RADIO*2;
	TIM6->DIER = TIM_DIER_UIE;
	TIM6->CR1 = TIM_CR1_CEN;
	
	// Sensor to motor processing time
	TIM7->PSC = 71; // 1us
	TIM7->ARR = 65535;
	TIM7->CR1 = TIM_CR1_CEN;
	
	// MPU transaction time
	TIM15->PSC = 71; // 1us
	TIM15->ARR = TIMEOUT_MPU;
	TIM15->DIER = TIM_DIER_UIE;
	TIM15->CR1 = TIM_CR1_CEN;
	
	// VBAT
	TIM16->PSC = 35999; // 0.5ms
	TIM16->ARR = VBAT_PERIOD*2;
	TIM16->DIER = TIM_DIER_UIE;
	TIM16->CR1 = TIM_CR1_CEN;
	
	/* UART ---------------------------------------------------*/
	
	USART2->BRR = 625; // 72MHz/115200bps
	USART2->CR1 = USART_CR1_RE;
	USART2->CR2 = (2 << USART_CR2_STOP_Pos);
	USART2->CR3 = USART_CR3_EIE | USART_CR3_DMAR;
#ifndef RADIO_SYNCH_AT_INIT
	UART2_RX_DMA_EN
#endif

#ifdef MPU_SPI

	/* SPI ----------------------------------------------------*/
	
	SPI2->CR1 = SPI_CR1_MSTR | (5 << SPI_CR1_BR_Pos) | SPI_CR1_CPOL | SPI_CR1_CPHA; // SPI clock = clock APB1/64 = 36MHz/64 = 562.5 kHz
	SPI2->CR2 = SPI_CR2_SSOE | SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN | SPI_CR2_FRXTH | SPI_CR2_ERRIE;
	
#else

	/* I2C ------------------------------------------------------*/
	
	I2C2->CR1 = I2C_CR1_TCIE | I2C_CR1_TXDMAEN | I2C_CR1_RXDMAEN | I2C_CR1_ERRIE;
	I2C2->CR2 = 104 << (1+I2C_CR2_SADD_Pos);
	I2C2->TIMINGR = (5 << I2C_TIMINGR_PRESC_Pos) | (9 << I2C_TIMINGR_SCLL_Pos) | (3 << I2C_TIMINGR_SCLH_Pos) | (3 << I2C_TIMINGR_SDADEL_Pos) | (3 << I2C_TIMINGR_SCLDEL_Pos); // 400kHz (Fast-mode)
	I2C2->CR1 |= I2C_CR1_PE;
	
#endif

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
	//EXTI->IMR = EXTI_IMR_MR15; // To be enabled after MPU init
	
	NVIC_EnableIRQ(EXTI15_10_IRQn);
	NVIC_EnableIRQ(USART2_IRQn);
#ifdef MPU_SPI
	NVIC_EnableIRQ(SPI2_IRQn);
#else
	NVIC_EnableIRQ(I2C2_ER_IRQn);
	NVIC_EnableIRQ(I2C2_EV_IRQn);
#endif
	NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	NVIC_EnableIRQ(DMA1_Channel5_IRQn);
	NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	NVIC_EnableIRQ(TIM4_IRQn);
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
	NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
	NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
	NVIC_EnableIRQ(USB_LP_CAN_RX0_IRQn);
	
	NVIC_SetPriority(EXTI15_10_IRQn,0);
	NVIC_SetPriority(USART2_IRQn,0);
	NVIC_SetPriority(SPI2_IRQn,0);
	NVIC_SetPriority(I2C2_ER_IRQn,0);
	NVIC_SetPriority(I2C2_EV_IRQn,0);
	NVIC_SetPriority(DMA1_Channel4_IRQn,0);
	NVIC_SetPriority(DMA1_Channel5_IRQn,0);
	NVIC_SetPriority(DMA1_Channel6_IRQn,0);
	NVIC_SetPriority(TIM4_IRQn,0);
	NVIC_SetPriority(TIM6_DAC_IRQn,0);
	NVIC_SetPriority(TIM1_BRK_TIM15_IRQn,0);
	NVIC_SetPriority(TIM1_UP_TIM16_IRQn,0);
	NVIC_SetPriority(USB_LP_CAN_RX0_IRQn,1);

	/* USB ----------------------------------------------------------*/
	
	USB->CNTR &= ~USB_CNTR_PDWN;
	wait_ms(1);
	
	USBD_Init(&USBD_device_handler, &USBD_VCP_descriptor, 0);
	USBD_RegisterClass(&USBD_device_handler, &USBD_CDC);
	USBD_CDC_RegisterInterface(&USBD_device_handler, &USBD_CDC_IF_fops);
	USBD_Start(&USBD_device_handler);
	
	/* MPU init ----------------------------------------------------*/
	
	MpuWrite(MPU_PWR_MGMT_1, MPU_PWR_MGMT_1__DEVICE_RST);
	wait_ms(100);
#ifdef MPU_SPI
	MpuWrite(MPU_SIGNAL_PATH_RST, MPU_SIGNAL_PATH_RST__ACCEL_RST | MPU_SIGNAL_PATH_RST__GYRO_RST | MPU_SIGNAL_PATH_RST__TEMP_RST);
	wait_ms(100);
	MpuWrite(MPU_USER_CTRL, MPU_USER_CTRL__I2C_IF_DIS);
	wait_ms(1);
#endif
	MpuWrite(MPU_PWR_MGMT_1, MPU_PWR_MGMT_1__CLKSEL(1));// | MPU_PWR_MGMT_1__TEMP_DIS); // Get MPU out of sleep, set CLK = gyro X clock, and disable temperature sensor
	wait_ms(100);
	//MpuWrite(MPU_PWR_MGMT_2, MPU_PWR_MGMT_2__STDBY_XA | MPU_PWR_MGMT_2__STDBY_YA | MPU_PWR_MGMT_2__STDBY_ZA); // Disable accelerometers
	//wait_ms(1);
	//MpuWrite(MPU_SMPLRT_DIV, 7); // Sample rate = Fs/(x+1)
	//wait_ms(1);
	MpuWrite(MPU_CFG, MPU_CFG__DLPF_CFG(1)); // Filter ON => Fs=1kHz, else 8kHz
	wait_ms(1);
	MpuWrite(MPU_GYRO_CFG, MPU_GYRO_CFG__FS_SEL(3)); // Full scale = +/-2000 deg/s
	wait_ms(1);
	MpuWrite(MPU_ACCEL_CFG, MPU_ACCEL_CFG__AFS_SEL(3)); // Full scale = +/- 16g
	wait_ms(100); // wait for filter to settle
	MpuWrite(MPU_INT_EN, MPU_INT_EN__DATA_RDY_EN);
	wait_ms(1);

	/* -----------------------------------------------------------------------------------*/
	
#ifdef MPU_SPI
	SPI2->CR1 &= ~SPI_CR1_BR_Msk; // SPI clock = clock APB1/2 = 36MHz/2 = 18MHz
#endif
	
	EXTI->IMR = EXTI_IMR_MR15; // Enable interrupt now to avoid problem with new SPI clock settings
	
	SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk; // Disable Systick interrupt, not needed anymore (but can still use COUNTFLAG)

	/* Main loop ---------------------------------------------------------------------
	-----------------------------------------------------------------------------------*/
	
	while (1)
	{
		// Processing time
		time[2] = TIM7->CNT;
		#ifdef DEBUG
			GPIOA->BSRR = GPIO_BSRR_BS_3;
		#endif
		
		/* Radio frame synchronisation -------------------------------------------------*/
		
		if (flag_radio_synch)
		{
			flag_radio_synch = 0;
			
			// Disable DMA
			USART2->CR1 &= ~USART_CR1_UE;			
			USART2->CR3 &= ~USART_CR3_DMAR;
			USART2->CR1 |= USART_CR1_UE;
			
			// Synchonise on IDLE character
			USART2->ICR = USART_ICR_ORECF | USART_ICR_IDLECF;
			while ((USART2->ISR & USART_ISR_IDLE) == 0)
				USART2->ICR = USART_ICR_ORECF;
			USART2->ICR = USART_ICR_IDLECF;
			USART2->RDR;
			
			// Enable DMA
			USART2->CR1 &= ~USART_CR1_UE;
			USART2->CR3 |= USART_CR3_DMAR;
			USART2->CR1 |= USART_CR1_UE;
			UART2_RX_DMA_EN
		}
		
		/* Process radio commands -----------------------------------------------------*/
		
		if (flag_radio)
		{
			flag_radio = 0;
			
			// Check header
		#if (RADIO_TYPE == 0)
			if ((uart2_rx_buffer[0] != 0x20) || (uart2_rx_buffer[1] != 0x40))
		#else
			if ((uart2_rx_buffer[0] != 0xA8) || ((uart2_rx_buffer[1] != 0x01) && (uart2_rx_buffer[1] != 0x81)))
		#endif
			{
				flag_error_radio = 1;
				flag_radio_synch = 1;
			}
			
			// TODO: verify checksum
			
			if (flag_error_radio)
			{
				flag_error_radio = 0;
				radio_error_count++;
				REG_ERROR &= 0x0000FFFF;
				REG_ERROR |= (uint32_t)radio_error_count << 16;
			}
			else
			{
				// Reset timeout
				TIM6->CNT = 0;
				flag_beep_radio = 0;
				
				radio_frame_count++;
			#if (RADIO_TYPE == 0)
				aileron_raw  = (uint16_t)uart2_rx_buffer[ 2] | ((uint16_t)uart2_rx_buffer[ 3] << 8);
				elevator_raw = (uint16_t)uart2_rx_buffer[ 4] | ((uint16_t)uart2_rx_buffer[ 5] << 8);
				throttle_raw = (uint16_t)uart2_rx_buffer[ 6] | ((uint16_t)uart2_rx_buffer[ 7] << 8);
				rudder_raw   = (uint16_t)uart2_rx_buffer[ 8] | ((uint16_t)uart2_rx_buffer[ 9] << 8);
				armed_raw    = (uint16_t)uart2_rx_buffer[10] | ((uint16_t)uart2_rx_buffer[11] << 8);
				chan6_raw    = (uint16_t)uart2_rx_buffer[12] | ((uint16_t)uart2_rx_buffer[13] << 8);
			#else
				aileron_raw  = (uint16_t)uart2_rx_buffer[ 4] | ((uint16_t)uart2_rx_buffer[ 3] << 8);
				elevator_raw = (uint16_t)uart2_rx_buffer[ 6] | ((uint16_t)uart2_rx_buffer[ 5] << 8);
				throttle_raw = (uint16_t)uart2_rx_buffer[ 8] | ((uint16_t)uart2_rx_buffer[ 7] << 8);
				rudder_raw   = (uint16_t)uart2_rx_buffer[10] | ((uint16_t)uart2_rx_buffer[ 9] << 8);
				armed_raw    = (uint16_t)uart2_rx_buffer[12] | ((uint16_t)uart2_rx_buffer[11] << 8);
				chan6_raw    = (uint16_t)uart2_rx_buffer[14] | ((uint16_t)uart2_rx_buffer[13] << 8);
			#endif
			}			
		#if (RADIO_TYPE == 0)
			throttle = (float)(throttle_raw - 1000) / 1000.0f;
			aileron = (float)((int16_t)aileron_raw - 1500) / 500.0f;
			elevator = (float)((int16_t)elevator_raw - 1500) / 500.0f;
			rudder = (float)((int16_t)rudder_raw - 1500) / 500.0f;
			armed = (float)(armed_raw - 1000) / 1000.0f;
			chan6 = (float)(chan6_raw - 1000) / 1000.0f;
		#else
			throttle = (float)(throttle_raw - 8800) / 6400.0f;
			aileron = (float)((int16_t)aileron_raw - 12000) / 3200.0f;
			elevator = (float)((int16_t)elevator_raw - 12000) / 3200.0f;
			rudder = (float)((int16_t)rudder_raw - 12000) / 3200.0f;
			armed = (float)(armed_raw - 8800) / 6400.0f;
			chan6 = (float)(chan6_raw - 8800) / 6400.0f;
		#endif
			
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
			
			// Send raw radio commands to host
			if (REG_DEBUG == 4)
			{
				USBD_CDC_IF_tx_buffer[ 0] = (uint8_t) radio_frame_count;
				USBD_CDC_IF_tx_buffer[ 1] = (uint8_t)(radio_frame_count >> 8);
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
				REG_DEBUG = 0;
			}
			
			// Beep if requested
			if (chan6 > 0.5f)
				flag_beep_user = 1;
			else
				flag_beep_user = 0;
			
			// Toggle LED at rate of Radio flag
			if (REG_CTRL__LED_SELECT == 3)
			{
				if ((radio_frame_count & 0x7F) == 0)
					GPIOB->BSRR = GPIO_BSRR_BR_5;
				else if ((radio_frame_count & 0x7F) == 64)
					GPIOB->BSRR = GPIO_BSRR_BS_5;
			}
		}
		
		/* Process sensors -----------------------------------------------------------------------*/
		
		if (flag_mpu)
		{
			flag_mpu = 0;
			
			#ifdef MPU_SPI
				if ((REG_CTRL__MPU_HOST_CTRL == 0) && ((spi2_rx_buffer[1] & MPU_INT_STATUS__DATA_RDY_INT) == 0))
					flag_error_mpu = 1;
			#else
				if ((REG_CTRL__MPU_HOST_CTRL == 0) && ((i2c2_rx_buffer[0] & MPU_INT_STATUS__DATA_RDY_INT) == 0))
					flag_error_mpu = 1;
			#endif
			
			if (flag_error_mpu)
			{
				flag_error_mpu = 0;
				mpu_error_count++;
				REG_ERROR &= 0xFFFF0000;
				REG_ERROR |= (uint32_t)mpu_error_count & 0x0000FFFF;
			}
			else
			{
				if (REG_CTRL__MPU_HOST_CTRL == 0)
				{
					// Reset timeout
					TIM15->CNT = 0;
					flag_beep_mpu = 0;
					
					mpu_sample_count++;
					
					#ifdef MPU_SPI
						accel_x_raw     = ((int16_t)spi2_rx_buffer[ 2] << 8) | (int16_t)spi2_rx_buffer[ 3];
						accel_y_raw     = ((int16_t)spi2_rx_buffer[ 4] << 8) | (int16_t)spi2_rx_buffer[ 5];
						accel_z_raw     = ((int16_t)spi2_rx_buffer[ 6] << 8) | (int16_t)spi2_rx_buffer[ 7];
						temperature_raw = ((int16_t)spi2_rx_buffer[ 8] << 8) | (int16_t)spi2_rx_buffer[ 9];
						gyro_x_raw      = ((int16_t)spi2_rx_buffer[10] << 8) | (int16_t)spi2_rx_buffer[11];
						gyro_y_raw      = ((int16_t)spi2_rx_buffer[12] << 8) | (int16_t)spi2_rx_buffer[13];
						gyro_z_raw      = ((int16_t)spi2_rx_buffer[14] << 8) | (int16_t)spi2_rx_buffer[15];
					#else
						accel_x_raw     = ((int16_t)i2c2_rx_buffer[ 1] << 8) | (int16_t)i2c2_rx_buffer[ 2];
						accel_y_raw     = ((int16_t)i2c2_rx_buffer[ 3] << 8) | (int16_t)i2c2_rx_buffer[ 4];
						accel_z_raw     = ((int16_t)i2c2_rx_buffer[ 5] << 8) | (int16_t)i2c2_rx_buffer[ 6];
						temperature_raw = ((int16_t)i2c2_rx_buffer[ 7] << 8) | (int16_t)i2c2_rx_buffer[ 8];
						gyro_x_raw      = ((int16_t)i2c2_rx_buffer[ 9] << 8) | (int16_t)i2c2_rx_buffer[10];
						gyro_y_raw      = ((int16_t)i2c2_rx_buffer[11] << 8) | (int16_t)i2c2_rx_buffer[12];
						gyro_z_raw      = ((int16_t)i2c2_rx_buffer[13] << 8) | (int16_t)i2c2_rx_buffer[14];
					#endif
					
					// MPU calibration
					if (flag_mpu_cal)
					{
						if (mpu_sample_count <= 1000)
						{
							gyro_x_dc += (float)gyro_x_raw;
							gyro_y_dc += (float)gyro_y_raw;
							gyro_z_dc += (float)gyro_z_raw;
						}
						else
						{
							flag_mpu_cal = 0;
							
							gyro_x_dc = gyro_x_dc / 1000.0f;
							gyro_y_dc = gyro_y_dc / 1000.0f;
							gyro_z_dc = gyro_z_dc / 1000.0f;
	
							gyro_x_scale = 0.061035f;
							gyro_y_scale = 0.061035f;
							gyro_z_scale = 0.061035f;
							
							accel_x_scale = 0.00048828f;
							accel_y_scale = 0.00048828f;
							accel_z_scale = 0.00048828f;
						}
					}
					
					// Record SPI transaction time
					time_mpu = (int32_t)time[1] - (int32_t)time[0];
					if (time_mpu < 0)
						time_mpu += 65536;
					if ((REG_CTRL__TIME_MAXHOLD == 0) || ((time_mpu > REG_TIME__SPI) && REG_CTRL__TIME_MAXHOLD))
					{
						REG_TIME &= 0xFFFF0000;
						REG_TIME |= (uint32_t)time_mpu & 0x0000FFFF;
					}
				}
				else if (flag_mpu_host_read)
				{
					flag_mpu_host_read = 0;
					#ifdef MPU_SPI
						USBD_CDC_IF_tx_buffer[0] = spi2_rx_buffer[1];
					#else
						USBD_CDC_IF_tx_buffer[0] = i2c2_rx_buffer[0];
					#endif
					USB_TX(1)
				}
			}
			
			// Send Raw sensor values to host
			if (REG_DEBUG == 1)
			{
				USBD_CDC_IF_tx_buffer[ 0] = (uint8_t) mpu_sample_count;
				USBD_CDC_IF_tx_buffer[ 1] = (uint8_t)(mpu_sample_count >> 8);
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
				REG_DEBUG = 0;
			}
			
			// Remove DC and scale
			gyro_x = ((float)gyro_x_raw - gyro_x_dc) * gyro_x_scale;
			gyro_y = ((float)gyro_y_raw - gyro_y_dc) * gyro_y_scale;
			gyro_z = ((float)gyro_z_raw - gyro_z_dc) * gyro_z_scale;
			accel_x = (float)accel_x_raw * accel_x_scale;
			accel_y = (float)accel_y_raw * accel_y_scale;
			accel_z = (float)accel_z_raw * accel_z_scale;
			temperature = (float)temperature_raw / 340.0f + 36.53f;
			
			// Send scaled sensor values to host
			if (REG_DEBUG == 2)
			{
				USBD_CDC_IF_tx_buffer[0] = (uint8_t) mpu_sample_count;
				USBD_CDC_IF_tx_buffer[1] = (uint8_t)(mpu_sample_count >> 8);
				float_to_bytes(&gyro_x, &USBD_CDC_IF_tx_buffer[2]);
				float_to_bytes(&gyro_y, &USBD_CDC_IF_tx_buffer[6]);
				float_to_bytes(&gyro_z, &USBD_CDC_IF_tx_buffer[10]);
				float_to_bytes(&accel_x, &USBD_CDC_IF_tx_buffer[14]);
				float_to_bytes(&accel_y, &USBD_CDC_IF_tx_buffer[18]);
				float_to_bytes(&accel_z, &USBD_CDC_IF_tx_buffer[22]);
				float_to_bytes(&temperature, &USBD_CDC_IF_tx_buffer[26]);
				USB_TX(30)
				REG_DEBUG = 0;
			}
			
			// Smooth radio command
			throttle_acc = throttle_acc *(1.0f - COMMAND_ALPHA) + throttle;
			aileron_acc = aileron_acc *(1.0f - COMMAND_ALPHA) + aileron;
			elevator_acc = elevator_acc *(1.0f - COMMAND_ALPHA) + elevator;
			rudder_acc = rudder_acc *(1.0f - COMMAND_ALPHA) + rudder;
			throttle_s = throttle_acc * COMMAND_ALPHA;
			aileron_s = aileron_acc * COMMAND_ALPHA;
			elevator_s = elevator_acc * COMMAND_ALPHA;
			rudder_s = rudder_acc * COMMAND_ALPHA;
			
			// Send smoothed radio commands to host
			if (REG_DEBUG == 5)
			{
				USBD_CDC_IF_tx_buffer[ 0] = (uint8_t) mpu_sample_count;
				USBD_CDC_IF_tx_buffer[ 1] = (uint8_t)(mpu_sample_count >> 8);
				float_to_bytes(&throttle_s, &USBD_CDC_IF_tx_buffer[2]);
				float_to_bytes(&aileron_s, &USBD_CDC_IF_tx_buffer[6]);
				float_to_bytes(&elevator_s, &USBD_CDC_IF_tx_buffer[10]);
				float_to_bytes(&rudder_s, &USBD_CDC_IF_tx_buffer[14]);
				USB_TX(18)
				REG_DEBUG = 0;
			}
			
			// Convert command into gyro rate
			pitch_rate = elevator_s * REG_COMMAND_RATE;
			roll_rate = aileron_s * REG_COMMAND_RATE;
			yaw_rate = rudder_s * REG_COMMAND_RATE;
			
			// Convert throttle into motor command
			throttle_rate = throttle_s * REG_THROTTLE_RANGE;
			
			// PID attenuation when high throttle
			pid_gain = 1.0f - (throttle_s *  REG_THROTTLE_PID_ATTEN);
			
			// PID attenuation when high rate
			pitch_gain = 1.0f - (REG_RATE_PID_ATTEN * abs(elevator));
			roll_gain = 1.0f - (REG_RATE_PID_ATTEN * abs(aileron));
			yaw_gain = 1.0f - (REG_RATE_PID_ATTEN * abs(rudder));
			
			// Previous error
			error_pitch_z = error_pitch;
			error_roll_z = error_roll;
			error_yaw_z = error_yaw;
			
			// Current error
			error_pitch = pitch_rate - gyro_y;
			error_roll = roll_rate - gyro_x;
			error_yaw = yaw_rate - gyro_z;
			
			// Integrate error
			if ((armed < 0.5f) && REG_CTRL__RESET_INTEGRAL_ON_ARMED)
			{
				error_pitch_i = 0.0f;
				error_roll_i = 0.0f;
				error_yaw_i = 0.0f;
			}
			else
			{
				error_pitch_i += error_pitch;
				error_roll_i += error_roll;
				error_yaw_i += error_yaw;
			}
			
			// Clip the integral
			if (error_pitch_i > error_pitch_i_max)
				error_pitch_i = error_pitch_i_max;
			else if (error_pitch_i < -error_pitch_i_max)
				error_pitch_i = -error_pitch_i_max;
			if (error_roll_i > error_roll_i_max)
				error_roll_i = error_roll_i_max;
			else if (error_roll_i < -error_roll_i_max)
				error_roll_i = -error_roll_i_max;
			if (error_yaw_i > error_yaw_i_max)
				error_yaw_i = error_yaw_i_max;
			else if (error_yaw_i < -error_yaw_i_max)
				error_yaw_i = -error_yaw_i_max;
			
			// PID
			/*if (chan6_raw < 1250)
			{
				pitch = (error_pitch * REG_PITCH_P);
				roll  = (error_roll  * REG_ROLL_P );
				yaw   = (error_yaw   * REG_YAW_P  );
			}
			else if (chan6_raw < 1750)
			{
				pitch = (error_pitch * REG_PITCH_P) + ((error_pitch - error_pitch_z) * REG_PITCH_D);
				roll  = (error_roll  * REG_ROLL_P ) + ((error_roll  - error_roll_z ) * REG_ROLL_D);
				yaw   = (error_yaw   * REG_YAW_P  ) + ((error_yaw   - error_yaw_z  ) * REG_YAW_D);
			}
			else
			{*/
				pitch = (error_pitch * REG_PITCH_P) + (error_pitch_i * REG_PITCH_I) + ((error_pitch - error_pitch_z) * REG_PITCH_D);
				roll  = (error_roll  * REG_ROLL_P ) + (error_roll_i  * REG_ROLL_I ) + ((error_roll  - error_roll_z ) * REG_ROLL_D);
				yaw   = (error_yaw   * REG_YAW_P  ) + (error_yaw_i   * REG_YAW_I  ) + ((error_yaw   - error_yaw_z  ) * REG_YAW_D);
			//}
			
			// PID adjustement
			pitch = pitch * pid_gain * pitch_gain;
			roll = roll * pid_gain * roll_gain;
			yaw = yaw * pid_gain * yaw_gain;
			
			// Send pitch/roll/yaw to host
			if (REG_DEBUG == 6)
			{
				USBD_CDC_IF_tx_buffer[0] = (uint8_t) mpu_sample_count;
				USBD_CDC_IF_tx_buffer[1] = (uint8_t)(mpu_sample_count >> 8);
				float_to_bytes(&pitch, &USBD_CDC_IF_tx_buffer[2]);
				float_to_bytes(&roll, &USBD_CDC_IF_tx_buffer[6]);
				float_to_bytes(&yaw, &USBD_CDC_IF_tx_buffer[10]);
				USB_TX(14)
				REG_DEBUG = 0;
			}
			
			// Motor matrix
			motor[0] = throttle_rate + roll + pitch - yaw;
			motor[1] = throttle_rate + roll - pitch + yaw;
			motor[2] = throttle_rate - roll - pitch - yaw;
			motor[3] = throttle_rate - roll + pitch + yaw;
			
			// Send motor actions to host
			if (REG_DEBUG == 7)
			{
				USBD_CDC_IF_tx_buffer[0] = (uint8_t) mpu_sample_count;
				USBD_CDC_IF_tx_buffer[1] = (uint8_t)(mpu_sample_count >> 8);
				for (i=0; i<4; i++)
					float_to_bytes(&motor[i], &USBD_CDC_IF_tx_buffer[i*4+2]);
				USB_TX(18)
				REG_DEBUG = 0;
			}
			
			// Offset and clip motor value
			for (i=0; i<4; i++)
			{
				motor_clip[i] = (int32_t)motor[i] + (int32_t)REG_MOTOR_ARMED;
				
				if (motor_clip[i] < (int32_t)REG_MOTOR_START)
					motor_clip[i] = (int32_t)REG_MOTOR_START;
				else if (motor_clip[i] > (int32_t)MOTOR_MAX)
					motor_clip[i] = (int32_t)MOTOR_MAX;
				
				if (REG_MOTOR_TEST__SELECT)
					motor_raw[i] = (REG_MOTOR_TEST__SELECT & (1 << i)) ? (uint32_t)REG_MOTOR_TEST__VALUE : 0;
				else if (armed < 0.5f)
					motor_raw[i] = 0;
				else
					motor_raw[i] = (uint32_t)motor_clip[i];
			}
			
			// Raise flag for motor command ready
			flag_motor = 1;
			
			// Send motor command to host
			if (REG_DEBUG == 8)
			{
				USBD_CDC_IF_tx_buffer[0] = (uint8_t) mpu_sample_count;
				USBD_CDC_IF_tx_buffer[1] = (uint8_t)(mpu_sample_count >> 8);
				for (i=0; i<4; i++)
				{
					USBD_CDC_IF_tx_buffer[i*2+2] = (uint8_t) motor_raw[i];
					USBD_CDC_IF_tx_buffer[i*2+3] = (uint8_t)(motor_raw[i] >> 8);
				}
				USB_TX(10)
				REG_DEBUG = 0;
			}
			
			// Toggle LED at rate of MPU flag
			if (REG_CTRL__LED_SELECT == 2)
			{
				if ((mpu_sample_count & 0x3FF) == 0)
					GPIOB->BSRR = GPIO_BSRR_BR_5;
				else if ((mpu_sample_count & 0x3FF) == 512)
					GPIOB->BSRR = GPIO_BSRR_BS_5;
			}
		}
		
		/* ESC command -----------------------------------------------------------------*/
		
		if (flag_motor)
		{
			flag_motor = 0;
			
			#ifdef ESC_DSHOT
				dshot_encode(&motor_raw[0], motor1_dshot);
				dshot_encode(&motor_raw[1], motor2_dshot);
				dshot_encode(&motor_raw[2], motor3_dshot);
				dshot_encode(&motor_raw[3], motor4_dshot);
				TIM_DMA_EN
			#else
				TIM2->CCR2 = MOTOR_MAX + 1 - MOTOR_MIN - (motor_raw[0]>>1);
				TIM2->CCR3 = MOTOR_MAX + 1 - MOTOR_MIN - (motor_raw[1]>>1);
				TIM3->CCR3 = MOTOR_MAX + 1 - MOTOR_MIN - (motor_raw[2]>>1);
				TIM3->CCR4 = MOTOR_MAX + 1 - MOTOR_MIN - (motor_raw[3]>>1);
			#endif
			TIM2->CR1 |= TIM_CR1_CEN;
			TIM3->CR1 |= TIM_CR1_CEN;
		}
		
		/* VBAT ---------------------------------------------------------------------*/
		
		if (flag_vbat)
		{
			flag_vbat = 0;
			
			vbat_sample_count++;
			
			if (ADC2->ISR & ADC_ISR_EOC)
				vbat = (float)ADC2->DR * ADC_SCALE;
			
			vbat_acc = vbat_acc * (1.0f - VBAT_ALPHA) + vbat;
			vbat = vbat_acc * VBAT_ALPHA;
			
			REG_VBAT = vbat;
			
			ADC2->CR |= ADC_CR_ADSTART;
			
			// Send VBAT to host
			if (REG_DEBUG == 3)
			{
				USBD_CDC_IF_tx_buffer[0] = (uint8_t) vbat_sample_count;
				USBD_CDC_IF_tx_buffer[1] = (uint8_t)(vbat_sample_count >> 8);
				float_to_bytes(&vbat, &USBD_CDC_IF_tx_buffer[2]);
				USB_TX(6)
				REG_DEBUG = 0;
			}
			
			// Beep if VBAT too low
			if ((REG_VBAT < REG_VBAT_MIN) && (REG_VBAT > VBAT_THRESHOLD))
				flag_beep_vbat = 1;
			else
				flag_beep_vbat = 0;
		}
		
		/* Host command from USB -------------------------------------------------------*/
		
		if (flag_host)
		{
			flag_host = 0;
			
			reg_addr = host_rx_buffer[1];
			
			switch (host_rx_buffer[0])
			{
				case 0: // REG read
				{
					if (reg_properties[reg_addr].is_float)
						float_to_uint32(&regf[reg_addr], &val);
					else
						val = reg[reg_addr];
					USBD_CDC_IF_tx_buffer[0] = (uint8_t)((val >> 24) & 0xFF);
					USBD_CDC_IF_tx_buffer[1] = (uint8_t)((val >> 16) & 0xFF);
					USBD_CDC_IF_tx_buffer[2] = (uint8_t)((val >>  8) & 0xFF);
					USBD_CDC_IF_tx_buffer[3] = (uint8_t)((val >>  0) & 0xFF);
					USB_TX(4)
					break;
				}
				case 1: // REG write
				{
					if (!reg_properties[reg_addr].read_only)
					{
						val = ((uint32_t)host_rx_buffer[2] << 24) | ((uint32_t)host_rx_buffer[3] << 16) | ((uint32_t)host_rx_buffer[4] << 8) | (uint32_t)host_rx_buffer[5];
						if (reg_properties[reg_addr].is_float)
							uint32_to_float(&val, &regf[reg_addr]);
						else
							reg[reg_addr] = val;
						flag_reg = 1;
					}
					break;
				}
				case 2: // SPI read to MPU
				{
					flag_mpu_host_read = 1;
					MpuRead(reg_addr,1);
					break;
				}
				case 3: // SPI write to MPU
				{
					MpuWrite(reg_addr,host_rx_buffer[5]);
					break;
				}
				case 4: // Flash read
				{
					val = flash_r[reg_addr];
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
					flash_w[reg_addr*2] = ((uint16_t)host_rx_buffer[4] << 8) | (uint16_t)host_rx_buffer[5];
					flash_w[reg_addr*2+1] = ((uint16_t)host_rx_buffer[2] << 8) | (uint16_t)host_rx_buffer[3];
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
		
		/* Actions only needed on a register write ---------------------------------------------------*/
		
		if (flag_reg)
		{
			flag_reg = 0;
		
		#ifdef MPU_SPI
			// Adapt SPI clock frequency
			if (REG_CTRL__MPU_HOST_CTRL)
				SPI2->CR1 |= (5 << SPI_CR1_BR_Pos); // 562.5kHz
			else
				SPI2->CR1 &= ~SPI_CR1_BR_Msk; // 18MHz
		#endif
			
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
			
			// Max value of integral term
			error_pitch_i_max = INTEGRAL_MAX / REG_PITCH_I;
			error_roll_i_max = INTEGRAL_MAX / REG_ROLL_I;
			error_yaw_i_max = INTEGRAL_MAX / REG_YAW_I;
			
			// Beep test
			if (REG_CTRL__BEEP_TEST)
				flag_beep_host = 1;
			else
				flag_beep_host = 0;
		}
		
		/*------------------------------------------------------------------*/
		
		// Record processing time
		time[3] = TIM7->CNT;
		time_process = (int32_t)time[3] - (int32_t)time[2];
		if (time_process < 0)
			time_process += 65536;
		if ((REG_CTRL__TIME_MAXHOLD == 0) || ((time_process > REG_TIME__PROCESS) && REG_CTRL__TIME_MAXHOLD))
		{
			REG_TIME &= 0x0000FFFF;
			REG_TIME |= (uint32_t)time_process << 16;
		}
	#ifdef DEBUG
		GPIOA->BSRR = GPIO_BSRR_BR_3;
	#endif
		
		// Wait for interrupts
		__WFI();
	}
}
