#include "stm32f4xx.h"
#include "usbd_cdc_if.h"
#include "usbd_desc.h"
#include "fc.h"
#include "fc_reg.h"
#include "mpu_reg.h"

/* Private defines ------------------------------------*/

#define VERSION 24

#define ESC_DSHOT

#define RADIO_TYPE 0 // 0:IBUS, 1:SUMD, 2:SBUS

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
#define COMMAND_ALPHA 0.1f
#define REG_FLASH_ADDR 0x080E0000

//#define DEBUG

/* Private variables --------------------------------------*/

volatile uint32_t tick;
USBD_HandleTypeDef USBD_device_handler;
uint8_t host_rx_buffer[6];
volatile uint8_t spi1_rx_buffer[16];
volatile uint8_t spi1_tx_buffer[16];
uint16_t time[4];
float armed;
volatile uint32_t motor_raw[4];
uint16_t mpu_error_count;
uint16_t radio_error_count;

volatile _Bool flag_mpu;
volatile _Bool flag_radio;
volatile _Bool flag_motor;
volatile _Bool flag_host;
volatile _Bool flag_reg;
volatile _Bool flag_mpu_host_read;
volatile _Bool flag_radio_synch;
volatile _Bool flag_mpu_cal;
volatile _Bool flag_vbat;
volatile _Bool flag_armed_locked;

volatile _Bool flag_beep_user;
volatile _Bool flag_beep_radio;
volatile _Bool flag_beep_mpu;
volatile _Bool flag_beep_host;
volatile _Bool flag_beep_vbat;

/* Private macros ---------------------------------------------------*/

#if (RADIO_TYPE == 0)
	#define RADIO_NB_BYTES 32
#elif (RADIO_TYPE == 1)
	#define RADIO_NB_BYTES 29
#else
	#define RADIO_NB_BYTES 25
#endif

#define DMA_CLEAR_ALL_FLAGS_0 (DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 |DMA_LIFCR_CDMEIF0 |DMA_LIFCR_CFEIF0)
#define DMA_CLEAR_ALL_FLAGS_1 (DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 |DMA_LIFCR_CDMEIF1 |DMA_LIFCR_CFEIF1)
#define DMA_CLEAR_ALL_FLAGS_2 (DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTEIF2 |DMA_LIFCR_CDMEIF2 |DMA_LIFCR_CFEIF2)
#define DMA_CLEAR_ALL_FLAGS_3 (DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3 |DMA_LIFCR_CDMEIF3 |DMA_LIFCR_CFEIF3)
#define DMA_CLEAR_ALL_FLAGS_4 (DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4 | DMA_HIFCR_CTEIF4 |DMA_HIFCR_CDMEIF4 |DMA_HIFCR_CFEIF4)
#define DMA_CLEAR_ALL_FLAGS_5 (DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 |DMA_HIFCR_CDMEIF5 |DMA_HIFCR_CFEIF5)
#define DMA_CLEAR_ALL_FLAGS_6 (DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6 |DMA_HIFCR_CDMEIF6 |DMA_HIFCR_CFEIF6)
#define DMA_CLEAR_ALL_FLAGS_7 (DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTEIF7 |DMA_HIFCR_CDMEIF7 |DMA_HIFCR_CFEIF7)

#define TIM_DMA_EN \
	TIM2->CR1 = 0; \
	TIM3->CR1 = 0; \
	TIM5->CR1 = 0; \
	TIM2->CNT = 0; \
	TIM3->CNT = 0; \
	TIM5->CNT = 0; \
	DMA1_Stream1->CR &= ~DMA_SxCR_EN; \
	DMA1_Stream2->CR &= ~DMA_SxCR_EN; \
	DMA1_Stream3->CR &= ~DMA_SxCR_EN; \
	DMA1_Stream4->CR &= ~DMA_SxCR_EN; \
	DMA1->LIFCR = DMA_CLEAR_ALL_FLAGS_1 | DMA_CLEAR_ALL_FLAGS_2 | DMA_CLEAR_ALL_FLAGS_3; \
	DMA1->HIFCR = DMA_CLEAR_ALL_FLAGS_4; \
	DMA1_Stream1->NDTR = 17; \
	DMA1_Stream2->NDTR = 17; \
	DMA1_Stream3->NDTR = 17; \
	DMA1_Stream4->NDTR = 17; \
	DMA1_Stream1->CR |= DMA_SxCR_EN; \
	DMA1_Stream2->CR |= DMA_SxCR_EN; \
	DMA1_Stream3->CR |= DMA_SxCR_EN; \
	DMA1_Stream4->CR |= DMA_SxCR_EN;

#define USB_TX(size) \
	USBD_CDC_SetTxBuffer(&USBD_device_handler, USBD_CDC_IF_tx_buffer, size); \
	USBD_CDC_TransmitPacket(&USBD_device_handler);

#define ABORT_MPU \
	DMA2_Stream0->CR &= ~DMA_SxCR_EN; \
	DMA2_Stream3->CR &= ~DMA_SxCR_EN; \
	DMA2->LIFCR = DMA_CLEAR_ALL_FLAGS_0 | DMA_CLEAR_ALL_FLAGS_3; \
	DMA2_Stream0->NDTR = 0; \
	DMA2_Stream3->NDTR = 0; \
	SPI1->CR1 &= ~SPI_CR1_SPE; \
	SPI1->CR1 |= SPI_CR1_MSTR; \
	flag_mpu_host_read = 0; \
	flag_mpu = 0; \
	mpu_error_count++;

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

void MpuWrite(uint8_t addr, uint8_t data)
{
	spi1_tx_buffer[0] = addr & 0x7F;
	spi1_tx_buffer[1] = data;
	DMA2_Stream0->NDTR = 2;
	DMA2_Stream3->NDTR = 2;
	DMA2_Stream0->CR |= DMA_SxCR_EN;
	DMA2_Stream3->CR |= DMA_SxCR_EN;
	SPI1->CR1 |= SPI_CR1_SPE;
}

void MpuRead(uint8_t addr, uint8_t size)
{
	spi1_tx_buffer[0] = 0x80 | (addr & 0x7F);
	DMA2_Stream0->NDTR = size+1;
	DMA2_Stream3->NDTR = size+1;
	DMA2_Stream0->CR |= DMA_SxCR_EN;
	DMA2_Stream3->CR |= DMA_SxCR_EN;
	SPI1->CR1 |= SPI_CR1_SPE;
}

void dshot_encode(volatile uint32_t* val, volatile uint32_t buf[17])
{
	int i;
	uint8_t bit[11];
	for (i=0; i<11; i++)
	{
		buf[i] = (*val & (1 << (10-i))) ? 60 : 30;
		bit[i] = (*val & (1 << i)) ? 1 : 0;
	}
	buf[11] = 30;
	buf[12] = (bit[10]^bit[6]^bit[2]) ? 60 : 30;
    buf[13] = (bit[ 9]^bit[5]^bit[1]) ? 60 : 30;
    buf[14] = (bit[ 8]^bit[4]^bit[0]) ? 60 : 30;
    buf[15] = (bit[ 7]^bit[3])        ? 60 : 30;
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

float expo(float linear, float scale, float expo_val)
{
	float x;
	float y;
	x = linear * expo_val;
	y = x;
	x = x * linear * expo_val;
	y += x / 2.0f;
	x = x * linear * expo_val;
	y += x / 6.0f;
	x = x * linear * expo_val;
	y += x / 24.0f;
	x = x * linear * expo_val;
	y += x / 120.0f;
	x = x * linear * expo_val;
	y += x / 720.0f;
	return (y / scale);
}

/* Interrupt routines --------------------------------*/

/*--- System timer ---*/
void SysTick_Handler()
{
	tick++;
}

/*--- Sample valid from MPU ---*/
void EXTI4_IRQHandler() 
{
	EXTI->PR = EXTI_PR_PR4; // Clear pending request

	if ((REG_CTRL__MPU_HOST_CTRL == 0) && ((SPI1->SR & SPI_SR_BSY) == 0))
	{
		MpuRead(58,15);

		// SPI transaction time
		time[0] = TIM7->CNT;
		
		#ifdef DEBUG
			GPIOB->BSRR = GPIO_BSRR_BS_12;
		#endif
	}
}

/*--- SPI error ---*/
void SPI1_IRQHandler() 
{
	ABORT_MPU
}

/*--- End of SPI receive ---*/
void DMA2_Stream0_IRQHandler() 
{
	// Check DMA transfer error
	if (DMA2->LISR & DMA_LISR_TEIF0)
		mpu_error_count++;
	else
		flag_mpu = 1; // Raise flag for sample ready
	
	// Disable DMA
	DMA2_Stream0->CR &= ~DMA_SxCR_EN;
	DMA2_Stream3->CR &= ~DMA_SxCR_EN;
	DMA2->LIFCR = DMA_CLEAR_ALL_FLAGS_0 | DMA_CLEAR_ALL_FLAGS_3;
	
	// Disable SPI
	SPI1->CR1 &= ~SPI_CR1_SPE;
	
	// SPI transaction time
	time[1] = TIM7->CNT;
	
	#ifdef DEBUG
		GPIOB->BSRR = GPIO_BSRR_BR_12;
	#endif
}

/*--- DMA Transfer error ---*/
void DMA2_Stream3_IRQHandler() 
{
	ABORT_MPU
}

/*--- UART error ---*/
void USART1_IRQHandler()
{
	DMA2_Stream5->CR &= ~DMA_SxCR_EN;
	DMA2->HIFCR = DMA_CLEAR_ALL_FLAGS_5;
	DMA2_Stream5->NDTR = 0;
	USART1->CR1 &= ~USART_CR1_UE;
	flag_radio = 0;
	flag_radio_synch = 1;
	radio_error_count++;
}

/*--- End of UART receive ---*/
void DMA2_Stream5_IRQHandler()
{
	// Check DMA transfer error
	if (DMA2->HISR & DMA_HISR_TEIF5)
		radio_error_count++;
	else
		flag_radio = 1; // Raise flag for radio commands ready
	
	// Disable DMA
	DMA2_Stream5->CR &= ~DMA_SxCR_EN;
	DMA2->HIFCR = DMA_CLEAR_ALL_FLAGS_5;
	
	// Enable DMA
	DMA2_Stream5->NDTR = RADIO_NB_BYTES;
	DMA2_Stream5->CR |= DMA_SxCR_EN;
}

/*--- Beeper period ---*/
void TIM4_IRQHandler()
{
	TIM4->SR &= ~TIM_SR_UIF;
	
	if (flag_beep_user || flag_beep_radio || flag_beep_mpu || flag_beep_host || flag_beep_vbat)
	{
		if (GPIOA->ODR & GPIO_ODR_OD0)
			GPIOA->ODR &= ~GPIO_ODR_OD0;
		else
			GPIOA->ODR |= GPIO_ODR_OD0;
	}
	else
		GPIOA->ODR &= ~GPIO_ODR_OD0;
}

/*--- Radio timeout ---*/
void TIM6_DAC_IRQHandler()
{
	TIM6->SR &= ~TIM_SR_UIF;
	
	// Disarm motors!
	armed = 0;
	
	// Beep!
	flag_beep_radio = 1;
}

/*--- MPU timeout ---*/
void TIM8_BRK_TIM12_IRQHandler()
{
	int i;
	
	TIM12->SR &= ~TIM_SR_UIF;
	
	// Stop motors!
	for (i=0; i<4; i++)
		motor_raw[i] = 0;
	flag_motor = 1;
	
	// Beep!
	flag_beep_mpu = 1;
}

/*--- VBAT sampling time ---*/
void TIM8_UP_TIM13_IRQHandler()
{
	TIM13->SR &= ~TIM_SR_UIF;
	flag_vbat = 1;
}

/*--- USB interrupt ---*/
void OTG_FS_IRQHandler(void)
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
	//uint16_t mpu_error_count; // Moved to global because shared by interrupt
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
	
	volatile uint8_t uart1_rx_buffer[32];
	
	uint16_t radio_frame_count;
	uint16_t throttle_raw;
	uint16_t aileron_raw;
	uint16_t elevator_raw;
	uint16_t rudder_raw;
	uint16_t armed_raw;
	uint16_t chan6_raw;
	//uint16_t radio_error_count; // Moved to global because shared by interrupt
	float throttle;
	float aileron;
	float elevator;
	float rudder;
	//float armed; // Moved to global because shared by interrupt
	float armed1;
	float chan6;
	float pitch_roll_expo_scale;
	float yaw_expo_scale;
	float throttle_rate;
	float pitch_rate;
	float roll_rate;
	float yaw_rate;
	float throttle_gain;
	float throttle_acc;
	float aileron_acc;
	float elevator_acc;
	float rudder_acc;
	float throttle_s;
	float aileron_s;
	float elevator_s;
	float rudder_s;
	
	float error_pitch;
	float error_roll;
	float error_yaw;
	float error_pitch_z;
	float error_roll_z;
	float error_yaw_z;
	float error_pitch_i;
	float error_roll_i;
	float error_yaw_i;
	float error_pitch_i_max;
	float error_roll_i_max;
	float error_yaw_i_max;
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
	
	int32_t t;
	uint16_t time_mpu;
	uint16_t time_process;
	
	_Bool armed_unlock_step1;
	
	/* Variable init ---------------------------------------*/
	
	flag_mpu = 0;
	flag_radio = 0;
	flag_motor = 0;
	flag_host = 0;
	flag_reg = 1;
	flag_mpu_host_read = 0;
	flag_radio_synch = 1;
	flag_mpu_cal = 1;
	flag_vbat = 0;
	flag_armed_locked = 1;

	flag_beep_user = 0;
	flag_beep_radio = 0;
	flag_beep_mpu = 0;
	flag_beep_host = 0;
	flag_beep_vbat = 0;
	
	radio_frame_count = 0;
	radio_error_count = 0;

	throttle = 0;
	aileron = 0;
	elevator = 0;
	rudder = 0;
	armed = 0;
	chan6 = 0;
	
	mpu_sample_count = 0;
	mpu_error_count = 0;
	
	gyro_x_dc = 0;
	gyro_y_dc = 0;
	gyro_z_dc = 0;
	gyro_x_scale = 0.061035f;
	gyro_y_scale = 0.061035f;
	gyro_z_scale = 0.061035f;
	accel_x_scale = 0.00048828f;
	accel_y_scale = 0.00048828f;
	accel_z_scale = 0.00048828f;
	
	throttle_acc = 0;
	aileron_acc = 0;
	elevator_acc = 0;
	rudder_acc = 0;
	
	error_pitch_i = 0;
	error_roll_i = 0;
	error_yaw_i = 0;
	
	vbat_acc = 15.0f / VBAT_ALPHA;
	vbat_sample_count = 0;
	
	armed_unlock_step1 = 0;
	
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
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	// SPI clock enable
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	// Timer clock enable
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN | RCC_APB1ENR_TIM5EN | RCC_APB1ENR_TIM6EN | RCC_APB1ENR_TIM7EN | RCC_APB1ENR_TIM12EN | RCC_APB1ENR_TIM13EN;
	// System configuration controller clock enable (to manage external interrupt line connection to GPIOs)
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	// DMA clock enable
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;
	// GPIO clock enable 
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
	// ADC clock enable
	//RCC->AHBENR |= RCC_AHBENR_ADC12EN;
	// USB clock enable
	RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
	
	/* GPIO ------------------------------------------------*/
	
	// MODER: 00:IN, 01:OUT, 10:AF, 11:analog
	// PUPDR: 00:float 01:PU, 10:PD
	// OTYPER: 0:PP, 1:OD
	// OSPEEDR: 00:low 4MHz, 01:mid 25MHz, 10:high 50MHz, 11:very high 100MHz
	
	// Reset non-zero registers
	GPIOA->MODER = 0;
	GPIOA->PUPDR = 0;
	GPIOA->OSPEEDR = 0;
	GPIOB->MODER = 0;
	GPIOB->PUPDR = 0;
	GPIOB->OSPEEDR = 0;
	
	// A0 : Servo 6, used as beeper
	GPIOA->MODER |= GPIO_MODER_MODER0_0;
	// A1 : Servo 5, TIM5_CH2, AF2, DMA1 Stream 4
	// A2 : Servo 4, TIM2_CH3, AF1, DMA1 Stream 1
	// A3 : Servo 3, TIM5_CH4, AF2, DMA1 Stream 3
	GPIOA->MODER |= GPIO_MODER_MODER1_1 | GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR1_1 | GPIO_OSPEEDER_OSPEEDR2_1 | GPIO_OSPEEDER_OSPEEDR3_1;
	GPIOA->AFR[0] |= (2 << GPIO_AFRL_AFSEL1_Pos) | (1 << GPIO_AFRL_AFSEL2_Pos) | (2 << GPIO_AFRL_AFSEL3_Pos);
	// A4 : SPI1 CS, AF5, need open-drain (external pull-up)
	// A5 : SPI1 CLK, AF5, need pull-up
	// A6 : SPI1 MISO, AF5, DMA2 Stream 3
	// A7 : SPI1 MOSI, AF5, DMA2 Stream 0
	GPIOA->MODER |= GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4_1 | GPIO_OSPEEDER_OSPEEDR5_1 | GPIO_OSPEEDER_OSPEEDR7_1;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR5_0;
	GPIOA->OTYPER |= GPIO_OTYPER_OT_4;
	GPIOA->AFR[0] |= (5 << GPIO_AFRL_AFSEL4_Pos) | (5 << GPIO_AFRL_AFSEL5_Pos) | (5 << GPIO_AFRL_AFSEL6_Pos) | (5 << GPIO_AFRL_AFSEL7_Pos);
	// A9 : UART1 Tx
	// A10: UART1 Rx, AF7, DMA2 Stream 5, need pull-up (for IDLE)
	GPIOA->MODER |= GPIO_MODER_MODER10_1;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR10_0;
	GPIOA->AFR[1] |= 7 << GPIO_AFRH_AFSEL10_Pos;
	// A11: USB_DM, AF10
	// A12: USB_DP, AF10
	GPIOA->MODER |= GPIO_MODER_MODER11_1 | GPIO_MODER_MODER12_1;
	GPIOA->AFR[1] |= (10 << GPIO_AFRH_AFSEL11_Pos) | (10 << GPIO_AFRH_AFSEL12_Pos);
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR11 | GPIO_OSPEEDER_OSPEEDR12;
	// A15: SPI3 CS, AF6
	// B0 : Servo 1
	// B1 : Servo 2, TIM3_CH4, AF2, DMA1 Stream 2
	GPIOB->MODER |= GPIO_MODER_MODER1_1;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR1_1;
	GPIOB->AFR[0] |= 2 << GPIO_AFRL_AFSEL1_Pos;
	// B4 : Red LED, need open-drain (external pull-up)
	GPIOB->MODER |= GPIO_MODER_MODER4_0;
	GPIOB->OTYPER |= GPIO_OTYPER_OT_4;
	// B5 : Blue LED, need open-drain (external pull-up)
	GPIOB->MODER |= GPIO_MODER_MODER5_0;
	GPIOB->OTYPER |= GPIO_OTYPER_OT_5;
	// B8 : I2C1 SCL, AF4
	// B9 : I2C1 SDA, AF4, Rx: DMA1 Stream 5 Tx: DMA1 Stream 6
	// B12: Input 3
	// B13: Input 4
	// B14: Input 5
	// B15: Input 6
#ifdef DEBUG
	GPIOB->MODER |= GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0;
#endif
	// C0 : SBUS invert
	GPIOC->MODER |= GPIO_MODER_MODER0_0;
	// C1 : Current sensor
	// C2 : Voltage sensor, ADC1
	GPIOC->MODER |= GPIO_MODER_MODER2;
	// C4 : MPU interrupt
	// C5 : 5V sensor
	// C6 : Input 7
	// C7 : Input 8
	// C8 : Input 9
	// C9 : Input 10
#ifdef DEBUG
	GPIOC->MODER |= GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0;
#endif
	// C10: SPI3 CLK, AF6
	// C11: SPI3 MISO, AF6, DMA1 Stream 7
	// C12: SPI3 MOSI, AF6, DMA1 Stream 0
	
	/* DMA --------------------------------------------------------------------------*/
	
	// DMA[y]_Stream[x]->CR = (3 << DMA_SxCR_CHSEL_Pos) | (1 << DMA_SxCR_PL_Pos) | (1 << DMA_SxCR_MSIZE_Pos) | (1 << DMA_SxCR_PSIZE_Pos) | DMA_SxCR_MINC | DMA_SxCR_PINC | (1 << DMA_SxCR_DIR_Pos) | DMA_SxCR_TCIE | DMA_SxCR_TEIE;

	// SPI1 Rx
	DMA2_Stream0->CR = (3 << DMA_SxCR_CHSEL_Pos) | (1 << DMA_SxCR_PL_Pos) | DMA_SxCR_MINC | DMA_SxCR_TCIE | DMA_SxCR_TEIE;
	DMA2_Stream0->M0AR = (uint32_t)spi1_rx_buffer;
	DMA2_Stream0->PAR = (uint32_t)&(SPI1->DR);

	// SPI1 Tx
	DMA2_Stream3->CR = (3 << DMA_SxCR_CHSEL_Pos) | (3 << DMA_SxCR_PL_Pos) | DMA_SxCR_MINC | (1 << DMA_SxCR_DIR_Pos) | DMA_SxCR_TEIE;
	DMA2_Stream3->M0AR = (uint32_t)spi1_tx_buffer;
	DMA2_Stream3->PAR = (uint32_t)&(SPI1->DR);

	// UART1 Rx
	DMA2_Stream5->CR = (4 << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_TCIE | DMA_SxCR_TEIE;
	DMA2_Stream5->M0AR = (uint32_t)uart1_rx_buffer;
	DMA2_Stream5->PAR = (uint32_t)&(USART1->DR);
	
	// Timers for DSHOT
	DMA1_Stream2->CR = (5 << DMA_SxCR_CHSEL_Pos) | (2 << DMA_SxCR_PL_Pos) | (2 << DMA_SxCR_MSIZE_Pos) | (2 << DMA_SxCR_PSIZE_Pos) | DMA_SxCR_MINC | (1 << DMA_SxCR_DIR_Pos);
	DMA1_Stream2->M0AR = (uint32_t)motor1_dshot;
	DMA1_Stream2->PAR = (uint32_t)&(TIM3->CCR4);
	
	DMA1_Stream3->CR = (6 << DMA_SxCR_CHSEL_Pos) | (2 << DMA_SxCR_PL_Pos) | (2 << DMA_SxCR_MSIZE_Pos) | (2 << DMA_SxCR_PSIZE_Pos) | DMA_SxCR_MINC | (1 << DMA_SxCR_DIR_Pos);
	DMA1_Stream3->M0AR = (uint32_t)motor2_dshot;
	DMA1_Stream3->PAR = (uint32_t)&(TIM5->CCR4);
	
	DMA1_Stream1->CR = (3 << DMA_SxCR_CHSEL_Pos) | (2 << DMA_SxCR_PL_Pos) | (2 << DMA_SxCR_MSIZE_Pos) | (2 << DMA_SxCR_PSIZE_Pos) | DMA_SxCR_MINC | (1 << DMA_SxCR_DIR_Pos);
	DMA1_Stream1->M0AR = (uint32_t)motor3_dshot;
	DMA1_Stream1->PAR = (uint32_t)&(TIM2->CCR3);
	
	DMA1_Stream4->CR = (6 << DMA_SxCR_CHSEL_Pos) | (2 << DMA_SxCR_PL_Pos) | (2 << DMA_SxCR_MSIZE_Pos) | (2 << DMA_SxCR_PSIZE_Pos) | DMA_SxCR_MINC | (1 << DMA_SxCR_DIR_Pos);
	DMA1_Stream4->M0AR = (uint32_t)motor4_dshot;
	DMA1_Stream4->PAR = (uint32_t)&(TIM5->CCR2);
	
	/* Timers --------------------------------------------------------------------------*/
	
#ifdef ESC_DSHOT
	// DMA driven timer for DShot600, 12Mhz, 0:15, 1:30, T:40
	TIM2->PSC = 0;
	TIM2->ARR = 80;
	TIM2->DIER = TIM_DIER_CC3DE;
	TIM2->CCER = TIM_CCER_CC3E;
	TIM2->CCMR2 = (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
	
	TIM3->PSC = 0;
	TIM3->ARR = 80;
	TIM3->DIER = TIM_DIER_CC4DE;
	TIM3->CCER = TIM_CCER_CC4E;
	TIM3->CCMR2 = (6 << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
	
	TIM5->PSC = 0;
	TIM5->ARR = 80;
	TIM5->DIER = TIM_DIER_CC2DE | TIM_DIER_CC4DE;
	TIM5->CCER = TIM_CCER_CC2E | TIM_CCER_CC4E;
	TIM5->CCMR1 = (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
	TIM5->CCMR2 = (6 << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
#else	
	// One-pulse mode for OneShot125
	TIM2->CR1 = TIM_CR1_OPM;
	TIM2->PSC = 5;
	TIM2->ARR = MOTOR_MAX + 1;
	TIM2->CCER = TIM_CCER_CC3E;
	TIM2->CCMR2 = 7 << TIM_CCMR2_OC3M_Pos;
	
	TIM3->CR1 = TIM_CR1_OPM;
	TIM3->PSC = 5;
	TIM3->ARR = MOTOR_MAX + 1;
	TIM3->CCER = TIM_CCER_CC4E;
	TIM3->CCMR2 = 7 << TIM_CCMR2_OC4M_Pos;
	
	TIM5->CR1 = TIM_CR1_OPM;
	TIM5->PSC = 5;
	TIM5->ARR = MOTOR_MAX + 1;
	TIM5->CCER = TIM_CCER_CC2E | TIM_CCER_CC4E;
	TIM5->CCMR1 = 7 << TIM_CCMR1_OC2M_Pos;
	TIM5->CCMR2 = 7 << TIM_CCMR2_OC4M_Pos;
#endif

	// Beeper
	TIM4->PSC = 23999; // 1ms
	TIM4->ARR = BEEPER_PERIOD;
	TIM4->DIER = TIM_DIER_UIE;
	//TIM4->CR1 = TIM_CR1_CEN;
	
	// Receiver timeout
	TIM6->PSC = 23999; // 1ms
	TIM6->ARR = TIMEOUT_RADIO;
	TIM6->DIER = TIM_DIER_UIE;
	TIM6->CR1 = TIM_CR1_CEN;
	
	// Processing time
	TIM7->PSC = 23; // 1us
	TIM7->ARR = 65535;
	TIM7->CR1 = TIM_CR1_CEN;
	
	// MPU timeout
	TIM12->PSC = 23; // 1us
	TIM12->ARR = TIMEOUT_MPU;
	TIM12->DIER = TIM_DIER_UIE;
	TIM12->CR1 = TIM_CR1_CEN;
	
	// VBAT
	TIM13->PSC = 23999; // 1ms
	TIM13->ARR = VBAT_PERIOD;
	TIM13->DIER = TIM_DIER_UIE;
	//TIM13->CR1 = TIM_CR1_CEN;
	
	/* UART ---------------------------------------------------*/

#if (RADIO_TYPE == 2)
	GPIOC->BSRR = GPIO_BSRR_BS_0; // Invert Rx
	USART1->BRR = 480; // 48MHz/100000bps
	USART1->CR1 = USART_CR1_RE | (1 << USART_CR1_M_Pos) | USART_CR1_PCE;
	USART1->CR2 = (2 << USART_CR2_STOP_Pos);// | USART_CR2_MSBFIRST ??;
#else
	GPIOC->BSRR = GPIO_BSRR_BR_0; // Do not invert Rx
	USART1->BRR = 417; // 48MHz/115200bps
	USART1->CR1 = USART_CR1_RE;
#endif
	USART1->CR3 = USART_CR3_EIE | USART_CR3_DMAR;
	
	/* SPI ----------------------------------------------------*/
	
	SPI1->CR1 = SPI_CR1_MSTR | (5 << SPI_CR1_BR_Pos) | SPI_CR1_CPOL | SPI_CR1_CPHA; // SPI clock = clock APB1/64 = 48MHz/64 = 750 kHz
	SPI1->CR2 = SPI_CR2_SSOE | SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN | SPI_CR2_ERRIE;

	/* ADC -----------------------------------------------------*/
	/*
	ADC1->CR2 |= ADC_CR2_ADON;
	wait_ms(1);
	
	ADC1->SMPR2 = 4 << ADC_SMPR2_SMP2_Pos;
	ADC1->SQR3 = 2 << ADC_SQR3_SQ1_Pos;
	*/
	/* Interrupts ---------------------------------------------------*/
	
	// MPU interrrupt
	SYSCFG->EXTICR[1] = SYSCFG_EXTICR2_EXTI4_PC;
	EXTI->RTSR |= EXTI_RTSR_TR4;
	//EXTI->IMR = EXTI_IMR_MR4; // To be enabled after MPU init
	
	NVIC_EnableIRQ(EXTI4_IRQn);
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_EnableIRQ(SPI1_IRQn);
	NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	NVIC_EnableIRQ(DMA2_Stream3_IRQn);
	NVIC_EnableIRQ(DMA2_Stream5_IRQn);
	NVIC_EnableIRQ(TIM4_IRQn);
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
	NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
	NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
	NVIC_EnableIRQ(OTG_FS_IRQn);
	
	NVIC_SetPriority(EXTI4_IRQn,0);
	NVIC_SetPriority(USART1_IRQn,0);
	NVIC_SetPriority(SPI1_IRQn,0);
	NVIC_SetPriority(DMA2_Stream0_IRQn,0);
	NVIC_SetPriority(DMA2_Stream3_IRQn,0);
	NVIC_SetPriority(DMA2_Stream5_IRQn,0);
	NVIC_SetPriority(TIM4_IRQn,0);
	NVIC_SetPriority(TIM6_DAC_IRQn,0);
	NVIC_SetPriority(TIM8_BRK_TIM12_IRQn,0);
	NVIC_SetPriority(TIM8_UP_TIM13_IRQn,0);
	NVIC_SetPriority(OTG_FS_IRQn,16);

	/* USB ----------------------------------------------------------*/
	
	USBD_Init(&USBD_device_handler, &USBD_VCP_descriptor, 0);
	USBD_RegisterClass(&USBD_device_handler, &USBD_CDC);
	USBD_CDC_RegisterInterface(&USBD_device_handler, &USBD_CDC_IF_fops);
	USBD_Start(&USBD_device_handler);
	
	/* MPU init ----------------------------------------------------*/
	
	MpuWrite(MPU_PWR_MGMT_1, MPU_PWR_MGMT_1__DEVICE_RST);
	wait_ms(100);
	MpuWrite(MPU_SIGNAL_PATH_RST, MPU_SIGNAL_PATH_RST__ACCEL_RST | MPU_SIGNAL_PATH_RST__GYRO_RST | MPU_SIGNAL_PATH_RST__TEMP_RST);
	wait_ms(100);
	MpuWrite(MPU_USER_CTRL, MPU_USER_CTRL__I2C_IF_DIS);
	wait_ms(1);
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
	
	SPI1->CR1 &= ~SPI_CR1_BR_Msk | (1 << SPI_CR1_BR_Pos); // SPI clock = clock APB2/2 = 48MHz/4 = 12MHz
	
	EXTI->IMR = EXTI_IMR_MR4; // Enable interrupt now to avoid problem with new SPI clock settings
	
	SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk; // Disable Systick interrupt, not needed anymore (but can still use COUNTFLAG)

	/* Main loop ---------------------------------------------------------------------
	-----------------------------------------------------------------------------------*/
	
	while (1)
	{
		// Processing time
		time[2] = TIM7->CNT;
		
		#ifdef DEBUG
			GPIOB->BSRR = GPIO_BSRR_BS_13;
		#endif
		
		/* Update variables depending on register ---------------------------------------------------*/
		
		if (flag_reg)
		{
			flag_reg = 0;
		
			// Adapt SPI clock frequency
			if (REG_CTRL__MPU_HOST_CTRL)
				SPI1->CR1 |= (5 << SPI_CR1_BR_Pos); // 750 kHz
			else
				SPI1->CR1 &= ~SPI_CR1_BR_Msk | (1 << SPI_CR1_BR_Pos); // 12 MHz
			
			// Manual contol of LED
			if (REG_CTRL__LED_SELECT == 0)
			{
				GPIOB->BSRR = GPIO_BSRR_BS_4;
				GPIOB->BSRR = GPIO_BSRR_BS_5;
			}
			else if (REG_CTRL__LED_SELECT == 1)
			{
				GPIOB->BSRR = GPIO_BSRR_BS_4;
				GPIOB->BSRR = GPIO_BSRR_BR_5;
			}
			else if (REG_CTRL__LED_SELECT == 3)
			{
				GPIOB->BSRR = GPIO_BSRR_BR_4;
				GPIOB->BSRR = GPIO_BSRR_BS_5;
			}
			
			// Expo
			x = REG_PITCH_ROLL_EXPO;
			pitch_roll_expo_scale = x;
			x = x * REG_PITCH_ROLL_EXPO;
			pitch_roll_expo_scale += x / 2.0f;
			x = x * REG_PITCH_ROLL_EXPO;
			pitch_roll_expo_scale += x / 6.0f;
			x = x * REG_PITCH_ROLL_EXPO;
			pitch_roll_expo_scale += x / 24.0f;
			x = x * REG_PITCH_ROLL_EXPO;
			pitch_roll_expo_scale += x / 120.0f;
			x = x * REG_PITCH_ROLL_EXPO;
			pitch_roll_expo_scale += x / 720.0f;
			
			x = REG_YAW_EXPO;
			yaw_expo_scale = x;
			x = x * REG_YAW_EXPO;
			yaw_expo_scale += x / 2.0f;
			x = x * REG_YAW_EXPO;
			yaw_expo_scale += x / 6.0f;
			x = x * REG_YAW_EXPO;
			yaw_expo_scale += x / 24.0f;
			x = x * REG_YAW_EXPO;
			yaw_expo_scale += x / 120.0f;
			x = x * REG_YAW_EXPO;
			yaw_expo_scale += x / 720.0f;
			
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
		
		/* Radio frame synchronisation -------------------------------------------------*/
		
		if (flag_radio_synch)
		{
			flag_radio_synch = 0;
			
			// Disable DMA
			USART1->CR1 &= ~USART_CR1_UE;			
			USART1->CR3 &= ~USART_CR3_DMAR;
			USART1->CR1 |= USART_CR1_UE;
			
			// Synchonise on IDLE character
			USART1->SR;
			USART1->DR;
			while ((USART1->SR & USART_SR_IDLE) == 0)
				USART1->DR;
			
			// Enable DMA
			USART1->CR1 &= ~USART_CR1_UE;
			USART1->CR3 |= USART_CR3_DMAR;
			USART1->CR1 |= USART_CR1_UE;
			DMA2_Stream5->NDTR = RADIO_NB_BYTES;
			DMA2_Stream5->CR |= DMA_SxCR_EN;
		}
		
		/* Process radio commands -----------------------------------------------------*/
		
		if (flag_radio)
		{
			flag_radio = 0;
			
			// Check header
			// TODO: verify checksum
		#if (RADIO_TYPE == 0)
			if ((uart1_rx_buffer[0] != 0x20) || (uart1_rx_buffer[1] != 0x40))
		#elif (RADIO_TYPE == 1)
			if ((uart1_rx_buffer[0] != 0xA8) || ((uart1_rx_buffer[1] != 0x01) && (uart1_rx_buffer[1] != 0x81)))
		#else
			if ((uart1_rx_buffer[0] != 0xF0) || (uart1_rx_buffer[24] != 0x00))
		#endif
			{
				flag_radio_synch = 1;
				radio_error_count++;
			}
			else
			{
				// Reset timeout
				TIM6->CNT = 0;
				flag_beep_radio = 0;
				
				radio_frame_count++;
				#if (RADIO_TYPE == 0)
					aileron_raw  = (uint16_t)uart1_rx_buffer[ 2] | ((uint16_t)uart1_rx_buffer[ 3] << 8);
					elevator_raw = (uint16_t)uart1_rx_buffer[ 4] | ((uint16_t)uart1_rx_buffer[ 5] << 8);
					throttle_raw = (uint16_t)uart1_rx_buffer[ 6] | ((uint16_t)uart1_rx_buffer[ 7] << 8);
					rudder_raw   = (uint16_t)uart1_rx_buffer[ 8] | ((uint16_t)uart1_rx_buffer[ 9] << 8);
					armed_raw    = (uint16_t)uart1_rx_buffer[10] | ((uint16_t)uart1_rx_buffer[11] << 8);
					chan6_raw    = (uint16_t)uart1_rx_buffer[12] | ((uint16_t)uart1_rx_buffer[13] << 8);
				#elif (RADIO_TYPE == 1)
					throttle_raw = (uint16_t)uart1_rx_buffer[ 4] | ((uint16_t)uart1_rx_buffer[ 3] << 8);
					aileron_raw  = (uint16_t)uart1_rx_buffer[ 6] | ((uint16_t)uart1_rx_buffer[ 5] << 8);
					elevator_raw = (uint16_t)uart1_rx_buffer[ 8] | ((uint16_t)uart1_rx_buffer[ 7] << 8);
					rudder_raw   = (uint16_t)uart1_rx_buffer[10] | ((uint16_t)uart1_rx_buffer[ 9] << 8);
					armed_raw    = (uint16_t)uart1_rx_buffer[12] | ((uint16_t)uart1_rx_buffer[11] << 8);
					chan6_raw    = (uint16_t)uart1_rx_buffer[14] | ((uint16_t)uart1_rx_buffer[13] << 8);
				#else
					throttle_raw = ((uart1_rx_buffer[1]    |uart1_rx_buffer[2]<<8)                          & 0x07FF);
					aileron_raw  = ((uart1_rx_buffer[2]>>3 |uart1_rx_buffer[3]<<5)                          & 0x07FF);
					elevator_raw = ((uart1_rx_buffer[3]>>6 |uart1_rx_buffer[4]<<2 |uart1_rx_buffer[5]<<10)  & 0x07FF);
					rudder_raw   = ((uart1_rx_buffer[5]>>1 |uart1_rx_buffer[6]<<7)                          & 0x07FF);
					armed_raw    = ((uart1_rx_buffer[6]>>4 |uart1_rx_buffer[7]<<4)                          & 0x07FF);
					chan6_raw    = ((uart1_rx_buffer[7]>>7 |uart1_rx_buffer[8]<<1 |uart1_rx_buffer[9]<<9)   & 0x07FF);
				#endif
			}
			
			#if (RADIO_TYPE == 0)
				throttle = (float)(throttle_raw - 1000) / 1000.0f;
				aileron = (float)((int16_t)aileron_raw - 1500) / 500.0f;
				elevator = (float)((int16_t)elevator_raw - 1500) / 500.0f;
				rudder = (float)((int16_t)rudder_raw - 1500) / 500.0f;
				armed1 = (float)(armed_raw - 1000) / 1000.0f;
				chan6 = (float)(chan6_raw - 1000) / 1000.0f;
			#elif (RADIO_TYPE == 1)
				throttle = (float)(throttle_raw - 8800) / 6400.0f;
				aileron = (float)((int16_t)aileron_raw - 12000) / 3200.0f;
				elevator = (float)((int16_t)elevator_raw - 12000) / 3200.0f;
				rudder = (float)((int16_t)rudder_raw - 12000) / 3200.0f;
				armed1 = (float)(armed_raw - 8800) / 6400.0f;
				chan6 = (float)(chan6_raw - 8800) / 6400.0f;
			#else
				throttle = (float)(throttle_raw - 1000) / 1000.0f;
				aileron = (float)((int16_t)aileron_raw - 1500) / 500.0f;
				elevator = (float)((int16_t)elevator_raw - 1500) / 500.0f;
				rudder = (float)((int16_t)rudder_raw - 1500) / 500.0f;
				armed1 = (float)(armed_raw - 1000) / 1000.0f;
				chan6 = (float)(chan6_raw - 1000) / 1000.0f;
			#endif
			
			if (flag_armed_locked)
			{
				armed = 0;
				
				if (!armed_unlock_step1 && (armed1 > 0.5f))
					armed_unlock_step1 = 1;
				if (armed_unlock_step1 && (armed1 < 0.5f) && (throttle < 0.01f))
					flag_armed_locked = 0;
			}
			else
				armed = armed1;
			
			if (REG_PITCH_ROLL_EXPO > 0.0f)
			{
				if (aileron >= 0)
					aileron = expo(aileron, pitch_roll_expo_scale, REG_PITCH_ROLL_EXPO);
				else
					aileron = -expo(-aileron, pitch_roll_expo_scale, REG_PITCH_ROLL_EXPO);
				
				if (elevator >= 0)
					elevator = expo(elevator, pitch_roll_expo_scale, REG_PITCH_ROLL_EXPO);
				else
					elevator = -expo(-elevator, pitch_roll_expo_scale, REG_PITCH_ROLL_EXPO);
			}
			
			if (REG_YAW_EXPO > 0.0f)
			{
				if (rudder >= 0)
					rudder = expo(rudder, yaw_expo_scale, REG_YAW_EXPO);
				else
					rudder = -expo(-rudder, yaw_expo_scale, REG_YAW_EXPO);
			}
			
			// Send raw radio commands to host
			if ((REG_DEBUG__CASE == 4) && ((radio_frame_count & REG_DEBUG__MASK) == 0))
			{
				USBD_CDC_IF_tx_buffer[ 0] = (uint8_t) throttle_raw;
				USBD_CDC_IF_tx_buffer[ 1] = (uint8_t)(throttle_raw >> 8);
				USBD_CDC_IF_tx_buffer[ 2] = (uint8_t) aileron_raw;
				USBD_CDC_IF_tx_buffer[ 3] = (uint8_t)(aileron_raw>> 8);
				USBD_CDC_IF_tx_buffer[ 4] = (uint8_t) elevator_raw;
				USBD_CDC_IF_tx_buffer[ 5] = (uint8_t)(elevator_raw>> 8);
				USBD_CDC_IF_tx_buffer[ 6] = (uint8_t) rudder_raw;
				USBD_CDC_IF_tx_buffer[ 7] = (uint8_t)(rudder_raw >> 8);
				USBD_CDC_IF_tx_buffer[ 8] = (uint8_t) armed_raw;
				USBD_CDC_IF_tx_buffer[ 9] = (uint8_t)(armed_raw >> 8);
				USBD_CDC_IF_tx_buffer[10] = (uint8_t) chan6_raw;
				USBD_CDC_IF_tx_buffer[11] = (uint8_t)(chan6_raw >> 8);
				USB_TX(12)
			}
			
			// Beep if requested
			if (chan6 > 0.5f)
				flag_beep_user = 1;
			else
				flag_beep_user = 0;
			
			// Toggle LED at rate of Radio flag
			if (REG_CTRL__LED_SELECT == 2)
			{
				if ((radio_frame_count & 0x7F) == 0)
					GPIOB->BSRR = GPIO_BSRR_BR_4;
				else if ((radio_frame_count & 0x7F) == 64)
					GPIOB->BSRR = GPIO_BSRR_BS_4;
			}
		}
		
		/* Process sensors -----------------------------------------------------------------------*/
		
		if (flag_mpu)
		{
			flag_mpu = 0;
			
			if ((REG_CTRL__MPU_HOST_CTRL == 0) && ((spi1_rx_buffer[1] & MPU_INT_STATUS__DATA_RDY_INT) == 0))
			{
				mpu_error_count++;
			}
			else if (REG_CTRL__MPU_HOST_CTRL == 0)
			{
				// Reset timeout
				TIM12->CNT = 0;
				flag_beep_mpu = 0;
				
				mpu_sample_count++;
				
				accel_x_raw     = ((int16_t)spi1_rx_buffer[ 2] << 8) | (int16_t)spi1_rx_buffer[ 3];
				accel_y_raw     = ((int16_t)spi1_rx_buffer[ 4] << 8) | (int16_t)spi1_rx_buffer[ 5];
				accel_z_raw     = ((int16_t)spi1_rx_buffer[ 6] << 8) | (int16_t)spi1_rx_buffer[ 7];
				temperature_raw = ((int16_t)spi1_rx_buffer[ 8] << 8) | (int16_t)spi1_rx_buffer[ 9];
				gyro_x_raw      = ((int16_t)spi1_rx_buffer[10] << 8) | (int16_t)spi1_rx_buffer[11];
				gyro_y_raw      = ((int16_t)spi1_rx_buffer[12] << 8) | (int16_t)spi1_rx_buffer[13];
				gyro_z_raw      = ((int16_t)spi1_rx_buffer[14] << 8) | (int16_t)spi1_rx_buffer[15];
				
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
					}
				}
				
				// Record SPI transaction time
				t = (int32_t)time[1] - (int32_t)time[0];
				if (t < 0)
					t += 65536;
				if ((REG_CTRL__TIME_MAXHOLD == 0) || (((uint16_t)t > time_mpu) && REG_CTRL__TIME_MAXHOLD))
					time_mpu = (uint16_t)t;
			}
			else if (flag_mpu_host_read)
			{
				flag_mpu_host_read = 0;
				USBD_CDC_IF_tx_buffer[0] = spi1_rx_buffer[1];
				USB_TX(1)
			}
			
			// Send Raw sensor values to host
			if ((REG_DEBUG__CASE == 1) && ((mpu_sample_count & REG_DEBUG__MASK) == 0))
			{
				USBD_CDC_IF_tx_buffer[ 0] = (uint8_t) gyro_x_raw;
				USBD_CDC_IF_tx_buffer[ 1] = (uint8_t)(gyro_x_raw >> 8);
				USBD_CDC_IF_tx_buffer[ 2] = (uint8_t) gyro_y_raw;
				USBD_CDC_IF_tx_buffer[ 3] = (uint8_t)(gyro_y_raw >> 8);
				USBD_CDC_IF_tx_buffer[ 4] = (uint8_t) gyro_z_raw;
				USBD_CDC_IF_tx_buffer[ 5] = (uint8_t)(gyro_z_raw >> 8);
				USBD_CDC_IF_tx_buffer[ 6] = (uint8_t) accel_x_raw;
				USBD_CDC_IF_tx_buffer[ 7] = (uint8_t)(accel_x_raw >> 8);
				USBD_CDC_IF_tx_buffer[ 8] = (uint8_t) accel_y_raw;
				USBD_CDC_IF_tx_buffer[ 9] = (uint8_t)(accel_y_raw >> 8);
				USBD_CDC_IF_tx_buffer[10] = (uint8_t) accel_z_raw;
				USBD_CDC_IF_tx_buffer[11] = (uint8_t)(accel_z_raw >> 8);
				USBD_CDC_IF_tx_buffer[12] = (uint8_t) temperature_raw;
				USBD_CDC_IF_tx_buffer[13] = (uint8_t)(temperature_raw >> 8);
				USB_TX(14)
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
			if ((REG_DEBUG__CASE == 2) && ((mpu_sample_count & REG_DEBUG__MASK) == 0))
			{
				float_to_bytes(&gyro_x, &USBD_CDC_IF_tx_buffer[0]);
				float_to_bytes(&gyro_y, &USBD_CDC_IF_tx_buffer[4]);
				float_to_bytes(&gyro_z, &USBD_CDC_IF_tx_buffer[8]);
				float_to_bytes(&accel_x, &USBD_CDC_IF_tx_buffer[12]);
				float_to_bytes(&accel_y, &USBD_CDC_IF_tx_buffer[16]);
				float_to_bytes(&accel_z, &USBD_CDC_IF_tx_buffer[20]);
				float_to_bytes(&temperature, &USBD_CDC_IF_tx_buffer[24]);
				USB_TX(28)
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
			if ((REG_DEBUG__CASE == 5) && ((mpu_sample_count & REG_DEBUG__MASK) == 0))
			{
				float_to_bytes(&throttle_s, &USBD_CDC_IF_tx_buffer[0]);
				float_to_bytes(&aileron_s, &USBD_CDC_IF_tx_buffer[4]);
				float_to_bytes(&elevator_s, &USBD_CDC_IF_tx_buffer[8]);
				float_to_bytes(&rudder_s, &USBD_CDC_IF_tx_buffer[12]);
				float_to_bytes(&armed, &USBD_CDC_IF_tx_buffer[16]);
				float_to_bytes(&chan6, &USBD_CDC_IF_tx_buffer[20]);
				USB_TX(24)
			}
			
			// Convert command into gyro rate
			pitch_rate = elevator_s * REG_PITCH_ROLL_RATE;
			roll_rate = aileron_s * REG_PITCH_ROLL_RATE;
			yaw_rate = rudder_s * REG_YAW_RATE;
			
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
			pitch = (error_pitch * REG_PITCH_P) + (error_pitch_i * REG_PITCH_I) + ((error_pitch - error_pitch_z) * REG_PITCH_D);
			roll  = (error_roll  * REG_ROLL_P ) + (error_roll_i  * REG_ROLL_I ) + ((error_roll  - error_roll_z ) * REG_ROLL_D);
			yaw   = (error_yaw   * REG_YAW_P  ) + (error_yaw_i   * REG_YAW_I  ) + ((error_yaw   - error_yaw_z  ) * REG_YAW_D);
			
			// Send pitch/roll/yaw to host
			if ((REG_DEBUG__CASE == 6) && ((mpu_sample_count & REG_DEBUG__MASK) == 0))
			{
				float_to_bytes(&pitch, &USBD_CDC_IF_tx_buffer[0]);
				float_to_bytes(&roll, &USBD_CDC_IF_tx_buffer[4]);
				float_to_bytes(&yaw, &USBD_CDC_IF_tx_buffer[8]);
				USB_TX(12)
			}
			
			// Convert throttle into motor command
			throttle_rate = throttle_s * REG_THROTTLE_RANGE;
			
			// Attenuation when high throttle
			throttle_gain = 1.0f - (throttle_s *  REG_THROTTLE_ATTEN);
			
			// Motor matrix
			motor[0] = throttle_rate + ((  roll + pitch - yaw) * throttle_gain);
			motor[1] = throttle_rate + ((  roll - pitch + yaw) * throttle_gain);
			motor[2] = throttle_rate + ((- roll - pitch - yaw) * throttle_gain);
			motor[3] = throttle_rate + ((- roll + pitch + yaw) * throttle_gain);
			
			// Send motor actions to host
			if ((REG_DEBUG__CASE == 7) && ((mpu_sample_count & REG_DEBUG__MASK) == 0))
			{
				for (i=0; i<4; i++)
					float_to_bytes(&motor[i], &USBD_CDC_IF_tx_buffer[i*4]);
				USB_TX(16)
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
			if ((REG_DEBUG__CASE == 8) && ((mpu_sample_count & REG_DEBUG__MASK) == 0))
			{
				for (i=0; i<4; i++)
				{
					USBD_CDC_IF_tx_buffer[i*2+0] = (uint8_t) motor_raw[i];
					USBD_CDC_IF_tx_buffer[i*2+1] = (uint8_t)(motor_raw[i] >> 8);
				}
				USB_TX(8)
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
				TIM3->CCR4 = MOTOR_MAX + 1 - MOTOR_MIN - (motor_raw[0]>>1);
				TIM5->CCR4 = MOTOR_MAX + 1 - MOTOR_MIN - (motor_raw[1]>>1);
				TIM2->CCR3 = MOTOR_MAX + 1 - MOTOR_MIN - (motor_raw[2]>>1);
				TIM5->CCR2 = MOTOR_MAX + 1 - MOTOR_MIN - (motor_raw[3]>>1);
			#endif
			TIM2->CR1 |= TIM_CR1_CEN;
			TIM3->CR1 |= TIM_CR1_CEN;
			TIM5->CR1 |= TIM_CR1_CEN;
		}
		
		/* VBAT ---------------------------------------------------------------------*/
		
		if (flag_vbat)
		{
			flag_vbat = 0;
			
			vbat_sample_count++;
			/*
			if (ADC2->ISR & ADC_ISR_EOC)
				vbat = (float)ADC2->DR * ADC_SCALE;
			*/
			vbat = 0;
			
			vbat_acc = vbat_acc * (1.0f - VBAT_ALPHA) + vbat;
			vbat = vbat_acc * VBAT_ALPHA;
			
			REG_VBAT = vbat;
			
			// Send VBAT to host
			if ((REG_DEBUG__CASE == 3) && ((vbat_sample_count & REG_DEBUG__MASK) == 0))
			{
				float_to_bytes(&vbat, &USBD_CDC_IF_tx_buffer[0]);
				USB_TX(4)
			}
			
			// Beep if VBAT too low
			if ((REG_VBAT < REG_VBAT_MIN) && (REG_VBAT > VBAT_THRESHOLD))
				flag_beep_vbat = 1;
			else
				flag_beep_vbat = 0;
		}
		
		/* Host command from USB -------------------------------------------------------*/
		
		REG_ERROR = ((uint32_t)radio_error_count << 16) | (uint32_t)mpu_error_count;
		REG_TIME = ((uint32_t)time_process << 16) | (uint32_t)time_mpu;
		
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
					FLASH->CR |= FLASH_CR_SER | (11 << FLASH_CR_SNB_Pos);
					FLASH->CR |= FLASH_CR_STRT;
					while (FLASH->SR & FLASH_SR_BSY) {}
					FLASH->CR &= ~FLASH_CR_SER;
					break;
				}
			}
		}
		
		/*------------------------------------------------------------------*/
		
		// Record processing time
		time[3] = TIM7->CNT;
		t = (int32_t)time[3] - (int32_t)time[2];
		if (t < 0)
			t += 65536;
		if ((REG_CTRL__TIME_MAXHOLD == 0) || (((uint16_t)t > time_process) && REG_CTRL__TIME_MAXHOLD))
			time_process = (uint16_t)t;
		
		#ifdef DEBUG
			GPIOB->BSRR = GPIO_BSRR_BR_13;
		#endif
		
		// Wait for interrupts
		__WFI();
	}
}
