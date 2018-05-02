#include "board.h"
#include "fc.h"
#include "radio.h"
#include "sensor.h"
#include "usb.h"

/* Private defines ------------------------------------*/

#define ADC_SCALE 0.0089f

/* Private types --------------------------------------*/

/* Global variables --------------------------------------*/

volatile uint8_t spi1_rx_buffer[16];
volatile uint8_t spi1_tx_buffer[16];
volatile uint8_t spi3_rx_buffer[7];
volatile uint8_t spi3_tx_buffer[7];
volatile uint32_t motor1_dshot[17];
volatile uint32_t motor2_dshot[17];
volatile uint32_t motor3_dshot[17];
volatile uint32_t motor4_dshot[17];

/* Private macros ---------------------------------------------------*/

#define DMA_CLEAR_ALL_FLAGS_0 (DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 |DMA_LIFCR_CDMEIF0 |DMA_LIFCR_CFEIF0)
#define DMA_CLEAR_ALL_FLAGS_1 (DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 |DMA_LIFCR_CDMEIF1 |DMA_LIFCR_CFEIF1)
#define DMA_CLEAR_ALL_FLAGS_2 (DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTEIF2 |DMA_LIFCR_CDMEIF2 |DMA_LIFCR_CFEIF2)
#define DMA_CLEAR_ALL_FLAGS_3 (DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3 |DMA_LIFCR_CDMEIF3 |DMA_LIFCR_CFEIF3)
#define DMA_CLEAR_ALL_FLAGS_4 (DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4 | DMA_HIFCR_CTEIF4 |DMA_HIFCR_CDMEIF4 |DMA_HIFCR_CFEIF4)
#define DMA_CLEAR_ALL_FLAGS_5 (DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 |DMA_HIFCR_CDMEIF5 |DMA_HIFCR_CFEIF5)
#define DMA_CLEAR_ALL_FLAGS_6 (DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6 |DMA_HIFCR_CDMEIF6 |DMA_HIFCR_CFEIF6)
#define DMA_CLEAR_ALL_FLAGS_7 (DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTEIF7 |DMA_HIFCR_CDMEIF7 |DMA_HIFCR_CFEIF7)

/* Functions ------------------------------------------------*/

void sensor_write(uint8_t addr, uint8_t data)
{
	spi1_tx_buffer[0] = addr & 0x7F;
	spi1_tx_buffer[1] = data;
	DMA2_Stream0->NDTR = 2;
	DMA2_Stream3->NDTR = 2;
	DMA2_Stream0->CR |= DMA_SxCR_EN;
	DMA2_Stream3->CR |= DMA_SxCR_EN;
	SPI1->CR1 |= SPI_CR1_SPE;
}

void sensor_read(uint8_t addr, uint8_t size)
{
	spi1_tx_buffer[0] = 0x80 | (addr & 0x7F);
	DMA2_Stream0->NDTR = size+1;
	DMA2_Stream3->NDTR = size+1;
	DMA2_Stream0->CR |= DMA_SxCR_EN;
	DMA2_Stream3->CR |= DMA_SxCR_EN;
	SPI1->CR1 |= SPI_CR1_SPE;
}

void rf_write(uint8_t addr, uint8_t * data, uint8_t size)
{
	spi3_tx_buffer[0] = 0x80 | (addr & 0x7F);
	memcpy((uint8_t *)&spi3_tx_buffer[1], data, size);
	DMA1_Stream0->NDTR = size+1;
	DMA1_Stream7->NDTR = size+1;
	DMA1_Stream0->CR |= DMA_SxCR_EN;
	DMA1_Stream7->CR |= DMA_SxCR_EN;
	SPI3->CR1 |= SPI_CR1_SPE;
}

void rf_read(uint8_t addr, uint8_t size)
{
	spi3_tx_buffer[0] = addr & 0x7F;
	DMA1_Stream0->NDTR = size+1;
	DMA1_Stream7->NDTR = size+1;
	DMA1_Stream0->CR |= DMA_SxCR_EN;
	DMA1_Stream7->CR |= DMA_SxCR_EN;
	SPI3->CR1 |= SPI_CR1_SPE;
}

void radio_error_recover()
{
	// Disable DMA UART
	DMA2->HIFCR = DMA_CLEAR_ALL_FLAGS_5;
	DMA2_Stream5->CR &= ~DMA_SxCR_EN;
	while (DMA2_Stream5->CR & DMA_SxCR_EN) {}
	USART1->CR3 &= ~USART_CR3_DMAR;
	USART1->CR1 &= ~USART_CR1_RE;
	
	// Clear status flags;
	USART1->SR;
	USART1->DR;
	
	USART1->CR1 |= USART_CR1_IDLEIE | USART_CR1_RE; // Set IDLE interrupt
		
	radio_error_count++;
}

__forceinline void sensor_error_recover()
{
	// Disable DMA SPI
	DMA2->LIFCR = DMA_CLEAR_ALL_FLAGS_0 | DMA_CLEAR_ALL_FLAGS_3;
	DMA2_Stream0->CR &= ~DMA_SxCR_EN;
	DMA2_Stream3->CR &= ~DMA_SxCR_EN;
	while ((DMA2_Stream0->CR & DMA_SxCR_EN) && (DMA2_Stream3->CR & DMA_SxCR_EN)) {}
	SPI1->CR1 &= ~SPI_CR1_SPE;
	
	SPI1->CR1 |= SPI_CR1_MSTR; // Set master bit that could be reset after a SPI mode error
	
	sensor_error_count++;
}

__forceinline void rf_error_recover()
{
	// Disable DMA SPI
	DMA1->LIFCR = DMA_CLEAR_ALL_FLAGS_0;
	DMA1->HIFCR = DMA_CLEAR_ALL_FLAGS_7;
	DMA1_Stream0->CR &= ~DMA_SxCR_EN;
	DMA1_Stream7->CR &= ~DMA_SxCR_EN;
	while ((DMA1_Stream0->CR & DMA_SxCR_EN) && (DMA1_Stream7->CR & DMA_SxCR_EN)) {};
	SPI3->CR1 &= ~SPI_CR1_SPE;
	
	SPI3->CR1 |= SPI_CR1_MSTR; // Set master bit that could be reset after a SPI error
	
	rf_error_count++;
}

void set_motors(uint32_t * motor_raw)
{
	#if (ESC == DSHOT)
		dshot_encode(&motor_raw[0], motor1_dshot);
		dshot_encode(&motor_raw[1], motor2_dshot);
		dshot_encode(&motor_raw[2], motor3_dshot);
		dshot_encode(&motor_raw[3], motor4_dshot);
		TIM2->CR1 = 0;
		TIM3->CR1 = 0;
		TIM5->CR1 = 0;
		TIM2->CNT = 0;
		TIM3->CNT = 0;
		TIM5->CNT = 0;
		DMA1_Stream1->CR &= ~DMA_SxCR_EN;
		DMA1_Stream2->CR &= ~DMA_SxCR_EN;
		DMA1_Stream3->CR &= ~DMA_SxCR_EN;
		DMA1_Stream4->CR &= ~DMA_SxCR_EN;
		DMA1->LIFCR = DMA_CLEAR_ALL_FLAGS_1 | DMA_CLEAR_ALL_FLAGS_2 | DMA_CLEAR_ALL_FLAGS_3;
		DMA1->HIFCR = DMA_CLEAR_ALL_FLAGS_4;
		DMA1_Stream1->NDTR = 17;
		DMA1_Stream2->NDTR = 17;
		DMA1_Stream3->NDTR = 17;
		DMA1_Stream4->NDTR = 17;
		DMA1_Stream1->CR |= DMA_SxCR_EN;
		DMA1_Stream2->CR |= DMA_SxCR_EN;
		DMA1_Stream3->CR |= DMA_SxCR_EN;
		DMA1_Stream4->CR |= DMA_SxCR_EN;
	#else
		TIM3->CCR4 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - motor_raw[0]; // Motor 2
		TIM5->CCR4 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - motor_raw[1]; // Motor 3
		TIM2->CCR3 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - motor_raw[2]; // Motor 4
		TIM5->CCR2 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - motor_raw[3]; // Motor 5
	#endif
	TIM2->CR1 |= TIM_CR1_CEN;
	TIM3->CR1 |= TIM_CR1_CEN;
	TIM5->CR1 |= TIM_CR1_CEN;
}

void toggle_led_sensor()
{
	GPIOB->ODR ^= GPIO_ODR_OD5;
}

void toggle_led_radio()
{
	GPIOB->ODR ^= GPIO_ODR_OD4;
}

void set_mpu_host(_Bool host)
{
	if (host) {
		DMA2_Stream0->M0AR = (uint32_t)spi1_rx_buffer;
		SPI1->CR1 &= ~SPI_CR1_BR_Msk;
		SPI1->CR1 |= 5 << SPI_CR1_BR_Pos; // SPI clock = clock APB2/64 = 48MHz/64 = 750kHz
	}
	else {
		DMA2_Stream0->M0AR = (uint32_t)&sensor_raw;
		SPI1->CR1 &= ~SPI_CR1_BR_Msk;
		SPI1->CR1 |= 1 << SPI_CR1_BR_Pos; // SPI clock = clock APB2/4 = 48MHz/4 = 12MHz
	}
}

float get_vbat()
{
	float vbat = 0;
	/*if (ADC2->ISR & ADC_ISR_EOC)
		vbat = (float)ADC2->DR * ADC_SCALE;
	ADC2->CR |= ADC_CR_ADSTART;*/
	return vbat;
}

void reset_timeout_radio()
{
	TIM6->CNT = 0;
}

void host_send(uint8_t * data, uint8_t size)
{
	USBD_CDC_SetTxBuffer(&USBD_device_handler, data, size);
	USBD_CDC_TransmitPacket(&USBD_device_handler);
}

uint16_t get_timer_process(void)
{
	return TIM7->CNT;
}

/* Interrupt routines -------------------------------------------------------------
-----------------------------------------------------------------------------------*/

/* RF Rx Done ----------------------------*/

void EXTI0_IRQHandler() 
{
	EXTI->PR = EXTI_PR_PR0; // Clear pending request
	flag_rf_rxtx_done = 1;
}

/* Sample valid from MPU --------------------*/

void EXTI4_IRQHandler() 
{
	EXTI->PR = EXTI_PR_PR4; // Clear pending request
	if ((REG_CTRL__SENSOR_HOST_CTRL == 0) && ((SPI1->SR & SPI_SR_BSY) == 0)) {
		sensor_read(59,14);
		timer_sensor[0] = TIM7->CNT; // SPI transaction time
	}
}

/* MPU SPI error -----------------------*/

void SPI1_IRQHandler() 
{
	sensor_error_recover();
}

/* End of MPU SPI receive -----------------------*/

void DMA2_Stream0_IRQHandler() 
{
	if (DMA2->LISR & DMA_LISR_TEIF0) // Check DMA transfer error
		sensor_error_recover();
	else {
		// Disable DMA SPI
		DMA2->LIFCR = DMA_CLEAR_ALL_FLAGS_0 | DMA_CLEAR_ALL_FLAGS_3;
		DMA2_Stream0->CR &= ~DMA_SxCR_EN;
		DMA2_Stream3->CR &= ~DMA_SxCR_EN;
		while ((DMA2_Stream0->CR & DMA_SxCR_EN) && (DMA2_Stream3->CR & DMA_SxCR_EN)) {}
		SPI1->CR1 &= ~SPI_CR1_SPE;
		
		if (flag_sensor_host_read) {
			flag_sensor_host_read = 0;
			host_send((uint8_t*)&spi1_rx_buffer[1],1);
		}
		else {
			TIM12->CNT = 0; // Reset timeout
			flag_sensor = 1; // Raise flag for sample ready
		}
	}
	
	// SPI transaction time
	timer_sensor[1] = TIM7->CNT;
}

/* MPU SPI DMA Transfer error ---------------------------*/

void DMA2_Stream3_IRQHandler()
{
	sensor_error_recover();
}

/* RF SPI error ---------------------------------------*/

void SPI3_IRQHandler() 
{
	rf_error_recover();
}

/* End of RF SPI receive --------------------------------*/

void DMA1_Stream0_IRQHandler() 
{
	if (DMA1->LISR & DMA_LISR_TEIF0) // Check DMA transfer error
		rf_error_recover();
	else {
		// Disable DMA SPI
		DMA1->LIFCR = DMA_CLEAR_ALL_FLAGS_0;
		DMA1->HIFCR = DMA_CLEAR_ALL_FLAGS_7;
		DMA1_Stream0->CR &= ~DMA_SxCR_EN;
		DMA1_Stream7->CR &= ~DMA_SxCR_EN;
		while ((DMA1_Stream0->CR & DMA_SxCR_EN) && (DMA1_Stream7->CR & DMA_SxCR_EN)) {};
		SPI3->CR1 &= ~SPI_CR1_SPE;
	
		if (flag_rf_host_read){
			flag_rf_host_read = 0;
			host_send((uint8_t*)&spi3_rx_buffer[1],1);
		}
		else
			flag_rf = 1;
	}
}

/* RF SPI DMA Transfer error ------------------------*/

void DMA1_Stream7_IRQHandler() 
{
	rf_error_recover();
}

/* Radio UART error ---------------------------------*/

void USART1_IRQHandler()
{
	if (USART1->SR & USART_SR_IDLE) {
		USART1->DR; // Clear status flags
		
		// Enable DMA UART
		USART1->CR1 &= ~USART_CR1_IDLEIE;
		DMA2_Stream5->NDTR = sizeof(radio_frame);
		DMA2_Stream5->CR |= DMA_SxCR_EN;
		USART1->CR3 |= USART_CR3_DMAR;
	}
	else
		radio_error_recover();
}

/* End of radio UART receive -------------------------------*/

void DMA2_Stream5_IRQHandler()
{
	// Check DMA transfer error
	if (DMA2->HISR & DMA_HISR_TEIF5)
		radio_error_recover();
	else {
		// Disable DMA UART
		DMA2->HIFCR = DMA_CLEAR_ALL_FLAGS_5;
		DMA2_Stream5->CR &= ~DMA_SxCR_EN;
		while (DMA2_Stream5->CR & DMA_SxCR_EN) {};
		USART1->CR3 &= ~USART_CR3_DMAR;
		USART1->CR1 &= ~USART_CR1_RE;
		
		flag_radio = 1; // Raise flag for radio commands ready
	
		// Enable DMA UART
		USART1->CR3 |= USART_CR3_DMAR;
		DMA2_Stream5->NDTR = sizeof(radio_frame);
		DMA2_Stream5->CR |= DMA_SxCR_EN;
		USART1->CR1 |= USART_CR1_RE;
	}
}

/* Beeper period ----------------------------------------------*/

void TIM4_IRQHandler()
{
	TIM4->SR &= ~TIM_SR_UIF;
	
	if (flag_beep_user || flag_beep_radio || flag_beep_sensor || flag_beep_host || flag_beep_vbat)
		GPIOB->ODR ^= GPIO_ODR_OD0;
	else
		GPIOB->ODR &= ~GPIO_ODR_OD0;
}

/* Radio timeout -----------------------------------*/

void TIM6_DAC_IRQHandler()
{
	TIM6->SR &= ~TIM_SR_UIF;
	flag_timeout_radio = 1;
}

/* MPU timeout ------------------------------------*/

void TIM8_BRK_TIM12_IRQHandler()
{
	TIM12->SR &= ~TIM_SR_UIF;
	flag_timeout_sensor = 1;
}

/* VBAT sampling time -------------------------*/

void TIM8_UP_TIM13_IRQHandler()
{
	TIM13->SR &= ~TIM_SR_UIF;
	flag_vbat = 1;
}

/* RF tempo ------------------------------*/

void TIM8_TRG_COM_TIM14_IRQHandler()
{
	TIM14->SR &= ~TIM_SR_UIF;
	TIM14->CNT = 0;
	TIM14->CR1 = 0;
	//RF_WRITE_1(SX1276_OP_MODE, SX1276_OP_MODE__MODE(3) | SX1276_OP_MODE__LONG_RANGE_MODE);
}

/* USB interrupt ------------------------------*/

void OTG_FS_IRQHandler(void)
{
	HAL_PCD_IRQHandler(&PCD_handler);
}

/* INIT -----------------------------------------------------------------
-----------------------------------------------------------------------*/

void board_init()
{
	/* RCC --------------------------------------------------------*/
	
	// Enable XTAL oscillator
	RCC->CR |= RCC_CR_HSEON;
	while ((RCC->CR & RCC_CR_HSERDY) == 0) {}
	
	// Set VCO at 192 MHz = XTAL(8MHz) * (N=192) / (M=8)
	// PLL system = VCO / ((P=0,default)+2) = 96 MHz , PLL USB = VCO / (Q=4) = 48 MHz
	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM_Msk | RCC_PLLCFGR_PLLN_Msk | RCC_PLLCFGR_PLLQ_Msk);
	RCC->PLLCFGR |= (8 << RCC_PLLCFGR_PLLM_Pos) | (192 << RCC_PLLCFGR_PLLN_Pos) | RCC_PLLCFGR_PLLSRC | (4 << RCC_PLLCFGR_PLLQ_Pos);
	RCC->CR |= RCC_CR_PLLON;
	while ((RCC->CR & RCC_CR_PLLRDY) == 0) {}
	
	// Select PLL as system clock
	FLASH->ACR |= (3 << FLASH_ACR_LATENCY_Pos);// | FLASH_ACR_PRFTEN; // Inrease Flash latency + Prefetch?
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while ((RCC->CFGR & RCC_CFGR_SWS_PLL) == 0) {}
	SystemCoreClock = 96000000;
	
	// Disable internal high-speed RC
	RCC->CR &= ~RCC_CR_HSION;
	
	// Set APB1 at 24 MHz (96/4) and APB2 at 48 MHz (96/2)
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;
	
	// Configure SysTick to generate interrupt every ms
	SysTick_Config(96000);
	
	/* Clock enable --------------------------------------------------*/
	
	// UART clock enable
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	// SPI clock enable
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
	// Timer clock enable
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN | RCC_APB1ENR_TIM5EN | RCC_APB1ENR_TIM6EN | RCC_APB1ENR_TIM7EN | RCC_APB1ENR_TIM12EN | RCC_APB1ENR_TIM13EN | RCC_APB1ENR_TIM14EN;
	// System configuration controller clock enable (to manage external interrupt line connection to GPIOs)
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	// DMA clock enable
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;
	// GPIO clock enable 
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;
	// ADC clock enable
	//RCC->AHBENR |= RCC_AHBENR_ADC12EN;
	// USB clock enable
	RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
	
	/* GPIO ------------------------------------------------*/
	
	// MODER: 00:IN, 01:OUT, 10:AF, 11:analog
	// PUPDR: 00:float 01:PU, 10:PD
	// OTYPER: 0:PP, 1:OD
	// OSPEEDR: 00:low 4MHz, 01:mid 25MHz, 10:high 50MHz, 11:very high 100MHz
	
	// Reset IOs
	GPIOA->MODER = 0;
	GPIOA->PUPDR = 0;
	GPIOA->OSPEEDR = 0;
	GPIOB->MODER = 0;
	GPIOB->PUPDR = 0;
	GPIOB->OSPEEDR = 0;
	GPIOC->MODER = 0;
	GPIOC->PUPDR = 0;
	GPIOC->OSPEEDR = 0;
	GPIOD->MODER = 0;
	GPIOD->PUPDR = 0;
	GPIOD->OSPEEDR = 0;
	
	// A0 : Servo 6, used as SX1276 DIO[0]
	// A1 : Servo 5, TIM5_CH2, AF2, DMA1 Stream 4
	// A2 : Servo 4, TIM2_CH3, AF1, DMA1 Stream 1
	// A3 : Servo 3, TIM5_CH4, AF2, DMA1 Stream 3
	GPIOA->MODER |= GPIO_MODER_MODER1_1 | GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR1_1 | GPIO_OSPEEDER_OSPEEDR2_1 | GPIO_OSPEEDER_OSPEEDR3_1;
	GPIOA->AFR[0] |= (2 << GPIO_AFRL_AFSEL1_Pos) | (1 << GPIO_AFRL_AFSEL2_Pos) | (2 << GPIO_AFRL_AFSEL3_Pos);
	// A4 : SPI1 CS, AF5, need open-drain (external pull-up)
	// A5 : SPI1 CLK, AF5, need pull-up (CPOL = 1)
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
	// A13: SWDIO, AF0
	// A14: SWCLK, AF0
	// A15: SPI3 CS, AF6, need open-drain (external pull-up)
	GPIOA->MODER |= GPIO_MODER_MODER15_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR15_1;
	GPIOA->OTYPER |= GPIO_OTYPER_OT_15;
	GPIOA->AFR[1] |= 6 << GPIO_AFRH_AFSEL15_Pos;
	// B0 : Servo 1, used as beeper
	GPIOB->MODER |= GPIO_MODER_MODER0_0;
	// B1 : Servo 2, TIM3_CH4, AF2, DMA1 Stream 2
	GPIOB->MODER |= GPIO_MODER_MODER1_1;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR1_1;
	GPIOB->AFR[0] |= 2 << GPIO_AFRL_AFSEL1_Pos;
	// B4 : Red LED, need open-drain (external pull-up)
	GPIOB->MODER |= GPIO_MODER_MODER4_0;
	GPIOB->OTYPER |= GPIO_OTYPER_OT_4;
	GPIOB->BSRR = GPIO_BSRR_BS_4;
	// B5 : Blue LED, need open-drain (external pull-up)
	GPIOB->MODER |= GPIO_MODER_MODER5_0;
	GPIOB->OTYPER |= GPIO_OTYPER_OT_5;
	GPIOB->BSRR = GPIO_BSRR_BS_5;
	// B8 : I2C1 SCL, AF4
	// B9 : I2C1 SDA, AF4, Rx: DMA1 Stream 5 Tx: DMA1 Stream 6
	// B12: Input 3
	// B13: Input 4
	// B14: Input 5
	// B15: Input 6
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
	// C10: SPI3 CLK, AF6, need pull-down (CPOL = 0)
	// C11: SPI3 MISO, AF6, DMA1 Stream 7
	// C12: SPI3 MOSI, AF6, DMA1 Stream 0
	GPIOC->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1 | GPIO_MODER_MODER12_1;
	GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10_1 | GPIO_OSPEEDER_OSPEEDR12_1;
	GPIOC->PUPDR |= GPIO_PUPDR_PUPDR10_1;
	GPIOC->AFR[1] |= (6 << GPIO_AFRH_AFSEL10_Pos) | (6 << GPIO_AFRH_AFSEL11_Pos) | (6 << GPIO_AFRH_AFSEL12_Pos);
	// D2 : RF reset, need open-drain (external pull-up)
	//GPIOD->MODER |= GPIO_MODER_MODER2_0;
	GPIOD->OTYPER |= GPIO_OTYPER_OT_2;
	
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
	DMA2_Stream5->M0AR = (uint32_t)&radio_frame;
	DMA2_Stream5->PAR = (uint32_t)&(USART1->DR);
	
	// SPI3 Rx
	DMA1_Stream0->CR = (0 << DMA_SxCR_CHSEL_Pos) | (3 << DMA_SxCR_PL_Pos) | DMA_SxCR_MINC | DMA_SxCR_TCIE | DMA_SxCR_TEIE;
	DMA1_Stream0->M0AR = (uint32_t)spi3_rx_buffer;
	DMA1_Stream0->PAR = (uint32_t)&(SPI3->DR);
	
	// SPI3 Tx
	DMA1_Stream7->CR = (0 << DMA_SxCR_CHSEL_Pos) | (1 << DMA_SxCR_PL_Pos) | DMA_SxCR_MINC | (1 << DMA_SxCR_DIR_Pos) | DMA_SxCR_TEIE;
	DMA1_Stream7->M0AR = (uint32_t)spi3_tx_buffer;
	DMA1_Stream7->PAR = (uint32_t)&(SPI3->DR);
	
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
	
#if (ESC == DSHOT)
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
	TIM2->PSC = 3-1;
	TIM2->ARR = SERVO_MAX*2 + 1;
	TIM2->CCER = TIM_CCER_CC3E;
	TIM2->CCMR2 = 7 << TIM_CCMR2_OC3M_Pos;
	
	TIM3->CR1 = TIM_CR1_OPM;
	TIM3->PSC = 3-1;
	TIM3->ARR = SERVO_MAX*2 + 1;
	TIM3->CCER = TIM_CCER_CC4E;
	TIM3->CCMR2 = 7 << TIM_CCMR2_OC4M_Pos;
	
	TIM5->CR1 = TIM_CR1_OPM;
	TIM5->PSC = 3-1;
	TIM5->ARR = SERVO_MAX*2 + 1;
	TIM5->CCER = TIM_CCER_CC2E | TIM_CCER_CC4E;
	TIM5->CCMR1 = 7 << TIM_CCMR1_OC2M_Pos;
	TIM5->CCMR2 = 7 << TIM_CCMR2_OC4M_Pos;
#endif

	// Beeper
	TIM4->PSC = 48000-1; // 1ms
	TIM4->ARR = BEEPER_PERIOD;
	TIM4->DIER = TIM_DIER_UIE;
	TIM4->CR1 = TIM_CR1_CEN;
	
	// Receiver timeout
	TIM6->PSC = 48000-1; // 1ms
	TIM6->ARR = TIMEOUT_RADIO;
	TIM6->DIER = TIM_DIER_UIE;
	//TIM6->CR1 = TIM_CR1_CEN; // To be enabled after radio init
	
	// Processing time
	TIM7->PSC = 48-1; // 1us
	TIM7->ARR = 65535;
	TIM7->CR1 = TIM_CR1_CEN;
	
	// Sensor timeout
	TIM12->PSC = 48-1; // 1us
	TIM12->ARR = TIMEOUT_SENSOR;
	TIM12->DIER = TIM_DIER_UIE;
	//TIM12->CR1 = TIM_CR1_CEN; // To be enabled after sensor init
	
	// VBAT
	TIM13->PSC = 48000-1; // 1ms
	TIM13->ARR = VBAT_PERIOD;
	TIM13->DIER = TIM_DIER_UIE;
	//TIM13->CR1 = TIM_CR1_CEN;
	
	// RF tempo
	TIM14->PSC = 48000-1; // 1ms
	TIM14->ARR = 200;
	TIM14->DIER = TIM_DIER_UIE;
	
	/* UART ---------------------------------------------------*/

#if (RADIO_TYPE == SBUS)
	GPIOC->BSRR = GPIO_BSRR_BS_0; // Invert Rx
	USART1->BRR = 480; // 48MHz/100000bps
	USART1->CR1 = USART_CR1_UE | USART_CR1_M | USART_CR1_PCE;
	USART1->CR2 = (2 << USART_CR2_STOP_Pos);
#else
	GPIOC->BSRR = GPIO_BSRR_BR_0; // Do not invert Rx
	USART1->BRR = 417; // 48MHz/115200bps
	USART1->CR1 = USART_CR1_UE;
#endif
	USART1->CR3 = USART_CR3_EIE;
	
	/* SPI ----------------------------------------------------*/
	
	SPI1->CR1 = SPI_CR1_MSTR | (5 << SPI_CR1_BR_Pos) | SPI_CR1_CPOL | SPI_CR1_CPHA; // SPI clock = clock APB2/64 = 48MHz/64 = 750 kHz
	SPI1->CR2 = SPI_CR2_SSOE | SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN | SPI_CR2_ERRIE;

	SPI3->CR1 = SPI_CR1_MSTR | (1 << SPI_CR1_BR_Pos); // SPI clock = clock APB1/4 = 24MHz/4 = 6 MHz
	SPI3->CR2 = SPI_CR2_SSOE | SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN | SPI_CR2_ERRIE;

	/* ADC -----------------------------------------------------*/
	/*
	ADC1->CR2 |= ADC_CR2_ADON;
	wait_ms(1);
	
	ADC1->SMPR2 = 4 << ADC_SMPR2_SMP2_Pos;
	ADC1->SQR3 = 2 << ADC_SQR3_SQ1_Pos;
	*/
	/* Interrupts ---------------------------------------------------*/
	
	SYSCFG->EXTICR[1] = SYSCFG_EXTICR2_EXTI4_PC; // Sensor interrrupt
	SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI0_PA; // RF interrupt
	EXTI->RTSR = EXTI_RTSR_TR4 | EXTI_RTSR_TR0; // Rising edge
	//EXTI->IMR = EXTI_IMR_MR4 | EXTI_IMR_MR0; // To be enabled after sensor init
	
	NVIC_EnableIRQ(EXTI0_IRQn);
	NVIC_EnableIRQ(EXTI4_IRQn);
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_EnableIRQ(SPI1_IRQn);
	NVIC_EnableIRQ(SPI3_IRQn);
	NVIC_EnableIRQ(DMA1_Stream0_IRQn);
	NVIC_EnableIRQ(DMA1_Stream7_IRQn);
	NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	NVIC_EnableIRQ(DMA2_Stream3_IRQn);
	NVIC_EnableIRQ(DMA2_Stream5_IRQn);
	NVIC_EnableIRQ(TIM4_IRQn);
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
	NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
	NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
	NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn);
	NVIC_EnableIRQ(OTG_FS_IRQn);
	
	NVIC_SetPriority(EXTI0_IRQn,0);
	NVIC_SetPriority(EXTI4_IRQn,0);
	NVIC_SetPriority(USART1_IRQn,0);
	NVIC_SetPriority(SPI1_IRQn,0);
	NVIC_SetPriority(SPI3_IRQn,0);
	NVIC_SetPriority(DMA1_Stream0_IRQn,0);
	NVIC_SetPriority(DMA1_Stream7_IRQn,0);
	NVIC_SetPriority(DMA2_Stream0_IRQn,0);
	NVIC_SetPriority(DMA2_Stream3_IRQn,0);
	NVIC_SetPriority(DMA2_Stream5_IRQn,0);
	NVIC_SetPriority(TIM4_IRQn,0);
	NVIC_SetPriority(TIM6_DAC_IRQn,0);
	NVIC_SetPriority(TIM8_BRK_TIM12_IRQn,0);
	NVIC_SetPriority(TIM8_UP_TIM13_IRQn,0);
	NVIC_SetPriority(TIM8_TRG_COM_TIM14_IRQn,0);
	NVIC_SetPriority(OTG_FS_IRQn,16);
	
	/* Host init -------------------------------------------*/
	
	usb_init();
	
	/* Sensor init ----------------------------------------------------*/
	
	wait_ms(1000);
	mpu_spi_init();
	set_mpu_host(0);
	EXTI->IMR = EXTI_IMR_MR4; // Enable interrupt
	TIM12->CR1 = TIM_CR1_CEN; // Enable timeout
	
	/* RF init -----------------------------------------------------*/
	/*
	// Reset
	GPIOD->BSRR = GPIO_BSRR_BR_2;
	wait_ms(1);
	GPIOD->BSRR = GPIO_BSRR_BS_2;
	wait_ms(1);
	sx1276_init();
	EXTI->IMR |= EXTI_IMR_MR0; // Enable external interrupts now
	*/
	
	/* Radio init --------------------------------*/
	
	USART1->CR1 |= USART_CR1_IDLEIE | USART_CR1_RE; // Set IDLE interrupt
	TIM6->CR1 = TIM_CR1_CEN; // Enable timeout
}
