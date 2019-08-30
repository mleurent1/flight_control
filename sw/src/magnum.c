#include "board.h"
#include "stm32f3xx.h" // CMSIS
#include "fc.h" // flags
#include "sensor.h" // mpu_spi_init()
#include "utils.h" // wait_ms()
#include "usb.h" // usb_init();

/* Private defines ------------------------------------*/

#define ADC_SCALE 0.0089f

/* Private macros ------------------------------------------*/

/* Private types --------------------------------------*/

/* Global variables --------------------------------------*/

volatile uint8_t spi1_rx_buffer[16];
volatile uint8_t spi1_tx_buffer[16];
#if (ESC == DSHOT)
	volatile uint32_t dshot12[17*2];
	volatile uint32_t dshot34[17*2];
#endif
volatile _Bool sensor_busy;
volatile int32_t time_sensor_start;

/* Functions ------------------------------------------------*/

inline __attribute__((always_inline)) void sensor_spi_dma_enable(uint8_t size)
{
	DMA1_Channel2->CNDTR = size + 1;
	DMA1_Channel3->CNDTR = size + 1;
	DMA1_Channel2->CCR |= DMA_CCR_EN;
	DMA1_Channel3->CCR |= DMA_CCR_EN;
	SPI1->CR1 |= SPI_CR1_SPE;
}

inline __attribute__((always_inline)) void sensor_spi_dma_disable()
{
	DMA1->IFCR = DMA_IFCR_CGIF2 | DMA_IFCR_CGIF3; // Clear all DMA flags
	DMA1_Channel2->CCR &= ~DMA_CCR_EN;
	DMA1_Channel3->CCR &= ~DMA_CCR_EN;
	SPI1->CR1 &= ~SPI_CR1_SPE;
}

inline __attribute__((always_inline)) void radio_uart_dma_enable(uint8_t size)
{
	DMA1_Channel6->CNDTR = size;
	DMA1_Channel6->CCR |= DMA_CCR_EN;
	USART2->CR1 |= USART_CR1_RE;
}

inline __attribute__((always_inline)) void radio_uart_dma_disable()
{
	DMA1->IFCR = DMA_IFCR_CGIF6; // Clear all DMA flags
	DMA1_Channel6->CCR &= ~DMA_CCR_EN;
	USART2->CR1 &= ~USART_CR1_RE;
}

void sensor_write(uint8_t addr, uint8_t data)
{
	spi1_tx_buffer[0] = addr & 0x7F;
	spi1_tx_buffer[1] = data;
	sensor_spi_dma_enable(1);
	sensor_busy = 1;
	// Wait for end of transaction
	while (sensor_busy)
		__WFI();
}

void sensor_read(uint8_t addr, uint8_t size)
{
	spi1_tx_buffer[0] = 0x80 | (addr & 0x7F);
	sensor_spi_dma_enable(size);
	sensor_busy = 1;
}

void radio_sync()
{
	radio_uart_dma_disable();
	USART2->CR1 |= USART_CR1_RE | USART_CR1_IDLEIE; // Enable UART with IDLE line detection
}

void set_motors(uint16_t * motor_raw, _Bool * motor_telemetry)
{
#if (ESC == DSHOT)
	int i;
	uint8_t motor1_dshot[16];
	uint8_t motor2_dshot[16];
	uint8_t motor3_dshot[16];
	uint8_t motor4_dshot[16];

	if ((DMA2_Channel1->CNDTR == 0) && (DMA1_Channel5->CNDTR == 0)) {
		TIM8->DIER = 0;
		TIM8->CR1 = 0;
		TIM8->CNT = 60;
		DMA2->IFCR = DMA_IFCR_CGIF1; // Clear all DMA flags
		DMA2_Channel1->CCR &= ~DMA_CCR_EN; // Disable DMA

		TIM15->DIER = 0;
		TIM15->CR1 = 0;
		TIM15->CNT = 60;
		DMA1->IFCR = DMA_IFCR_CGIF5; // Clear all DMA flags
		DMA1_Channel5->CCR &= ~DMA_CCR_EN; // Disable DMA

		dshot_encode(&motor_raw[0], motor2_dshot, motor_telemetry[0]);
		dshot_encode(&motor_raw[1], motor1_dshot, motor_telemetry[1]);
		dshot_encode(&motor_raw[2], motor4_dshot, motor_telemetry[2]);
		dshot_encode(&motor_raw[3], motor3_dshot, motor_telemetry[3]);
		for (i=0; i<16; i++) {
			dshot12[i*2+0] = (uint32_t)motor1_dshot[i];
			dshot12[i*2+1] = (uint32_t)motor2_dshot[i];
			dshot34[i*2+0] = (uint32_t)motor3_dshot[i];
			dshot34[i*2+1] = (uint32_t)motor4_dshot[i];
		}

		DMA2_Channel1->CNDTR = 17*2;
		DMA2_Channel1->CCR |= DMA_CCR_EN;
		TIM8->DIER = TIM_DIER_UDE;

		DMA1_Channel5->CNDTR = 17*2;
		DMA1_Channel5->CCR |= DMA_CCR_EN;
		TIM15->DIER = TIM_DIER_UDE;

		TIM8->CR1 = TIM_CR1_CEN;
		TIM15->CR1 = TIM_CR1_CEN;
	}
#else
	TIM8->CCR2 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - (uint32_t)motor_raw[1];
	TIM8->CCR3 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - (uint32_t)motor_raw[0];
	TIM15->CCR1 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - (uint32_t)motor_raw[3];
	TIM15->CCR2 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - (uint32_t)motor_raw[2];
	TIM8->CR1 |= TIM_CR1_CEN;
	TIM15->CR1 |= TIM_CR1_CEN;
#endif
}

void toggle_led()
{
	GPIOB->ODR ^= GPIO_ODR_3;
}

void toggle_beeper(_Bool en)
{
	if (en)
		GPIOC->ODR ^= GPIO_ODR_15;
	else
		GPIOC->BSRR = GPIO_BSRR_BR_15;
}

void set_mpu_host(_Bool host)
{
	// Wait for end of current transaction
	while (sensor_busy)
		__WFI();

	if (host) {
		DMA1_Channel2->CMAR = (uint32_t)spi1_rx_buffer;
		SPI1->CR1 &= ~SPI_CR1_BR_Msk;
		SPI1->CR1 |= 5 << SPI_CR1_BR_Pos; // 750kHz
	}
	else {
		DMA1_Channel2->CMAR = (uint32_t)&sensor_raw;
		SPI1->CR1 &= ~SPI_CR1_BR_Msk;
		SPI1->CR1 |= 1 << SPI_CR1_BR_Pos; // 12MHz
	}
}

void host_send(uint8_t * data, uint8_t size)
{
	USBD_CDC_SetTxBuffer(&USBD_device_handler, data, size);
	USBD_CDC_TransmitPacket(&USBD_device_handler);
}

int32_t get_timer_process(void)
{
	return (int32_t)TIM7->CNT;
}

/* --------------------------------------------------------------------------------
 Interrupt routines
--------------------------------------------------------------------------------- */

/* Sensor ready IRQ ---------------------------*/

void EXTI15_10_IRQHandler()
{
	EXTI->PR = EXTI_PR_PIF13; // Clear pending request
	if ((REG_CTRL__SENSOR_HOST_CTRL == 0) && (!sensor_busy)) {
		sensor_read(59,14);
		time_sensor_start = (int32_t)TIM7->CNT; // record sensor transaction start time
	}
}

/* Sensor SPI IRQ ------------------------------*/

void SPI1_IRQHandler()
{
	if (SPI1->SR & (SPI_SR_MODF | SPI_SR_OVR)) {
		sensor_spi_dma_disable();
		SPI1->SR; // Read SR to clear flags
		sensor_error_count++;
	}
}

/* DMA IRQ of sensor Rx SPI ----------------------*/

void DMA1_Channel2_IRQHandler()
{
	sensor_spi_dma_disable();
	sensor_busy = 0;

	if (REG_CTRL__SENSOR_HOST_CTRL == 1) // Send SPI read data to host
		host_send((uint8_t*)&spi1_rx_buffer[1], 1);
	else {
		flag_sensor = 1; // Raise flag for sample ready
		time_sensor = (int32_t)TIM7->CNT - time_sensor_start; // Record sensor transaction time
	}
}

/* Radio UART IRQ ------------------------------*/

void USART2_IRQHandler()
{
	if (USART2->ISR & USART_ISR_IDLE) {
		USART2->ICR = USART_ICR_IDLECF; // Clear status flags
		USART2->CR1 &= ~(USART_CR1_IDLEIE | USART_CR1_RE); // Disable idle line detection
		radio_uart_dma_enable(sizeof(radio_frame));
	}
	else {
		USART2->ICR = USART_ICR_ORECF | USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NCF; // Clear all flags
		radio_error_count++;
		radio_sync();
	}
}

/* DMA IRQ of radio Rx UART-----------------------*/

void DMA1_Channel6_IRQHandler()
{
	radio_uart_dma_disable();
	radio_uart_dma_enable(sizeof(radio_frame));
	flag_radio = 1; // Raise flag for radio commands ready
}

/* Status period --------------------------------------*/

void TIM6_DAC_IRQHandler()
{
	TIM6->SR &= ~TIM_SR_UIF;
	flag_status = 1;
}

/* VBAT sampling time -----------------------------*/

#ifdef VBAT
	void TIM1_UP_TIM16_IRQHandler()
	{
		TIM16->SR &= ~TIM_SR_UIF;

		if (ADC1->ISR & ADC_ISR_EOC)
			vbat = (float)ADC1->DR * ADC_SCALE;
		ADC1->CR |= ADC_CR_ADSTART;

		flag_vbat = 1;
	}
#endif

/* USB interrupt -----------------------------*/

void USB_LP_CAN_RX0_IRQHandler()
{
	HAL_PCD_IRQHandler(&PCD_handler);
}

/* ---------------------------------------------------------------------
 board init
--------------------------------------------------------------------- */

void board_init()
{
	/* RCC --------------------------------------------------------*/

	// Enable XTAL oscillator
	RCC->CR |= RCC_CR_HSEON;
	while ((RCC->CR & RCC_CR_HSERDY) == 0) {}

	// Set PLL at 48MHz = 6 * XTAL
	RCC->CFGR |= RCC_CFGR_PLLMUL6 | RCC_CFGR_PLLSRC_HSE_PREDIV;
	RCC->CR |= RCC_CR_PLLON;
	while ((RCC->CR & RCC_CR_PLLRDY) == 0) {}

	// Select PLL as system clock
	FLASH->ACR |= 1 << FLASH_ACR_LATENCY_Pos; // Inrease Flash latency!
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while ((RCC->CFGR & RCC_CFGR_SWS_PLL) == 0) {}
	SystemCoreClock = 48000000;

	// Set APB1 at 24MHz and USB at 48MHz
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_USBPRE;

	// Select system clock as UART2/3/4/5 clock and as I2C1/2 clock
	RCC->CFGR3 |= RCC_CFGR3_USART2SW_SYSCLK | RCC_CFGR3_USART3SW_SYSCLK | RCC_CFGR3_UART4SW_SYSCLK | RCC_CFGR3_UART5SW_SYSCLK | RCC_CFGR3_I2C1SW_SYSCLK | RCC_CFGR3_I2C2SW_SYSCLK;;

	// Configure SysTick to generate interrupt every ms
	SysTick_Config(48000);

	// DMA clock enable
	RCC->AHBENR |= RCC_AHBENR_DMA1EN | RCC_AHBENR_DMA2EN;

	// System configuration controller clock enable (to manage external interrupt line connection to GPIOs)
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	/* GPIO ------------------------------------------------*/

	// MODER: 00:IN, 01:OUT, 10:AF, 11:analog
	// PUPDR: 00:float 01:PU, 10:PD
	// OTYPER: 0:PP, 1:OD
	// OSPEEDR: x0:low 01:mid 11:high

	// GPIO clock enable
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;

	// Reset non-zero registers
	GPIOA->MODER = 0;
	GPIOA->PUPDR = 0;
	GPIOA->OSPEEDR = 0;
	GPIOB->MODER = 0;
	GPIOB->PUPDR = 0;
	GPIOB->OSPEEDR = 0;

	// A8 : TIM1 CH1 AF6, NOT USED
	// A13: SWDIO, AF0
	// A14: SWCLK, AF0

	/* USB ----------------------------------------*/

	// A11: USB_DM, AF14
	// A12: USB_DP, AF14
	GPIOA->MODER |= GPIO_MODER_MODER11_1 | GPIO_MODER_MODER12_1;
	GPIOA->AFR[1] |= (14 << GPIO_AFRH_AFRH3_Pos) | (14 << GPIO_AFRH_AFRH4_Pos);
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR11 | GPIO_OSPEEDER_OSPEEDR12;

	// Clock and interrupt enable
	RCC->APB1ENR |= RCC_APB1ENR_USBEN;
	NVIC_EnableIRQ(USB_LP_CAN_RX0_IRQn);
	NVIC_SetPriority(USB_LP_CAN_RX0_IRQn,16);

	// Init
	usb_init();

	/* Gyro/accel sensor ----------------------------------------------------*/

	// A4 : SPI1 CS, AF5, need pull-up
	// A5 : SPI1 CLK, AF5, need pull-up
	// A6 : SPI1 MISO, AF5
	// A7 : SPI1 MOSI, AF5
	GPIOA->MODER |= GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4 | GPIO_OSPEEDER_OSPEEDR5 | GPIO_OSPEEDER_OSPEEDR7;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR4_0 | GPIO_PUPDR_PUPDR5_0;
	GPIOA->AFR[0] |= (5 << GPIO_AFRL_AFRL4_Pos) | (5 << GPIO_AFRL_AFRL5_Pos) | (5 << GPIO_AFRL_AFRL6_Pos) | (5 << GPIO_AFRL_AFRL7_Pos);

	// SPI config
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	SPI1->CR1 = SPI_CR1_MSTR | (5 << SPI_CR1_BR_Pos) | SPI_CR1_CPOL | SPI_CR1_CPHA; // SPI clock = clock APB2/64 = 24MHz/64 = 750 kHz
	SPI1->CR2 = SPI_CR2_SSOE | SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN | SPI_CR2_FRXTH | SPI_CR2_ERRIE;
	NVIC_EnableIRQ(SPI1_IRQn);
	NVIC_SetPriority(SPI1_IRQn,0);

	// Sensor interrrupt, PC13
	SYSCFG->EXTICR[3] = SYSCFG_EXTICR4_EXTI13_PC;
	EXTI->RTSR |= EXTI_RTSR_TR13;
	NVIC_EnableIRQ(EXTI15_10_IRQn);
	NVIC_SetPriority(EXTI15_10_IRQn,0);

	// DMA SPI1 Rx
	DMA1_Channel2->CCR = DMA_CCR_MINC | DMA_CCR_PL_0 | DMA_CCR_TCIE;
	DMA1_Channel2->CMAR = (uint32_t)spi1_rx_buffer;
	DMA1_Channel2->CPAR = (uint32_t)&(SPI1->DR);
	NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	NVIC_SetPriority(DMA1_Channel2_IRQn,0);

	// DMA SPI1 Tx
	DMA1_Channel3->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_PL_1;
	DMA1_Channel3->CMAR = (uint32_t)spi1_tx_buffer;
	DMA1_Channel3->CPAR = (uint32_t)&(SPI1->DR);

	// Init
	mpu_spi_init();
	set_mpu_host(0);
	EXTI->IMR = EXTI_IMR_MR13; // enable external interrupt now

	/* Radio Rx UART ---------------------------------------------------*/

	// B4 : UART2 Rx, AF7, pull-up for IDLE
	GPIOB->MODER |= GPIO_MODER_MODER4_1;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR4_0;
	GPIOB->AFR[0] |= 7 << GPIO_AFRL_AFRL4_Pos;

	// UART config
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	USART2->BRR = 417; // 48MHz/115200bps
	USART2->CR1 = USART_CR1_UE;
	USART2->CR3 = USART_CR3_DMAR | USART_CR3_EIE;
	NVIC_EnableIRQ(USART2_IRQn);
	NVIC_SetPriority(USART2_IRQn,0);

	// DMA UART2 Rx
	DMA1_Channel6->CCR = DMA_CCR_TCIE | DMA_CCR_MINC;
	DMA1_Channel6->CMAR = (uint32_t)&radio_frame;
	DMA1_Channel6->CPAR = (uint32_t)&(USART2->RDR);
	NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	NVIC_SetPriority(DMA1_Channel6_IRQn,0);

	// Init
	radio_sync();

	/* Motors -----------------------------------------*/

	// B9 : Motor 1, to TIM8_CH3, AF10
	// B8 : Motor 2, to TIM8_CH2, AF10
	// A3 : Motor 3, to TIM15_CH2, AF9
	// A2 : Motor 4, to TIM15_CH1, AF9
	GPIOB->MODER |= GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8 | GPIO_OSPEEDER_OSPEEDR9;
	GPIOB->AFR[1] |= (10 << GPIO_AFRH_AFRH0_Pos) | (10 << GPIO_AFRH_AFRH1_Pos);
	GPIOA->MODER |= GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2 | GPIO_OSPEEDER_OSPEEDR3;
	GPIOA->AFR[0] |= (9 << GPIO_AFRL_AFRL2_Pos) | (9 << GPIO_AFRL_AFRL3_Pos);

	// TIM8 and TIM15 clock enable
	RCC->APB2ENR |= RCC_APB2ENR_TIM8EN | RCC_APB2ENR_TIM15EN;

	// output enable
	TIM8->BDTR = TIM_BDTR_MOE;
	TIM15->BDTR = TIM_BDTR_MOE;

#if (ESC == DSHOT)
	// DMA driven timer for DShot600, 48Mhz: 0:30, 1:60, T:80
	TIM8->PSC = 0; // 0:DShot600, 1:DShot300
	TIM8->ARR = 80;
	//TIM8->DIER = TIM_DIER_UDE; // Managed in motor function
	TIM8->CCER = TIM_CCER_CC2E | TIM_CCER_CC3E;
	TIM8->CCMR1 = (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
	TIM8->CCMR2 = (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
	TIM8->DCR = (1 << TIM_DCR_DBL_Pos) | (14 << TIM_DCR_DBA_Pos);

	TIM15->PSC = 0;
	TIM15->ARR = 80;
	//TIM15->DIER = TIM_DIER_UDE; // Managed in motor function
	TIM15->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
	TIM15->CCMR1 = (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE | (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
	TIM15->DCR = (1 << TIM_DCR_DBL_Pos) | (13 << TIM_DCR_DBA_Pos);

	// Pad DSHOT pulse with zeros
	dshot12[16*2+0] = 0;
	dshot12[16*2+1] = 0;
	dshot34[16*2+0] = 0;
	dshot34[16*2+1] = 0;

	// DMA TIM8_UP
	DMA2_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_PL | DMA_CCR_PSIZE_1 | DMA_CCR_MSIZE_1;
	DMA2_Channel1->CMAR = (uint32_t)dshot12;
	DMA2_Channel1->CPAR = (uint32_t)&(TIM8->DMAR);

	// DMA TIM15_UP
	DMA1_Channel5->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_PL | DMA_CCR_PSIZE_1 | DMA_CCR_MSIZE_1;
	DMA1_Channel5->CMAR = (uint32_t)dshot34;
	DMA1_Channel5->CPAR = (uint32_t)&(TIM15->DMAR);

#elif (ESC == ONESHOT)
	// One-pulse mode for OneShot125
	TIM8->CR1 = TIM_CR1_OPM;
	TIM8->PSC = 3-1;
	TIM8->ARR = SERVO_MAX*2 + 1;
	TIM8->CCER = TIM_CCER_CC2E | TIM_CCER_CC3E;
	TIM8->CCMR1 = (7 << TIM_CCMR1_OC2M_Pos);
	TIM8->CCMR2 = (7 << TIM_CCMR2_OC3M_Pos);

	TIM15->CR1 = TIM_CR1_OPM;
	TIM15->PSC = 3-1;
	TIM15->ARR = SERVO_MAX*2 + 1;
	TIM15->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
	TIM15->CCMR1 = (7 << TIM_CCMR1_OC1M_Pos) | (7 << TIM_CCMR1_OC2M_Pos);
#endif

	/* LED -----------------------*/

	// B3 : Red LED, need open-drain
	GPIOB->MODER |= GPIO_MODER_MODER3_0;
	GPIOB->OTYPER |= GPIO_OTYPER_OT_3;
	GPIOB->BSRR = GPIO_BSRR_BS_3;

	/* VBAT ADC -----------------------------------------------------*/

#ifdef VBAT

	// Clock enable
	RCC->AHBENR |= RCC_AHBENR_ADC12EN;

	// A0 : VBAT/10
	GPIOA->MODER |= GPIO_MODER_MODER0;

	// Select AHB clock as ADC clock
	ADC12_COMMON->CCR = ADC12_CCR_CKMODE_0;

	// Regulator startup
	ADC1->CR = 0;
	ADC1->CR = ADC_CR_ADVREGEN_0;
	wait_ms(1);

	// Calibration
	ADC1->CR |= ADC_CR_ADCAL;
	while (ADC1->CR & ADC_CR_ADCAL) {}

	// Enable
	ADC1->CR |= ADC_CR_ADEN;
	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) {}

	ADC1->SMPR1 = 4 << ADC_SMPR1_SMP2_Pos;
	ADC1->SQR1 = 1 << ADC_SQR1_SQ1_Pos;

	// Get first vbat value
	ADC1->CR |= ADC_CR_ADSTART;
	while ((ADC1->ISR & ADC_ISR_EOC) == 0) {}
	vbat = (float)ADC1->DR * ADC_SCALE;

	// Timer
	RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
	TIM16->PSC = 48000-1; // 1ms
	TIM16->ARR = VBAT_PERIOD;
	TIM16->DIER = TIM_DIER_UIE;
	TIM16->CR1 = TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
	NVIC_SetPriority(TIM1_UP_TIM16_IRQn,0);

#endif

	/* Beeper -----------------------------------------*/

#ifdef BEEPER

	// C15: Beeper
	GPIOC->MODER |= GPIO_MODER_MODER15_0;
	GPIOC->BSRR = GPIO_BSRR_BR_15;

#endif

	/* Status and processing time timers --------------------------------------------------------------------------*/

	// status timer
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	TIM6->PSC = 48000-1; // 1ms
	TIM6->ARR = STATUS_PERIOD;
	TIM6->DIER = TIM_DIER_UIE;
	TIM6->CR1 = TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
	NVIC_SetPriority(TIM6_DAC_IRQn,0);

	// timer to record processing time
	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
	TIM7->PSC = 48-1; // 1us
	TIM7->ARR = 65535;
	TIM7->CR1 = TIM_CR1_CEN;

}
