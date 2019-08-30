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

volatile uint8_t i2c2_rx_buffer[15];
volatile uint8_t i2c2_tx_buffer[2];
volatile uint8_t i2c2_rx_bytes_to_read;
volatile uint8_t i2c2_tx_bytes_written;
#if (ESC == DSHOT)
	volatile uint32_t dshot[17*4];
#endif
volatile _Bool sensor_busy;
volatile int32_t time_sensor_start;

/* Functions ------------------------------------------------*/

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
	i2c2_tx_buffer[0] = addr;
	i2c2_tx_buffer[1] = data;
	i2c2_rx_bytes_to_read = 0;
	i2c2_tx_bytes_written = 0;
	I2C2->CR2 &= ~(I2C_CR2_RD_WRN | I2C_CR2_NBYTES);
	I2C2->CR2 |= (2 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	sensor_busy = 1;
	// Wait for end of transaction
	while (sensor_busy)
		__WFI();
}

void sensor_read(uint8_t addr, uint8_t size)
{
	i2c2_tx_buffer[0] = addr;
	i2c2_rx_bytes_to_read = size;
	i2c2_tx_bytes_written = 0;
	I2C2->CR2 &= ~(I2C_CR2_RD_WRN | I2C_CR2_NBYTES);
	I2C2->CR2 |= (1 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
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

	if (DMA1_Channel3->CNDTR == 0) {
		TIM3->DIER = 0;
		TIM3->CR1 = 0;
		TIM3->CNT = 60;
		DMA1->IFCR = DMA_IFCR_CGIF3; // Clear all DMA flags
		DMA1_Channel3->CCR &= ~DMA_CCR_EN; // Disable DMA

		dshot_encode(&motor_raw[0], motor2_dshot, motor_telemetry[0]);
		dshot_encode(&motor_raw[1], motor1_dshot, motor_telemetry[1]);
		dshot_encode(&motor_raw[2], motor3_dshot, motor_telemetry[2]);
		dshot_encode(&motor_raw[3], motor4_dshot, motor_telemetry[3]);
		for (i=0; i<16; i++) {
			dshot[i*4+0] = (uint32_t)motor1_dshot[i];
			dshot[i*4+1] = (uint32_t)motor2_dshot[i];
			dshot[i*4+2] = (uint32_t)motor3_dshot[i];
			dshot[i*4+3] = (uint32_t)motor4_dshot[i];
		}

		DMA1_Channel3->CNDTR = 17*4;
		DMA1_Channel3->CCR |= DMA_CCR_EN;
		TIM3->DIER = TIM_DIER_UDE;
		TIM3->CR1 = TIM_CR1_CEN;
	}
#else
	TIM3->CCR1 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - (uint32_t)motor_raw[1];
	TIM3->CCR2 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - (uint32_t)motor_raw[0];
	TIM3->CCR3 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - (uint32_t)motor_raw[2];
	TIM3->CCR4 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - (uint32_t)motor_raw[3];
	TIM3->CR1 |= TIM_CR1_CEN;
#endif
}

void toggle_led()
{
	GPIOB->ODR ^= GPIO_ODR_5;
}

void toggle_beeper(_Bool en)
{
	if (en)
		GPIOA->ODR ^= GPIO_ODR_0;
	else
		GPIOA->BSRR = GPIO_BSRR_BR_0;
}

void set_mpu_host(_Bool host)
{
	// Wait for end of current transaction
	while (sensor_busy)
		__WFI();

	if (host)
		DMA1_Channel5->CMAR = (uint32_t)i2c2_rx_buffer;
	else
		DMA1_Channel5->CMAR = (uint32_t)&sensor_raw + 1;
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
	EXTI->PR = EXTI_PR_PIF15; // Clear pending request
	if ((REG_CTRL__SENSOR_HOST_CTRL == 0) && (!sensor_busy)) {
		sensor_read(59,14);
		time_sensor_start = (int32_t)TIM7->CNT; // record sensor transaction start time
	}
}

/* Sensor I2C error ------------------------*/

void I2C2_ER_IRQHandler()
{
	DMA1->IFCR = DMA_IFCR_CGIF5; // Clear all flags
	DMA1_Channel5->CCR &= ~DMA_CCR_EN; // Disable DMA Rx I2C

	// Reset I2C
	I2C2->CR1 &= ~I2C_CR1_PE;
	I2C2->CR1 |= I2C_CR1_PE;

	sensor_error_count++;
}

/* Sensor I2C R/W management ------------------------*/

void I2C2_EV_IRQHandler()
{
	if (I2C2->ISR & I2C_ISR_TXIS) { // Tx buffer empty
		I2C2->TXDR = i2c2_tx_buffer[i2c2_tx_bytes_written]; // write will clear TXE flag
		i2c2_tx_bytes_written++;
	}
	else if (I2C2->ISR & I2C_ISR_TC) { // Transfer completed
		if (i2c2_rx_bytes_to_read > 0) {
			DMA1_Channel5->CNDTR = i2c2_rx_bytes_to_read;
			DMA1_Channel5->CCR |= DMA_CCR_EN; // Enable DMA for Rx
			// Restart I2C transfer for read
			I2C2->CR2 &= ~I2C_CR2_NBYTES;
			I2C2->CR2 |= (i2c2_rx_bytes_to_read << I2C_CR2_NBYTES_Pos) | I2C_CR2_RD_WRN | I2C_CR2_START;
			i2c2_rx_bytes_to_read = 0;
		}
		else {
			// End I2C transfer
			I2C2->CR2 |= I2C_CR2_STOP; // stop will clear TC flag
		}
	}
	else if (I2C2->ISR & I2C_ISR_STOPF) { // stop condition completed
		I2C2->ICR = I2C_ICR_STOPCF; // clear stop flags
		sensor_busy = 0;
	}
}

/* DMA IRQ of sensor Rx I2C ----------------------*/

void DMA1_Channel5_IRQHandler()
{
	DMA1->IFCR = DMA_IFCR_CGIF5; // Clear all flags
	DMA1_Channel5->CCR &= ~DMA_CCR_EN; // Disable DMA

	if (REG_CTRL__SENSOR_HOST_CTRL == 1) // Send I2C read data to host
		host_send((uint8_t*)&i2c2_rx_buffer[0], 1);
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
	void TIM1_BRK_TIM15_IRQHandler()
	{
		TIM15->SR &= ~TIM_SR_UIF;

		if (ADC2->ISR & ADC_ISR_EOC)
			vbat = (float)ADC2->DR * ADC_SCALE;
		ADC2->CR |= ADC_CR_ADSTART;

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
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	// System configuration controller clock enable (to manage external interrupt line connection to GPIOs)
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	/* GPIO ------------------------------------------------*/

	// MODER: 00:IN, 01:OUT, 10:AF, 11:analog
	// PUPDR: 00:float 01:PU, 10:PD
	// OTYPER: 0:PP, 1:OD
	// OSPEEDR: x0:low 01:mid 11:high

	// GPIO clock enable
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

	// Reset non-zero registers
	GPIOA->MODER = 0;
	GPIOA->PUPDR = 0;
	GPIOA->OSPEEDR = 0;
	GPIOB->MODER = 0;
	GPIOB->PUPDR = 0;
	GPIOB->OSPEEDR = 0;

	// A1 : Motor 5, to TIM2_CH2, AF1, NOT USED
	// A2 : Motor 6, to TIM2_CH3, AF1, NOT USED
	// A3 : Motor 7, NOT USED
	// A7 : PPM
	// A8 : Motor 8, NOT USED
	// A13: SWDIO, AF0
	// A14: SWCLK, AF0

	// B2 : RSSI
	// B3 : UART2 Tx, AF7, NOT USED
	// B6 : UART1 Tx, NOT USED
	// B7 : UART1 Rx, NOT USED
	// B10: UART3 Tx, AF7, NOT USED
	// B11: UART3 Rx, AF7, NOT USED

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

	// A9 : I2C2 SCL, AF4
	// A10: I2C2 SDA, AF4
	GPIOA->MODER |= GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9 | GPIO_OSPEEDER_OSPEEDR10;
	GPIOA->OTYPER |= GPIO_OTYPER_OT_9 | GPIO_OTYPER_OT_10;
	GPIOA->AFR[1] |= (4 << GPIO_AFRH_AFRH1_Pos) | (4 << GPIO_AFRH_AFRH2_Pos);

	// I2C config
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	I2C2->CR1 = I2C_CR1_TCIE | I2C_CR1_TXIE | I2C_CR1_RXDMAEN | I2C_CR1_STOPIE | I2C_CR1_ERRIE;
	I2C2->CR2 = 104 << (1+I2C_CR2_SADD_Pos);
	I2C2->TIMINGR = (5 << I2C_TIMINGR_PRESC_Pos) | (9 << I2C_TIMINGR_SCLL_Pos) | (3 << I2C_TIMINGR_SCLH_Pos) | (3 << I2C_TIMINGR_SDADEL_Pos) | (3 << I2C_TIMINGR_SCLDEL_Pos); // 400kHz (Fast-mode)
	I2C2->CR1 |= I2C_CR1_PE;
	NVIC_EnableIRQ(I2C2_ER_IRQn);
	NVIC_EnableIRQ(I2C2_EV_IRQn);
	NVIC_SetPriority(I2C2_ER_IRQn,0);
	NVIC_SetPriority(I2C2_EV_IRQn,0);

	// Sensor interrrupt, PA15
	SYSCFG->EXTICR[3] = SYSCFG_EXTICR4_EXTI15_PA;
	EXTI->RTSR |= EXTI_RTSR_TR15;
	NVIC_EnableIRQ(EXTI15_10_IRQn);
	NVIC_SetPriority(EXTI15_10_IRQn,0);

	// DMA I2C2 Rx
	DMA1_Channel5->CCR = DMA_CCR_MINC | DMA_CCR_PL_0 | DMA_CCR_TCIE | DMA_CCR_TEIE;
	DMA1_Channel5->CMAR = (uint32_t)i2c2_rx_buffer;
	DMA1_Channel5->CPAR = (uint32_t)&(I2C2->RXDR);
	NVIC_EnableIRQ(DMA1_Channel5_IRQn);
	NVIC_SetPriority(DMA1_Channel5_IRQn,0);

	// Init
	mpu_i2c_init();
	set_mpu_host(0);
	EXTI->IMR = EXTI_IMR_MR15; // enable external interrupt now

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

	// A4 : Motor 1, to TIM3_CH2, AF2
	// A6 : Motor 2, to TIM3_CH1, AF2
	// B0 : Motor 3, to TIM3_CH3, AF2
	// B1 : Motor 4, to TIM3_CH4, AF2
	GPIOA->MODER |= GPIO_MODER_MODER4_1 | GPIO_MODER_MODER6_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4 | GPIO_OSPEEDER_OSPEEDR6;
	GPIOA->AFR[0] |= (2 << GPIO_AFRL_AFRL4_Pos) | (2 << GPIO_AFRL_AFRL6_Pos);
	GPIOB->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0 | GPIO_OSPEEDER_OSPEEDR1;
	GPIOB->AFR[0] |= (2 << GPIO_AFRL_AFRL0_Pos) | (2 << GPIO_AFRL_AFRL1_Pos);

	// TIM3 clock enable
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

#if (ESC == DSHOT)
	// DMA driven timer for DShot600, 48Mhz: 0:30, 1:60, T:80
	TIM3->PSC = 0; // 0:DShot600, 1:DShot300
	TIM3->ARR = 80;
	//TIM3->DIER = TIM_DIER_UDE; // Managed in motor function
	TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
	TIM3->CCMR1 = (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE | (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
	TIM3->CCMR2 = (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE | (6 << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
	TIM3->DCR = (3 << TIM_DCR_DBL_Pos) | (13 << TIM_DCR_DBA_Pos);

	// Pad DSHOT pulse with zeros
	dshot[16*4+0] = 0;
	dshot[16*4+1] = 0;
	dshot[16*4+2] = 0;
	dshot[16*4+3] = 0;

	// DMA TIM3_UP
	DMA1_Channel3->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_PL | DMA_CCR_PSIZE_1 | DMA_CCR_MSIZE_1;
	DMA1_Channel3->CMAR = (uint32_t)dshot;
	DMA1_Channel3->CPAR = (uint32_t)&(TIM3->DMAR);
#elif (ESC == ONESHOT)
	// One-pulse mode for OneShot125
	TIM3->CR1 = TIM_CR1_OPM;
	TIM3->PSC = 3-1;
	TIM3->ARR = SERVO_MAX*2 + 1;
	TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
	TIM3->CCMR1 = (7 << TIM_CCMR1_OC1M_Pos) | (7 << TIM_CCMR1_OC2M_Pos);
	TIM3->CCMR2 = (7 << TIM_CCMR2_OC3M_Pos) | (7 << TIM_CCMR2_OC4M_Pos);
#endif

	/* LED -----------------------*/

	// B5 : Red LED, need open-drain
	GPIOB->MODER |= GPIO_MODER_MODER5_0;
	GPIOB->OTYPER |= GPIO_OTYPER_OT_5;
	GPIOB->BSRR = GPIO_BSRR_BS_5;

	/* VBAT ADC -----------------------------------------------------*/

#ifdef VBAT

	// Clock enable
	RCC->AHBENR |= RCC_AHBENR_ADC12EN;

	// A5 : VBAT/10
	GPIOA->MODER |= GPIO_MODER_MODER5;

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

	// Get first vbat value
	ADC2->CR |= ADC_CR_ADSTART;
	while ((ADC2->ISR & ADC_ISR_EOC) == 0) {}
	vbat = (float)ADC2->DR * ADC_SCALE;

	// Timer
	RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
	TIM15->PSC = 48000-1; // 1ms
	TIM15->ARR = VBAT_PERIOD;
	TIM15->DIER = TIM_DIER_UIE;
	TIM15->CR1 = TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
	NVIC_SetPriority(TIM1_BRK_TIM15_IRQn,0);

#endif

	/* Beeper -----------------------------------------*/

#ifdef BEEPER

	// A0 : Beeper
	GPIOA->MODER |= GPIO_MODER_MODER0_0;
	GPIOA->BSRR = GPIO_BSRR_BR_0;

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
