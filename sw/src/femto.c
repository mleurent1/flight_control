#include "board.h"
#include "fc.h"
#include "radio.h"
#include "sensor.h"
#include "usb.h"

/* Private defines ------------------------------------*/

#define ADC_SCALE 0.0089f

/* Private macros ------------------------------------------*/

/* Private types --------------------------------------*/

/* Global variables --------------------------------------*/

volatile uint8_t spi1_rx_buffer[16];
volatile uint8_t spi1_tx_buffer[16];
DMA_Channel_TypeDef DMA1_Channel2_SPI;
DMA_Channel_TypeDef DMA1_Channel2_TIM;
volatile uint32_t dshot[17*4];

/* Functions ------------------------------------------------*/

void sensor_write(uint8_t addr, uint8_t data)
{
	spi1_tx_buffer[0] = addr & 0x7F;
	spi1_tx_buffer[1] = data;
	DMA1_Channel2->CNDTR = 2;
	DMA1_Channel3->CNDTR = 2;
	DMA1_Channel2->CCR |= DMA_CCR_EN;
	DMA1_Channel3->CCR |= DMA_CCR_EN;
	GPIOB->BSRR = GPIO_BSRR_BR_9;
	SPI1->CR1 |= SPI_CR1_SPE;
}

void sensor_read(uint8_t addr, uint8_t size)
{
	spi1_tx_buffer[0] = 0x80 | (addr & 0x7F);
	DMA1_Channel2->CNDTR = size+1;
	DMA1_Channel3->CNDTR = size+1;
	DMA1_Channel2->CCR |= DMA_CCR_EN;
	DMA1_Channel3->CCR |= DMA_CCR_EN;
	GPIOB->BSRR = GPIO_BSRR_BR_9;
	SPI1->CR1 |= SPI_CR1_SPE;
}

void rf_write(uint8_t addr, uint8_t * data, uint8_t size)
{
	
}

void rf_read(uint8_t addr, uint8_t size)
{
	
}

void radio_error_recover()
{
	// Disable DMA UART
	DMA1->IFCR = DMA_IFCR_CGIF6;
	DMA1_Channel6->CCR &= ~DMA_CCR_EN;
	USART2->CR3 &= ~USART_CR3_DMAR;
	USART2->CR1 &= ~USART_CR1_RE;
	
	// Clear status flags;
	USART2->ICR = USART_ICR_IDLECF | USART_ICR_ORECF | USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NCF;
	
	// Set IDLE interrupt
	USART2->CR1 |= USART_CR1_IDLEIE | USART_CR1_RE;
		
	radio_error_count++;
}

void set_motors(uint32_t * motor_raw)
{
#if (ESC == DSHOT)
	int i;
	uint32_t motor1_dshot[16];
	uint32_t motor2_dshot[16];
	uint32_t motor3_dshot[16];
	uint32_t motor4_dshot[16];
	
	dshot_encode(&motor_raw[0], motor1_dshot);
	dshot_encode(&motor_raw[1], motor2_dshot);
	dshot_encode(&motor_raw[2], motor3_dshot);
	dshot_encode(&motor_raw[3], motor4_dshot);
	for (i=0; i<16; i++) {
		dshot[i*4+0] = motor2_dshot[i];
		dshot[i*4+1] = motor1_dshot[i];
		dshot[i*4+2] = motor3_dshot[i];
		dshot[i*4+3] = motor4_dshot[i];
	}
	TIM2->CNT = 60;
	TIM2->DIER = TIM_DIER_UDE;
	memcpy(DMA1_Channel2, (const void*)&DMA1_Channel2_TIM, sizeof(DMA_Channel_TypeDef));
	DMA1_Channel2->CCR |= DMA_CCR_EN;
	TIM2->CR1 = TIM_CR1_CEN;
#else
	TIM2->CCR1 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - motor_raw[0];
	TIM2->CCR2 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - motor_raw[1];
	TIM2->CCR3 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - motor_raw[2];
	TIM2->CCR4 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - motor_raw[3];
	TIM2->CR1 |= TIM_CR1_CEN;
#endif
}

void toggle_led_sensor()
{
	GPIOB->ODR ^= GPIO_ODR_8;
}

void toggle_led_radio()
{
	GPIOB->ODR ^= GPIO_ODR_8;
}

void set_mpu_host(_Bool host)
{
	if (host) {
		#if (ESC == DSHOT)
			TIM2->CR1 = 0;
			TIM2->DIER = 0;
			DMA1_Channel2->CCR &= ~DMA_CCR_EN; // Disable DMA TIM
			DMA1->IFCR = DMA_IFCR_CGIF2; // Clear all transfer flags
			memcpy(DMA1_Channel2, (const void*)&DMA1_Channel2_SPI, sizeof(DMA_Channel_TypeDef));
		#endif
		DMA1_Channel2->CMAR = (uint32_t)spi1_rx_buffer;
		SPI1->CR1 &= ~SPI_CR1_BR_Msk;
		SPI1->CR1 |= 5 << SPI_CR1_BR_Pos; // 750kHz
	}
	else {
		SPI1->CR1 &= ~SPI_CR1_BR_Msk;
		SPI1->CR1 |= 1 << SPI_CR1_BR_Pos; // 12MHz
	}
}

float get_vbat()
{
	float vbat = 0;
	if (ADC2->ISR & ADC_ISR_EOC)
		vbat = (float)ADC2->DR * ADC_SCALE;
	ADC2->CR |= ADC_CR_ADSTART;
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

/* --------------------------------------------------------------------------------
 Interrupt routines
--------------------------------------------------------------------------------- */

/* Sensor ready IRQ ---------------------------*/

void EXTI15_10_IRQHandler() 
{
	EXTI->PR = EXTI_PR_PIF13; // Clear pending request
	if (REG_CTRL__SENSOR_HOST_CTRL == 0) {
		#if (ESC == DSHOT)
			TIM2->CR1 = 0;
			TIM2->DIER = 0;
			DMA1_Channel2->CCR &= ~DMA_CCR_EN; // Disable DMA TIM
			DMA1->IFCR = DMA_IFCR_CGIF2; // Clear all transfer flags
			memcpy(DMA1_Channel2, (const void*)&DMA1_Channel2_SPI, sizeof(DMA_Channel_TypeDef));
		#endif
		sensor_read(59,14);
		timer_sensor[0] = TIM7->CNT; // Start recording SPI transaction time
	}
}

/* Sensor SPI IRQ ------------------------------*/

void SPI1_IRQHandler() 
{
	DMA1_Channel2->CCR &= ~DMA_CCR_EN; // Disable DMA Rx SPI
	DMA1_Channel3->CCR &= ~DMA_CCR_EN; // Disable DMA Tx SPI
	SPI1->CR1 &= ~SPI_CR1_SPE; // Disable SPI
	GPIOB->BSRR = GPIO_BSRR_BS_9; // Release SPI CSN
	sensor_error_count++;
}

/* DMA IRQ of sensor Rx SPI ----------------------*/

void DMA1_Channel2_IRQHandler() 
{
	DMA1_Channel2->CCR &= ~DMA_CCR_EN; // Disable DMA Rx SPI
	DMA1_Channel3->CCR &= ~DMA_CCR_EN; // Disable DMA Tx SPI
	DMA1->IFCR = DMA_IFCR_CGIF2 | DMA_IFCR_CGIF3; // Clear all transfer flags
		
	SPI1->CR1 &= ~SPI_CR1_SPE; // Disable SPI
	GPIOB->BSRR = GPIO_BSRR_BS_9; // Release SPI CSN
	
	if (flag_sensor_host_read) { // Send SPI read data to host
		flag_sensor_host_read = 0;
		host_send((uint8_t*)&spi1_rx_buffer[1], 1);
	}
	else {
		TIM15->CNT = 0; // Reset sensor timeout
		flag_sensor = 1; // Raise flag for sample ready
	}

	timer_sensor[1] = TIM7->CNT; // Record SPI transaction time
}

/* Radio UART IRQ ------------------------------*/

void USART2_IRQHandler()
{
	if (USART2->ISR & USART_ISR_IDLE) {
		USART2->ICR = USART_ICR_IDLECF;  // Clear status flags
		
		// Enable DMA UART
		USART2->CR1 &= ~USART_CR1_IDLEIE;
		USART2->CR3 |= USART_CR3_DMAR;
		DMA1_Channel6->CNDTR = sizeof(radio_frame);
		DMA1_Channel6->CCR |= DMA_CCR_EN;
	}
	else
		radio_error_recover();
}

/* End of radio UART receive -----------------------*/

void DMA1_Channel6_IRQHandler()
{
	if (DMA1->ISR & DMA_ISR_TEIF6) // Check DMA transfer error
		
		radio_error_recover();
	else {
		// Raise flag for radio commands ready
		flag_radio = 1; 
	
		// Disable DMA UART
		DMA1->IFCR = DMA_IFCR_CGIF6;
		DMA1_Channel6->CCR &= ~DMA_CCR_EN;
		USART2->CR1 &= ~USART_CR1_RE;
		
		// Enable DMA UART
		DMA1_Channel6->CNDTR = sizeof(radio_frame);
		DMA1_Channel6->CCR |= DMA_CCR_EN;
		USART2->CR1 |= USART_CR1_RE;
	}
}

/* Beeper period --------------------------------*/

void TIM4_IRQHandler()
{
	TIM4->SR &= ~TIM_SR_UIF;
	
	if (flag_beep_user || flag_beep_radio || flag_beep_sensor || flag_beep_host || flag_beep_vbat)
		GPIOC->ODR ^= GPIO_ODR_15;
	else
		GPIOC->ODR &= ~GPIO_ODR_15;
}

/* Radio timeout --------------------------------------*/

void TIM6_DAC_IRQHandler()
{
	TIM6->SR &= ~TIM_SR_UIF;
	flag_timeout_radio = 1;
}

/* MPU timeout ----------------------------------*/

void TIM1_BRK_TIM15_IRQHandler()
{
	TIM15->SR &= ~TIM_SR_UIF;
	flag_timeout_sensor = 1;
}

/* VBAT sampling time -----------------------------*/

void TIM1_UP_TIM16_IRQHandler()
{
	TIM16->SR &= ~TIM_SR_UIF;
	flag_vbat = 1;
}

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
	
	/* Clock enable --------------------------------------------------*/
	
	// UART clock enable
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	// SPI clock enable
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	// Timer clock enable
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM4EN | RCC_APB1ENR_TIM6EN | RCC_APB1ENR_TIM7EN;
	RCC->APB2ENR |= RCC_APB2ENR_TIM15EN | RCC_APB2ENR_TIM16EN;
	// System configuration controller clock enable (to manage external interrupt line connection to GPIOs)
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	// DMA clock enable
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	// GPIO clock enable 
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;
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
	GPIOC->MODER = 0;
	GPIOC->PUPDR = 0;
	GPIOC->OSPEEDR = 0;
	
	// A0 : Motor 1, to TIM2_CH1, AF1
	// A1 : Motor 2, to TIM2_CH2, AF1
	// A2 : Motor 3, to TIM2_CH3, AF1
	// A3 : Motor 4, to TIM2_CH4, AF1
	GPIOA->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1 | GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0 | GPIO_OSPEEDER_OSPEEDR1 | GPIO_OSPEEDER_OSPEEDR2 | GPIO_OSPEEDER_OSPEEDR3;
	GPIOA->AFR[0] |= (1 << GPIO_AFRL_AFRL0_Pos) | (1 << GPIO_AFRL_AFRL1_Pos) | (1 << GPIO_AFRL_AFRL2_Pos) | (1 << GPIO_AFRL_AFRL3_Pos);
	// A4 : Current
	// A5 : VBAT
	GPIOA->MODER |= GPIO_MODER_MODER5;
	// A11: USB_DM, AF14
	// A12: USB_DP, AF14
	GPIOA->MODER |= GPIO_MODER_MODER11_1 | GPIO_MODER_MODER12_1;
	GPIOA->AFR[1] |= (14 << GPIO_AFRH_AFRH3_Pos) | (14 << GPIO_AFRH_AFRH4_Pos);
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR11 | GPIO_OSPEEDER_OSPEEDR12;
	// A14: UART2 Tx, AF7, NOT USED
	// A15: UART2 Rx, AF7
	GPIOA->MODER |= GPIO_MODER_MODER15_1;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR15_0;
	GPIOA->AFR[1] |= 7 << GPIO_AFRH_AFRH7_Pos;
	// B3 : SPI1 CLK, AF5, need pull-up
	// B4 : SPI1 MISO, AF5
	// B5 : SPI1 MOSI, AF5
	GPIOB->MODER |= GPIO_MODER_MODER3_1 | GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3 | GPIO_OSPEEDER_OSPEEDR5;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR3_0;
	GPIOB->AFR[0] |= (5 << GPIO_AFRL_AFRL3_Pos) | (5 << GPIO_AFRL_AFRL4_Pos) | (5 << GPIO_AFRL_AFRL5_Pos);
	// B8 : Red LED, need open-drain
	GPIOB->MODER |= GPIO_MODER_MODER8_0;
	GPIOB->OTYPER |= GPIO_OTYPER_OT_8;
	GPIOB->BSRR = GPIO_BSRR_BS_8;
	// B9 : SPI1 CS, driven manually
	GPIOB->MODER |= GPIO_MODER_MODER9_0;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9;
	GPIOB->BSRR = GPIO_BSRR_BS_9;
	// B10: UART3 Tx, AF7, NOT USED
	// B11: UART3 Rx, AF7, NOT USED
	// C13: MPU interrupt
	// C15: Beeper
	GPIOC->MODER |= GPIO_MODER_MODER15_0;
	
	/* DMA --------------------------------------------------------------------------*/
	
	// SPI1 Rx
	DMA1_Channel2_SPI.CCR = DMA_CCR_MINC | DMA_CCR_PL_0 | DMA_CCR_TCIE;
	DMA1_Channel2_SPI.CMAR = (uint32_t)&sensor_raw;
	DMA1_Channel2_SPI.CNDTR = 0;
	DMA1_Channel2_SPI.CPAR = (uint32_t)&(SPI1->DR);
	
	DMA1_Channel2->CCR = DMA1_Channel2_SPI.CCR ;
	DMA1_Channel2->CMAR = (uint32_t)spi1_rx_buffer;
	DMA1_Channel2->CPAR = DMA1_Channel2_SPI.CPAR;
	
	// SPI1 Tx
	DMA1_Channel3->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_PL_1;
	DMA1_Channel3->CMAR = (uint32_t)spi1_tx_buffer;
	DMA1_Channel3->CPAR = (uint32_t)&(SPI1->DR);
	
	// UART2 Rx
	DMA1_Channel6->CCR = DMA_CCR_TCIE | DMA_CCR_MINC | DMA_CCR_TEIE;
	DMA1_Channel6->CMAR = (uint32_t)&radio_frame;
	DMA1_Channel6->CPAR = (uint32_t)&(USART2->RDR);

	// TIM2 for DSHOT
	DMA1_Channel2_TIM.CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_PL | DMA_CCR_PSIZE_1 | DMA_CCR_MSIZE_1;
	DMA1_Channel2_TIM.CMAR = (uint32_t)dshot;
	DMA1_Channel2_TIM.CNDTR = 17*4;
	DMA1_Channel2_TIM.CPAR = (uint32_t)&(TIM2->DMAR);

	/* Timers --------------------------------------------------------------------------*/
	
#if (ESC == DSHOT)
	// DMA driven timer for DShot600, 24Mhz: 0:15, 1:30, T:40
	TIM2->PSC = 0;
	TIM2->ARR = 80;
	//TIM2->DIER = TIM_DIER_UDE;
	TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
	TIM2->CCMR1 = (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE | (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
	TIM2->CCMR2 = (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE | (6 << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
	TIM2->DCR = (3 << TIM_DCR_DBL_Pos) | (13 << TIM_DCR_DBA_Pos);
	
	dshot[16*4+0] = 0;
	dshot[16*4+1] = 0;
	dshot[16*4+2] = 0;
	dshot[16*4+3] = 0;
#else	
	// One-pulse mode for OneShot125
	TIM2->CR1 = TIM_CR1_OPM;
	TIM2->PSC = 3-1;
	TIM2->ARR = SERVO_MAX*2 + 1;
	TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
	TIM2->CCMR1 = (7 << TIM_CCMR1_OC1M_Pos) | (7 << TIM_CCMR1_OC2M_Pos);
	TIM2->CCMR2 = (7 << TIM_CCMR2_OC3M_Pos) | (7 << TIM_CCMR2_OC4M_Pos);
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
	//TIM6->CR1 = TIM_CR1_CEN;
	
	// Processing time
	TIM7->PSC = 48-1; // 1us
	TIM7->ARR = 65535;
	TIM7->CR1 = TIM_CR1_CEN;
	
	// MPU timeout
	TIM15->PSC = 48-1; // 1us
	TIM15->ARR = TIMEOUT_SENSOR;
	TIM15->DIER = TIM_DIER_UIE;
	TIM15->CR1 = TIM_CR1_CEN;
	
	// VBAT
	TIM16->PSC = 48000-1; // 1ms
	TIM16->ARR = VBAT_PERIOD;
	TIM16->DIER = TIM_DIER_UIE;
	TIM16->CR1 = TIM_CR1_CEN;
	
	/* UART ---------------------------------------------------*/
	
#if (RADIO_TYPE == 2)
	USART2->BRR = 480; // 48MHz/100000bps
	USART2->CR1 = USART_CR1_UE | USART_CR1_M0 | USART_CR1_PCE;
	USART2->CR2 = (2 << USART_CR2_STOP_Pos) | USART_CR2_RXINV;
#else
	USART2->BRR = 417; // 48MHz/115200bps
	USART2->CR1 = USART_CR1_UE;
#endif
	USART2->CR3 = USART_CR3_EIE;
	
	/* SPI ----------------------------------------------------*/
	
	SPI1->CR1 = SPI_CR1_MSTR | (5 << SPI_CR1_BR_Pos) | SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_SSM | SPI_CR1_SSI; // SPI clock = clock APB2/64 = 48MHz/64 = 750 kHz
	SPI1->CR2 = SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN | SPI_CR2_FRXTH | SPI_CR2_ERRIE;
	
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
	SYSCFG->EXTICR[3] = SYSCFG_EXTICR4_EXTI13_PC;
	EXTI->RTSR |= EXTI_RTSR_TR13;
	//EXTI->IMR = EXTI_IMR_MR13; // To be enabled after MPU init
	
	NVIC_EnableIRQ(EXTI15_10_IRQn);
	NVIC_EnableIRQ(USART2_IRQn);
	NVIC_EnableIRQ(SPI1_IRQn);
	NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	NVIC_EnableIRQ(TIM4_IRQn);
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
	NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
	NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
	NVIC_EnableIRQ(USB_LP_CAN_RX0_IRQn);
	
	NVIC_SetPriority(EXTI15_10_IRQn,0);
	NVIC_SetPriority(USART2_IRQn,0);
	NVIC_SetPriority(SPI1_IRQn,0);
	NVIC_SetPriority(DMA1_Channel2_IRQn,0);
	NVIC_SetPriority(DMA1_Channel6_IRQn,0);
	NVIC_SetPriority(TIM4_IRQn,0);
	NVIC_SetPriority(TIM6_DAC_IRQn,0);
	NVIC_SetPriority(TIM1_BRK_TIM15_IRQn,0);
	NVIC_SetPriority(TIM1_UP_TIM16_IRQn,0);
	NVIC_SetPriority(USB_LP_CAN_RX0_IRQn,16);

	/* Host init -------------------------------------------*/
	
	usb_init();
	
	/* Sensor init ----------------------------------------------------*/
	
	wait_ms(1000);
	mpu_spi_init();
	set_mpu_host(0);
	EXTI->IMR = EXTI_IMR_MR13; // Enable external interrupt now
	
	/* Radio init ----------------------------------*/
	
	// Set IDLE interrupt
	//USART2->CR1 |= USART_CR1_IDLEIE | USART_CR1_RE;
}
