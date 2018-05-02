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

volatile uint8_t spi2_rx_buffer[16];
volatile uint8_t spi2_tx_buffer[16];
volatile uint32_t motor1_dshot[17];
volatile uint32_t motor2_dshot[17];
volatile uint32_t motor3_dshot[17];
volatile uint32_t motor4_dshot[17];

/* Functions ------------------------------------------------*/

void sensor_write(uint8_t addr, uint8_t data)
{
	spi2_tx_buffer[0] = addr & 0x7F;
	spi2_tx_buffer[1] = data;
	DMA1_Channel4->CNDTR = 2;
	DMA1_Channel5->CNDTR = 2;
	DMA1_Channel4->CCR |= DMA_CCR_EN;
	DMA1_Channel5->CCR |= DMA_CCR_EN;
	SPI2->CR1 |= SPI_CR1_SPE;
}

void sensor_read(uint8_t addr, uint8_t size)
{
	spi2_tx_buffer[0] = 0x80 | (addr & 0x7F);
	DMA1_Channel4->CNDTR = size+1;
	DMA1_Channel5->CNDTR = size+1;
	DMA1_Channel4->CCR |= DMA_CCR_EN;
	DMA1_Channel5->CCR |= DMA_CCR_EN;
	SPI2->CR1 |= SPI_CR1_SPE;
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

__forceinline void sensor_error_recover()
{
	// Disable DMA SPI
	DMA1->IFCR = DMA_IFCR_CGIF4 | DMA_IFCR_CGIF5;
	DMA1_Channel4->CCR &= ~DMA_CCR_EN;
	DMA1_Channel5->CCR &= ~DMA_CCR_EN;
	SPI2->CR1 &= ~SPI_CR1_SPE;
	/*while (SPI2->SR & SPI_SR_FRLVL)
		SPI2->DR;*/
	
	// Set master bit that could be reset after a SPI error
	SPI2->CR1 |= SPI_CR1_MSTR;
	
	sensor_error_count++;
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
		TIM2->CNT = 0;
		TIM3->CNT = 0;
		DMA1_Channel1->CCR &= ~DMA_CCR_EN;
		DMA1_Channel2->CCR &= ~DMA_CCR_EN;
		DMA1_Channel3->CCR &= ~DMA_CCR_EN;
		DMA1_Channel7->CCR &= ~DMA_CCR_EN;
		DMA1_Channel1->CNDTR = 17;
		DMA1_Channel2->CNDTR = 17;
		DMA1_Channel3->CNDTR = 17;
		DMA1_Channel7->CNDTR = 17;
		DMA1_Channel1->CCR |= DMA_CCR_EN;
		DMA1_Channel2->CCR |= DMA_CCR_EN;
		DMA1_Channel3->CCR |= DMA_CCR_EN;
		TIM3->CR1 |= TIM_CR1_CEN;
		DMA1_Channel7->CCR |= DMA_CCR_EN;
		TIM2->CR1 |= TIM_CR1_CEN;
	#else
		TIM2->CCR2 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - motor_raw[0];
		TIM2->CCR3 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - motor_raw[1];
		TIM3->CCR3 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - motor_raw[2];
		TIM3->CCR4 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - motor_raw[3];
		TIM2->CR1 |= TIM_CR1_CEN;
		TIM3->CR1 |= TIM_CR1_CEN;
	#endif
}

void toggle_led_sensor()
{
	GPIOB->ODR ^= GPIO_ODR_5;
}

void toggle_led_radio()
{
	GPIOB->ODR ^= GPIO_ODR_5;
}

void set_mpu_host(_Bool host)
{
	if (host) {
		DMA1_Channel4->CMAR = (uint32_t)spi2_rx_buffer;
		SPI2->CR1 &= ~SPI_CR1_BR_Msk;
		SPI2->CR1 |= 4 << SPI_CR1_BR_Pos; // 700kHz
	}
	else {
		DMA1_Channel4->CMAR = (uint32_t)&sensor_raw;
		SPI2->CR1 &= ~SPI_CR1_BR_Msk; // 12MHz
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

/* Interrupt routines -------------------------------------------------------------
-----------------------------------------------------------------------------------*/

/* Sample valid from MPU ---------------------------*/

void EXTI15_10_IRQHandler() 
{
	EXTI->PR = EXTI_PR_PIF15; // Clear pending request
	if ((REG_CTRL__SENSOR_HOST_CTRL == 0) && ((SPI2->SR & SPI_SR_BSY) == 0)) {
		sensor_read(59,14);
		timer_sensor[0] = TIM7->CNT; // SPI transaction time
	}
}

/* MPU SPI error ------------------------------*/

void SPI2_IRQHandler() 
{
	sensor_error_recover();
}

/* End of MPU SPI receive ----------------------*/

void DMA1_Channel4_IRQHandler() 
{
	if (DMA1->ISR & DMA_ISR_TEIF4) // Check DMA transfer error
		sensor_error_recover();
	else {
		// Disable DMA SPI
		DMA1->IFCR = DMA_IFCR_CGIF4 | DMA_IFCR_CGIF5;
		DMA1_Channel4->CCR &= ~DMA_CCR_EN;
		DMA1_Channel5->CCR &= ~DMA_CCR_EN;
		SPI2->CR1 &= ~SPI_CR1_SPE;
		
		if (flag_sensor_host_read) {
			flag_sensor_host_read = 0;
			host_send((uint8_t*)&spi2_rx_buffer[1], 1);
		}
		else {
			TIM15->CNT = 0; // Reset timeout
			flag_sensor = 1; // Raise flag for sample ready
		}
	}
	
	// SPI transaction time
	timer_sensor[1] = TIM7->CNT;
}

/* MPU SPI DMA Transfer error -----------------------*/

void DMA1_Channel5_IRQHandler() 
{
	sensor_error_recover();
}

/* Radio UART error ------------------------------*/

void USART2_IRQHandler()
{
	if (USART2->ISR & USART_ISR_IDLE) {
		 // Clear status flags
		USART2->ICR = USART_ICR_IDLECF;
		
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
		GPIOA->ODR ^= GPIO_ODR_0;
	else
		GPIOA->ODR &= ~GPIO_ODR_0;
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

/* INIT ----------------------------------------------------------------
-----------------------------------------------------------------------*/

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
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
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
	GPIOA->MODER |= GPIO_MODER_MODER0_0;
	// A1 : Motor 5, to TIM2_CH2, AF1
	// A2 : Motor 6, to TIM2_CH3, AF1
	GPIOA->MODER |= GPIO_MODER_MODER1_1 | GPIO_MODER_MODER2_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR1 | GPIO_OSPEEDER_OSPEEDR2;
	GPIOA->AFR[0] |= (1 << GPIO_AFRL_AFRL1_Pos) | (1 << GPIO_AFRL_AFRL2_Pos);
	// A3 : Motor 7
	// A4 : Motor 1, to TIM3_CH2, AF2, NOT USED
	// A5 : VBAT/10
	GPIOA->MODER |= GPIO_MODER_MODER5;
	// A6 : Motor 2, to TIM3_CH1, AF2, NOT USED
	// A7 : PPM
	// A8 : Motor 8
	// A11: USB_DM, AF14
	// A12: USB_DP, AF14
	GPIOA->MODER |= GPIO_MODER_MODER11_1 | GPIO_MODER_MODER12_1;
	GPIOA->AFR[1] |= (14 << GPIO_AFRH_AFRH3_Pos) | (14 << GPIO_AFRH_AFRH4_Pos);
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR11 | GPIO_OSPEEDER_OSPEEDR12;
	// A13: SWDIO, AF0
	// A14: SWCLK, AF0
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
	GPIOB->BSRR = GPIO_BSRR_BS_5;
	// B6 : UART1 Tx, NOT USED
	// B7 : UART1 Rx, NOT USED
	// B10: UART3 Tx, AF7, NOT USED
	// B11: UART3 Rx, AF7, NOT USED
	// B12: SPI2 CS, AF5, need pull-up
	// B13: SPI2 CLK, AF5, need pull-up
	// B14: SPI2 MISO, AF5
	// B15: SPI2 MOSI, AF5
	GPIOB->MODER |= GPIO_MODER_MODER12_1 | GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR12 | GPIO_OSPEEDER_OSPEEDR13 | GPIO_OSPEEDER_OSPEEDR15;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR12_0 | GPIO_PUPDR_PUPDR13_0;
	GPIOB->AFR[1] |= (5 << GPIO_AFRH_AFRH4_Pos) | (5 << GPIO_AFRH_AFRH5_Pos) | (5 << GPIO_AFRH_AFRH6_Pos) | (5 << GPIO_AFRH_AFRH7_Pos);
	
	/* DMA --------------------------------------------------------------------------*/
	
	// SPI2 Rx
	DMA1_Channel4->CCR = DMA_CCR_MINC | DMA_CCR_PL_0 | DMA_CCR_TCIE | DMA_CCR_TEIE;
	DMA1_Channel4->CMAR = (uint32_t)spi2_rx_buffer;
	DMA1_Channel4->CPAR = (uint32_t)&(SPI2->DR);

	// SPI2 Tx
	DMA1_Channel5->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_PL_1 | DMA_CCR_TEIE;
	DMA1_Channel5->CMAR = (uint32_t)spi2_tx_buffer;
	DMA1_Channel5->CPAR = (uint32_t)&(SPI2->DR);
	
	// UART2 Rx
	DMA1_Channel6->CCR = DMA_CCR_TCIE | DMA_CCR_MINC | DMA_CCR_TEIE;
	DMA1_Channel6->CMAR = (uint32_t)&radio_frame;
	DMA1_Channel6->CPAR = (uint32_t)&(USART2->RDR);

	// Timers for DSHOT
	DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_PL | DMA_CCR_PSIZE_1 | DMA_CCR_MSIZE_1;
	DMA1_Channel1->CMAR = (uint32_t)motor2_dshot;
	DMA1_Channel1->CPAR = (uint32_t)&(TIM2->CCR3);
	
	DMA1_Channel2->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_PL | DMA_CCR_PSIZE_1 | DMA_CCR_MSIZE_1;
	DMA1_Channel2->CMAR = (uint32_t)motor3_dshot;
	DMA1_Channel2->CPAR = (uint32_t)&(TIM3->CCR3);
	
	DMA1_Channel3->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_PL | DMA_CCR_PSIZE_1 | DMA_CCR_MSIZE_1;
	DMA1_Channel3->CMAR = (uint32_t)motor4_dshot;
	DMA1_Channel3->CPAR = (uint32_t)&(TIM3->CCR4);
	
	DMA1_Channel7->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_PL | DMA_CCR_PSIZE_1 | DMA_CCR_MSIZE_1;
	DMA1_Channel7->CMAR = (uint32_t)motor1_dshot;
	DMA1_Channel7->CPAR = (uint32_t)&(TIM2->CCR2);
	
	/* Timers --------------------------------------------------------------------------*/
	
#if (ESC == DSHOT)
	// DMA driven timer for DShot600, 24Mhz, 0:15, 1:30, T:40
	TIM2->PSC = 0;
	TIM2->ARR = 80;
	TIM2->DIER = TIM_DIER_CC2DE | TIM_DIER_CC3DE;
	TIM2->CCER = TIM_CCER_CC2E | TIM_CCER_CC3E;
	TIM2->CCMR1 = (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
	TIM2->CCMR2 = (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
	
	TIM3->PSC = 0;
	TIM3->ARR = 80;
	TIM3->DIER = TIM_DIER_CC3DE | TIM_DIER_CC4DE;
	TIM3->CCER = TIM_CCER_CC3E | TIM_CCER_CC4E;
	TIM3->CCMR2 = (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE | (6 << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
#else	
	// One-pulse mode for OneShot125
	TIM2->CR1 = TIM_CR1_OPM;
	TIM2->PSC = 3-1;
	TIM2->ARR = SERVO_MAX*2 + 1;
	TIM2->CCER = TIM_CCER_CC2E | TIM_CCER_CC3E;
	TIM2->CCMR1 = (7 << TIM_CCMR1_OC2M_Pos);
	TIM2->CCMR2 = (7 << TIM_CCMR2_OC3M_Pos);
	
	TIM3->CR1 = TIM_CR1_OPM;
	TIM3->PSC = 3-1;
	TIM3->ARR = SERVO_MAX*2 + 1;;
	TIM3->CCER = TIM_CCER_CC3E | TIM_CCER_CC4E;
	TIM3->CCMR2 = (7 << TIM_CCMR2_OC3M_Pos) | (7 << TIM_CCMR2_OC4M_Pos);
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
	TIM6->CR1 = TIM_CR1_CEN;
	
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
	
	SPI2->CR1 = SPI_CR1_MSTR | (4 << SPI_CR1_BR_Pos) | SPI_CR1_CPOL | SPI_CR1_CPHA; // SPI clock = clock APB1/32 = 24MHz/32 = 700 kHz
	SPI2->CR2 = SPI_CR2_SSOE | SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN | SPI_CR2_FRXTH | SPI_CR2_ERRIE;
	
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
	NVIC_EnableIRQ(SPI2_IRQn);
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
	NVIC_SetPriority(DMA1_Channel4_IRQn,0);
	NVIC_SetPriority(DMA1_Channel5_IRQn,0);
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
	EXTI->IMR = EXTI_IMR_MR15; // Enable external interrupt now
	
	/* Radio init ----------------------------------*/
	
	// Set IDLE interrupt
	USART2->CR1 |= USART_CR1_IDLEIE | USART_CR1_RE;
}
