#include "board.h"
#include "fc.h"
#include "radio.h"
#include "sensor.h"

/* Private defines ------------------------------------*/

#define ADC_SCALE 0.0089f

/* Private types --------------------------------------*/

/* Global variables --------------------------------------*/

volatile uint8_t i2c1_rx_buffer[15];
volatile uint8_t i2c1_tx_buffer[2];
volatile uint8_t i2c1_tx_nb_bytes;

/* Private macros ---------------------------------------------------*/

/* Functions ------------------------------------------------*/

void sensor_write(uint8_t addr, uint8_t data)
{
	i2c1_tx_buffer[0] = addr;
	i2c1_tx_buffer[1] = data;
	DMA1_Channel3->CNDTR = 0;
	i2c1_tx_nb_bytes = 0;
	I2C1->CR2 &= ~(I2C_CR2_RD_WRN | I2C_CR2_NBYTES);
	I2C1->CR2 |= (2 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
}

void sensor_read(uint8_t addr, uint8_t size)
{
	i2c1_tx_buffer[0] = addr;
	DMA1_Channel3->CNDTR = size;
	i2c1_tx_nb_bytes = 0;
	I2C1->CR2 &= ~(I2C_CR2_RD_WRN | I2C_CR2_NBYTES);
	I2C1->CR2 |= (1 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
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
	DMA1->IFCR = DMA_IFCR_CGIF5;
	DMA1_Channel5->CCR &= ~DMA_CCR_EN;
	USART1->CR3 &= ~USART_CR3_DMAR;
	USART1->CR1 &= ~USART_CR1_RE;
	
	USART1->ICR = USART_ICR_IDLECF | USART_ICR_ORECF | USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NCF; // Clear status flags
	USART1->CR1 |= USART_CR1_IDLEIE | USART_CR1_RE; // Set IDLE interrupt
		
	radio_error_count++;
}

__forceinline void sensor_error_recover()
{
	// Disable DMA I2C
	DMA1->IFCR = DMA_IFCR_CGIF3;
	DMA1_Channel3->CCR &= ~DMA_CCR_EN;
	I2C1->CR1 &= ~I2C_CR1_PE;
	I2C1->CR1 |= I2C_CR1_PE;
	
	sensor_error_count++;
}

__forceinline void host_error_recover()
{
	// Disable DMA UART
	DMA1->IFCR = DMA_IFCR_CGIF6 | DMA_IFCR_CGIF7;
	DMA1_Channel6->CCR &= ~DMA_CCR_EN;
	DMA1_Channel7->CCR &= ~DMA_CCR_EN;
	USART2->CR1 &= ~USART_CR1_RE;
	
	// Clear status flags;
	USART2->ICR = USART_ICR_ORECF | USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NCF;
	
	// Enable DMA UART
	DMA1_Channel6->CNDTR = sizeof(host_buffer_rx);
	DMA1_Channel6->CCR |= DMA_CCR_EN;
	USART2->CR1 |= USART_CR1_RE;
}

void set_motors(uint32_t * motor_raw)
{
	TIM3->CCR1 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - motor_raw[0];
	TIM3->CCR2 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - motor_raw[1];
	TIM3->CCR3 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - motor_raw[2];
	TIM3->CCR4 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - motor_raw[3];
	TIM3->CR1 |= TIM_CR1_CEN;
}

void toggle_led_sensor()
{
	GPIOB->ODR ^= GPIO_ODR_3;
}

void toggle_led_radio()
{
	GPIOB->ODR ^= GPIO_ODR_3;
}

void set_mpu_host(_Bool host)
{
	if (host)
		DMA1_Channel3->CMAR = (uint32_t)i2c1_rx_buffer;
	else
		DMA1_Channel3->CMAR = (uint32_t)&sensor_raw + 1;
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
	DMA1->IFCR = DMA_IFCR_CGIF7;
	DMA1_Channel7->CCR &= ~DMA_CCR_EN;
	DMA1_Channel7->CMAR = (uint32_t)data;
	DMA1_Channel7->CNDTR = size;
	DMA1_Channel7->CCR |= DMA_CCR_EN;
}

uint16_t get_timer_process(void)
{
	return TIM7->CNT;
}

/* Interrupt routines -------------------------------------------------------------
-----------------------------------------------------------------------------------*/

/* Sample valid from MPU ------------------------------*/

void EXTI15_10_IRQHandler() 
{
	EXTI->PR = EXTI_PR_PIF12; // Clear pending request
	if ((REG_CTRL__SENSOR_HOST_CTRL == 0) && ((I2C1->ISR & I2C_ISR_BUSY) == 0)) {
		sensor_read(59,14);
		timer_sensor[0] = TIM7->CNT; // SPI transaction time
	}
}

/* MPU I2C error ------------------------*/

void I2C1_ER_IRQHandler()
{
	sensor_error_recover();
}

/* MPU I2C R/W management ------------------------*/

void I2C1_EV_IRQHandler()
{
	if (I2C1->ISR & I2C_ISR_TXIS) { // Tx buffer empty
		I2C1->TXDR = i2c1_tx_buffer[i2c1_tx_nb_bytes];
		i2c1_tx_nb_bytes++;
	}
	else if (I2C1->ISR & I2C_ISR_TC) { // Transfer completed
		if (DMA1_Channel3->CNDTR > 0) {
			DMA1_Channel3->CCR |= DMA_CCR_EN; // Enable DMA for Rx
			// Restart I2C transfer for read
			I2C1->CR2 &= ~I2C_CR2_NBYTES;
			I2C1->CR2 |= (DMA1_Channel3->CNDTR << I2C_CR2_NBYTES_Pos) | I2C_CR2_RD_WRN | I2C_CR2_START;
		}
		else {
			// End I2C transfer
			I2C1->CR2 |= I2C_CR2_STOP;
			while (I2C1->ISR & I2C_ISR_BUSY) {}
		}
	}
}

/* End of MPU I2C receive -------------------------------------*/

void DMA1_Channel3_IRQHandler() 
{
	if (DMA1->ISR & DMA_ISR_TEIF3) // Check DMA transfer error
		sensor_error_recover();
	else {
		// Disable DMA I2C
		DMA1->IFCR = DMA_IFCR_CGIF3;
		DMA1_Channel3->CCR &= ~DMA_CCR_EN;
		
		if (flag_sensor_host_read) {
			flag_sensor_host_read = 0;
			host_send((uint8_t*)&i2c1_rx_buffer[0], 1);
		}
		else {
			TIM15->CNT = 0; // Reset timeout
			flag_sensor = 1; // Raise flag for sample ready
		}
	}
	
	// I2C transaction time
	timer_sensor[1] = TIM7->CNT;
}

/* Host UART error ------------------------*/

void USART2_IRQHandler()
{
	host_error_recover();
}

/* End of Host UART receive ------------------*/

void DMA1_Channel6_IRQHandler()
{
	if (DMA1->ISR & DMA_ISR_TEIF6) // Check DMA transfer error
		host_error_recover();
	else {
		// Disable DMA UART
		DMA1->IFCR = DMA_IFCR_CGIF6;
		DMA1_Channel6->CCR &= ~DMA_CCR_EN;
		USART2->CR1 &= ~USART_CR1_RE;
		
		flag_host = 1; // Raise flag for host request
	}
	
	// Enable DMA UART
	DMA1_Channel6->CNDTR = sizeof(host_buffer_rx);
	DMA1_Channel6->CCR |= DMA_CCR_EN;
	USART2->CR1 |= USART_CR1_RE;
}

/* Host UART Tx DMA error ------------------------*/

void DMA1_Channel7_IRQHandler()
{
	host_error_recover();
}

/* Radio UART IRQ ------------------------*/

void USART1_IRQHandler()
{
	if (USART1->ISR & USART_ISR_IDLE) {
		USART1->ICR = USART_ICR_IDLECF; // Clear IDLE flag
		
		// Enable DMA UART
		USART1->CR1 &= ~USART_CR1_IDLEIE;
		DMA1_Channel5->CNDTR = sizeof(radio_frame);
		DMA1_Channel5->CCR |= DMA_CCR_EN;
		USART1->CR3 |= USART_CR3_DMAR;
	}
	else
		radio_error_recover();
}

/* End of Radio UART receive ------------------*/

void DMA1_Channel5_IRQHandler()
{
	if (DMA1->ISR & DMA_ISR_TEIF5) // Check DMA transfer error
		radio_error_recover();
	else {
		// Disable DMA UART
		DMA1->IFCR = DMA_IFCR_CGIF5;
		DMA1_Channel5->CCR &= ~DMA_CCR_EN;
		USART1->CR1 &= ~USART_CR1_RE;
		
		flag_radio = 1; // Raise flag for radio commands ready
		
		// Enable DMA UART
		DMA1_Channel5->CNDTR = sizeof(radio_frame);
		DMA1_Channel5->CCR |= DMA_CCR_EN;
		USART1->CR1 |= USART_CR1_RE;
	}
}

/* Beeper period -----------------------*/
/*
void TIM4_IRQHandler()
{
	TIM4->SR &= ~TIM_SR_UIF;
	
	if (flag_beep_user || flag_beep_radio || flag_beep_sensor || flag_beep_host || flag_beep_vbat)
		GPIOA->ODR ^= GPIO_ODR_0;
	else
		GPIOA->ODR &= ~GPIO_ODR_0;
}
*/
/* Radio timeout ------------------------*/

void TIM6_DAC_IRQHandler()
{
	TIM6->SR &= ~TIM_SR_UIF;
	flag_timeout_radio = 1;
}

/* MPU timeout ------------------------------*/

void TIM1_BRK_TIM15_IRQHandler()
{
	TIM15->SR &= ~TIM_SR_UIF;
	flag_timeout_sensor = 1;
}

/* VBAT sampling time -------------------------*/

void TIM1_UP_TIM16_IRQHandler()
{
	TIM16->SR &= ~TIM_SR_UIF;
	flag_vbat = 1;
}

/* INIT -----------------------------------------------------------------
-----------------------------------------------------------------------*/

void board_init()
{
	/* RCC --------------------------------------------------------*/
	
	// Enable XTAL oscillator
	RCC->CR |= RCC_CR_HSEON | RCC_CR_HSEBYP;
	while ((RCC->CR & RCC_CR_HSERDY) == 0) {}
	
	// Set PLL at 48MHz = 6 * XTAL
	RCC->CFGR |= RCC_CFGR_PLLMUL6 | RCC_CFGR_PLLSRC_HSE_PREDIV;
	RCC->CR |= RCC_CR_PLLON;
	while ((RCC->CR & RCC_CR_PLLRDY) == 0) {}
	
	// Select PLL as system clock
	FLASH->ACR |= 1 << FLASH_ACR_LATENCY_Pos; // Inrease Flash latency!
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while ((RCC->CFGR & RCC_CFGR_SWS_PLL) == 0) {}
	SystemCoreClock = 48000000; // Update system clock value
	
	// Set APB1 at 24MHz
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
	
	// Select system clock as UART1 clock and  as I2C1/2 clock
	RCC->CFGR3 |= RCC_CFGR3_USART1SW_SYSCLK | RCC_CFGR3_I2C1SW_SYSCLK;
	
	// Configure SysTick to generate interrrupt every ms
	SysTick_Config(48000); 
	
	/* Clock enable --------------------------------------------------*/
	
	// UART clock enable
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	// I2C clock enable
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	// Timer clock enable
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM6EN | RCC_APB1ENR_TIM7EN;
	RCC->APB2ENR |= RCC_APB2ENR_TIM15EN | RCC_APB2ENR_TIM16EN;
	// System configuration controller clock enable (to manage external interrupt line connection to GPIOs)
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	// DMA clock enable
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	// GPIO clock enable 
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;
	// ADC clock enable
	//RCC->AHBENR |= RCC_AHBENR_ADC12EN;
	
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
	
	// A2 : USART2 TX
	// A15: USART2 RX
	GPIOA->MODER |= GPIO_MODER_MODER2_1 | GPIO_MODER_MODER15_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2 | GPIO_OSPEEDER_OSPEEDR15;
	GPIOA->AFR[0] |= (7 << GPIO_AFRL_AFRL2_Pos);
	GPIOA->AFR[1] |= (7 << GPIO_AFRH_AFRH7_Pos);
	// A10: USART1 RX
	GPIOA->MODER |= GPIO_MODER_MODER10_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10;
	GPIOA->AFR[1] |= (7 << GPIO_AFRH_AFRH2_Pos);
	// A12: Sensor interrupt
	// B0 : Motor 3, TIM3 CH3
	// B1 : Motor 4, TIM3 CH4
	// B4 : Motor 1, TIM3 CH1
	// B5 : Motor 2, TIM3 CH2
	GPIOB->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1 | GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0 | GPIO_OSPEEDER_OSPEEDR1 | GPIO_OSPEEDER_OSPEEDR4 | GPIO_OSPEEDER_OSPEEDR5;
	GPIOB->AFR[0] |= (2 << GPIO_AFRL_AFRL0_Pos) | (2 << GPIO_AFRL_AFRL1_Pos) | (2 << GPIO_AFRL_AFRL4_Pos) | (2 << GPIO_AFRL_AFRL5_Pos);
	// B6 : I2C1 SCL
	// B7 : I2C1 SDA
	GPIOB->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7;
	GPIOB->OTYPER |= GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7;
	GPIOB->AFR[0] |= (4 << GPIO_AFRL_AFRL6_Pos) | (4 << GPIO_AFRL_AFRL7_Pos);
	// B3 : LED
	GPIOB->MODER |= GPIO_MODER_MODER3_0;
	GPIOB->BSRR = GPIO_BSRR_BR_3;
	// ? : VBAT
	//GPIO?->MODER |= GPIO_MODER_MODER?;
	
	/* DMA --------------------------------------------------------------------------*/
	
	// I2C1 Rx
	SYSCFG->CFGR3 = SYSCFG_CFGR3_I2C1_RX_DMA_RMP_0;
	DMA1_Channel3->CCR = DMA_CCR_MINC | DMA_CCR_PL_0 | DMA_CCR_TCIE | DMA_CCR_TEIE;
	DMA1_Channel3->CMAR = (uint32_t)i2c1_rx_buffer;
	DMA1_Channel3->CPAR = (uint32_t)&(I2C1->RXDR);
	
	// UART1 Rx
	DMA1_Channel5->CCR = DMA_CCR_TCIE | DMA_CCR_MINC | DMA_CCR_TEIE;
	DMA1_Channel5->CMAR = (uint32_t)&radio_frame;
	DMA1_Channel5->CPAR = (uint32_t)&(USART1->RDR);
	
	// UART2 Rx
	DMA1_Channel6->CCR = DMA_CCR_TCIE | DMA_CCR_MINC | DMA_CCR_TEIE;
	DMA1_Channel6->CMAR = (uint32_t)&host_buffer_rx;
	DMA1_Channel6->CPAR = (uint32_t)&(USART2->RDR);
	
	// UART2 Tx
	DMA1_Channel7->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TEIE;
	DMA1_Channel7->CPAR = (uint32_t)&(USART2->TDR);
	
	/* Timers --------------------------------------------------------------------------*/
	
	// One-pulse mode for OneShot125
	TIM3->CR1 = TIM_CR1_OPM;
	TIM3->PSC = 3-1;
	TIM3->ARR = SERVO_MAX*2 + 1;
	TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
	TIM3->CCMR1 = (7 << TIM_CCMR1_OC1M_Pos) | (7 << TIM_CCMR1_OC2M_Pos);
	TIM3->CCMR2 = (7 << TIM_CCMR2_OC3M_Pos) | (7 << TIM_CCMR2_OC4M_Pos);
	
	// Beeper
	/*TIM4->PSC = 48000-1; // 1ms
	TIM4->ARR = BEEPER_PERIOD;
	TIM4->DIER = TIM_DIER_UIE;
	TIM4->CR1 = TIM_CR1_CEN;*/
	
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
	TIM15->PSC = 48-1; // 1us
	TIM15->ARR = TIMEOUT_SENSOR;
	TIM15->DIER = TIM_DIER_UIE;
	//TIM15->CR1 = TIM_CR1_CEN; // To be enabled after sensor init
	
	// VBAT
	TIM16->PSC = 48000-1; // 1ms
	TIM16->ARR = VBAT_PERIOD;
	TIM16->DIER = TIM_DIER_UIE;
	//TIM16->CR1 = TIM_CR1_CEN;
	
	/* UART ---------------------------------------------------*/
	
	// Host
	USART2->BRR = 208; // 24MHz/115200bps
	USART2->CR3 = USART_CR3_DMAR | USART_CR3_DMAT | USART_CR3_EIE;
	USART2->CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;
	// Radio
	#if (RADIO_TYPE == SBUS)
		USART1->BRR = 480; // 48MHz/100000bps
		USART1->CR1 = USART_CR1_UE | USART_CR1_M0 | USART_CR1_PCE;
		USART1->CR2 = (2 << USART_CR2_STOP_Pos) | USART_CR2_RXINV;
	#else
		USART1->BRR = 417; // 48MHz/115200bps
		USART1->CR1 = USART_CR1_UE;
	#endif
	USART1->CR3 = USART_CR3_EIE;
	
	/* I2C ------------------------------------------------------*/
	
	I2C1->CR1 = I2C_CR1_TCIE | I2C_CR1_TXIE | I2C_CR1_RXDMAEN | I2C_CR1_ERRIE;
	I2C1->CR2 = 104 << (1+I2C_CR2_SADD_Pos);
	I2C1->TIMINGR = (5 << I2C_TIMINGR_PRESC_Pos) | (9 << I2C_TIMINGR_SCLL_Pos) | (3 << I2C_TIMINGR_SCLH_Pos) | (3 << I2C_TIMINGR_SDADEL_Pos) | (3 << I2C_TIMINGR_SCLDEL_Pos); // 400kHz (Fast-mode)
	I2C1->CR1 |= I2C_CR1_PE;
	
	/* ADC -----------------------------------------------------*/
	/*
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
	*/
	/* Interrupts ---------------------------------------------------*/
	
	// MPU interrrupt
	SYSCFG->EXTICR[3] = SYSCFG_EXTICR4_EXTI12_PA;
	EXTI->RTSR |= EXTI_RTSR_TR12;
	//EXTI->IMR = EXTI_IMR_MR12; // To be enabled after MPU init
	
	NVIC_EnableIRQ(EXTI15_10_IRQn);
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_EnableIRQ(USART2_IRQn);
	NVIC_EnableIRQ(I2C1_ER_IRQn);
	NVIC_EnableIRQ(I2C1_EV_IRQn);
	NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	NVIC_EnableIRQ(DMA1_Channel5_IRQn);
	NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	NVIC_EnableIRQ(DMA1_Channel7_IRQn);
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
	NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
	NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
	
	NVIC_SetPriority(EXTI15_10_IRQn,0);
	NVIC_SetPriority(USART1_IRQn,0);
	NVIC_SetPriority(USART2_IRQn,0);
	NVIC_SetPriority(I2C1_ER_IRQn,0);
	NVIC_SetPriority(I2C1_EV_IRQn,0);
	NVIC_SetPriority(DMA1_Channel3_IRQn,0);
	NVIC_SetPriority(DMA1_Channel5_IRQn,0);
	NVIC_SetPriority(DMA1_Channel6_IRQn,0);
	NVIC_SetPriority(DMA1_Channel7_IRQn,0);
	NVIC_SetPriority(TIM6_DAC_IRQn,0);
	NVIC_SetPriority(TIM1_BRK_TIM15_IRQn,0);
	NVIC_SetPriority(TIM1_UP_TIM16_IRQn,0);

	/* Host init -----------------------------------------------------------*/
	
	DMA1_Channel6->CNDTR = sizeof(host_buffer_rx);
	DMA1_Channel6->CCR |= DMA_CCR_EN;

	/* Sensor init ----------------------------------------------------*/
	
	wait_ms(1000);
	mpu_i2c_init();
	set_mpu_host(0);
	EXTI->IMR = EXTI_IMR_MR12; // Enable interrupt
	TIM15->CR1 = TIM_CR1_CEN; // Enable timeout
	
	/* Radio init ----------------------------------*/
	
	USART1->CR1 |= USART_CR1_IDLEIE | USART_CR1_RE; // Set IDLE interrupt
	TIM6->CR1 = TIM_CR1_CEN; // Enable timeout
}
