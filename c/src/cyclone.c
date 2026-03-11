#include <stdint.h>
#include <stdbool.h>
#include <string.h> // memcpy()

#include "board.h"
#include "stm32f3xx.h" // CMSIS
#include "fc.h" // flags
#include "sensor.h" // mpu_spi_init()
#include "utils.h" // wait_ms()
#include "usb.h" // usb_init()
#include "osd.h" // osd_init()
#include "smart_audio.h" // sma_data_received()

/* Private defines ------------------------------------*/

/* Private macros ------------------------------------------*/

/* Private types --------------------------------------*/

/* Private variables --------------------------------------*/

volatile uint8_t spi2_rx_nbytes;
#if (ESC == DSHOT)
	volatile uint8_t dshot[17*4];
#endif
volatile bool dshot_busy = false;
volatile DMA_Channel_TypeDef dma_cfg_dshot;
volatile DMA_Channel_TypeDef dma_cfg_osd;
volatile uint8_t* uart1_tx_data;
volatile uint8_t* uart1_rx_data;
// Initialize uart1 variables to 0 to avoid unwanted transfer at startup 
volatile uint8_t uart1_tx_nbytes = 0; 
volatile uint8_t uart1_rx_nbytes = 0; 
volatile uint8_t uart1_byte_cnt = 0;
volatile uint8_t uart3_tx_buffer[128]; // OSD telemetry is 2 lines of 30 x 16-bit characters + 0xFF terminator
volatile uint8_t uart3_rx_nbytes;
#ifdef LED
	volatile uint8_t led_buffer[24*LED+1];
#endif

/* Private functions ------------------------------------------------*/

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

/* Public functions ---------------------------------------*/

void sensor_transfer(uint8_t* data_out, uint8_t* data_in, uint8_t size_out, uint8_t size_in)
{
	spi2_rx_nbytes = size_in; // Record transfer size

	// Disable DMA channels in order to configure them
	DMA1_Channel4->CCR &= ~DMA_CCR_EN;
	DMA1_Channel5->CCR &= ~DMA_CCR_EN;

	DMA1_Channel4->CMAR = (uint32_t)data_in;
	DMA1_Channel5->CMAR = (uint32_t)data_out;
	DMA1_Channel4->CNDTR = size_in;
	DMA1_Channel5->CNDTR = size_out;

	// Enable DMA channels and start SPI transaction
	DMA1_Channel4->CCR |= DMA_CCR_EN;
	DMA1_Channel5->CCR |= DMA_CCR_EN;
	SPI2->CR1 |= SPI_CR1_SPE;

	sensor_busy = true;
}

void en_sensor_irq(void)
{
	EXTI->IMR = EXTI_IMR_MR15;
}

void trig_radio_rx(void)
{
	radio_uart_dma_enable(sizeof(radio_frame));
}

void trig_delayed_radio_rx(void)
{
	TIM7->CR1 |= TIM_CR1_CEN;
}

void set_motors(uint16_t * motor_raw, bool * motor_telemetry)
{
#if (ESC == DSHOT)
	uint8_t i;
	uint8_t motor1_dshot[16];
	uint8_t motor2_dshot[16];
	uint8_t motor3_dshot[16];
	uint8_t motor4_dshot[16];

	if (!osd_busy) {
		TIM3->CR1 = 0; // Disable timer

		// NB: motor 1 is on CH2 and motor 2 is on CH1
		dshot_encode(&motor_raw[0], motor2_dshot, motor_telemetry[0]);
		dshot_encode(&motor_raw[1], motor1_dshot, motor_telemetry[1]);
		dshot_encode(&motor_raw[2], motor3_dshot, motor_telemetry[2]);
		dshot_encode(&motor_raw[3], motor4_dshot, motor_telemetry[3]);
		for (i=0; i<16; i++) {
			dshot[i*4+0] = motor1_dshot[i];
			dshot[i*4+1] = motor2_dshot[i];
			dshot[i*4+2] = motor3_dshot[i];
			dshot[i*4+3] = motor4_dshot[i];
		}

		// Configure DMA
		TIM3->DIER = 0; // DMA requests must be disabled before configuring DMA, otherwise there are glitches 
		DMA1_Channel3->CCR = 0;
		*DMA1_Channel3 = dma_cfg_dshot;
		DMA1_Channel3->CCR |= DMA_CCR_EN;
		
		//TIM3->CNT = TIM3->ARR; // Reset timer just before an update event (overflow)
		TIM3->DIER = TIM_DIER_UDE; // Enable Update DMA requests
		TIM3->CR1 = TIM_CR1_CEN; // Enable timer

		dshot_busy = true;
	}
#else
	TIM3->CCR1 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - (uint32_t)motor_raw[1];
	TIM3->CCR2 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - (uint32_t)motor_raw[0];
	TIM3->CCR3 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - (uint32_t)motor_raw[2];
	TIM3->CCR4 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - (uint32_t)motor_raw[3];
	TIM3->CR1 |= TIM_CR1_CEN;
#endif
}

void toggle_led(bool en)
{
	if (en)
		GPIOB->ODR ^= GPIO_ODR_5;
	else
		GPIOB->BSRR = GPIO_BSRR_BS_5;
}

void toggle_beeper(bool en)
{
	if (en)
		GPIOA->ODR ^= GPIO_ODR_0;
	else
		GPIOA->BSRR = GPIO_BSRR_BR_0;
}

void host_send(uint8_t * data, uint8_t size)
{
	USBD_CDC_SetTxBuffer(&USBD_device_handler, data, size);
	USBD_CDC_TransmitPacket(&USBD_device_handler);
}

uint16_t get_t_us(void)
{
	return TIM6->CNT;
}

void trig_vbat_meas(void)
{
	ADC2->CR |= ADC_CR_JADSTART;
}

__attribute__((section(".RamFunc"))) void osd_transfer(uint8_t* data_out, uint8_t* data_in, uint8_t size_out, uint8_t size_in)
{
	uart3_tx_buffer[0] = size_out;
	memcpy((uint8_t*)&uart3_tx_buffer[1], data_out, size_out);
	uart3_rx_nbytes = size_in; // Record transfer size

	DMA1_Channel2->CCR &= ~DMA_CCR_EN;
	DMA1_Channel2->CMAR = (uint32_t)uart3_tx_buffer;
	DMA1_Channel2->CNDTR = size_out + 1;
	DMA1_Channel2->CCR |= DMA_CCR_EN;

	dma_cfg_osd.CMAR = (uint32_t)data_in;
	dma_cfg_osd.CNDTR = size_in;

	if (!dshot_busy) {
		TIM3->DIER = 0; // Disable concurrent timer update DMA requests for DSHOT
		DMA1_Channel3->CCR = 0; 
		*DMA1_Channel3 = dma_cfg_osd;
		DMA1_Channel3->CCR |= DMA_CCR_EN; 
		USART3->CR1 |= USART_CR1_TE | USART_CR1_RE;

		osd_busy = true;
	}
}

void sma_transfer(uint8_t* data_out, uint8_t* data_in, uint8_t size_out, uint8_t size_in)
{
	uart1_tx_data = data_out;
	uart1_rx_data = data_in;
	uart1_tx_nbytes = size_out;
	uart1_rx_nbytes = size_in;
	USART1->CR1 &= ~(USART_CR1_RE | USART_CR1_RXNEIE); // Disable any pending Rx
	USART1->CR1 |= USART_CR1_TE | USART_CR1_TXEIE; // Enable UART Tx and Tx empty IRQ
	sma_busy = true;
}

void runcam_send(uint8_t * data, uint8_t size)
{
	uart1_tx_data = data;
	uart1_tx_nbytes = size;
	USART1->CR1 |= USART_CR1_TE | USART_CR1_TXEIE; // Enable UART Tx and Tx empty IRQ
	sma_busy = true; // Uses SMA busy flag as RUNCAM uses same UART ressource
}

#ifdef LED

void set_leds(uint8_t * grb)
{
	uint16_t i;

	TIM8->CR1 = 0; // disable timer
	DMA2->IFCR = DMA_IFCR_CGIF5; // Clear all DMA flags

	for (i=0; i<8; i++) {
		led_buffer[ 0+i] = (grb[0] & (1 << (7-i))) ? 38 : 19;
		led_buffer[ 8+i] = (grb[1] & (1 << (7-i))) ? 38 : 19;
		led_buffer[16+i] = (grb[2] & (1 << (7-i))) ? 38 : 19;
	}
	for (i=1; i<LED; i++) {
		memcpy((uint8_t*)&led_buffer[i*24], (uint8_t*)&led_buffer[(i-1)*24], 24);
	}
	led_buffer[24*LED] = 0;

	// Configure DMA
	TIM8->DIER = 0; // DMA requests must be disabled before configuring DMA, otherwise there are glitches
	DMA2_Channel5->CCR &= ~DMA_CCR_EN;
	DMA2_Channel5->CNDTR = 24*LED+1;
	DMA2_Channel5->CCR |= DMA_CCR_EN;

	TIM8->CNT = 40;
	TIM8->DIER = TIM_DIER_CC2DE; // Enable compare DMA requests
	TIM8->CR1 = TIM_CR1_CEN; // Enable timer
}

#endif

/* --------------------------------------------------------------------------------
 Interrupt routines
--------------------------------------------------------------------------------- */

__attribute__((section(".RamFunc"))) void SysTick_Handler()
{
	t_ms++;
	flag_time = 1;
}

/* Sensor ready IRQ ---------------------------*/

__attribute__((section(".RamFunc"))) void EXTI15_10_IRQHandler()
{
	EXTI->PR = EXTI_PR_PIF15; // Clear pending request
	if ((REG_CTRL__SENSOR_HOST_CTRL == 0) && (!sensor_busy)) {
		sensor_read_samples();
	}
}

/* DMA IRQ of sensor Rx SPI ----------------------*/

__attribute__((section(".RamFunc"))) void DMA1_Channel4_IRQHandler()
{
	DMA1->IFCR = DMA_IFCR_CTCIF4; // Clear transfer complete IRQ
	SPI2->CR1 &= ~SPI_CR1_SPE; // End SPI transaction (release NSS)
	sensor_busy = false;

	if (REG_CTRL__SENSOR_HOST_CTRL == 1) {
		host_send((uint8_t*)DMA1_Channel4->CMAR, spi2_rx_nbytes);
	}
	else {
		if (SPI2->SR & SPI_SR_OVR) { // Overrun error
			// Specific procedure to clear flag
			SPI2->DR;
			SPI2->SR;
			// Empty Rx FIFO
			while (SPI2->SR & SPI_SR_FRLVL_Msk) SPI2->DR;
			sensor_error_count++;
			flag_sensor = false; // In case last sample was not processed
		}
		else {
			flag_sensor = true; // Raise flag for sample ready
		}
	}
}

/* Radio UART IRQ ------------------------------*/

__attribute__((section(".RamFunc"))) void USART2_IRQHandler()
{
	USART2->ICR = USART_ICR_ORECF | USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NCF; // Clear all flags
	radio_error_count++;
	radio_uart_dma_enable(sizeof(radio_frame));
}

/* DMA IRQ of radio Rx UART-----------------------*/

__attribute__((section(".RamFunc"))) void DMA1_Channel6_IRQHandler()
{
	radio_uart_dma_disable();
	radio_uart_dma_enable(sizeof(radio_frame));
	flag_radio = 1; // Raise flag for radio commands ready
}

/* VBAT ADC conversion IRQ -----------------------------*/

__attribute__((section(".RamFunc"))) void ADC1_2_IRQHandler()
{
	ADC2->ISR = ADC_ISR_JEOS; // Clear IRQ

	vbat = (float)ADC2->JDR1 * REG_VBAT_SCALE;
#ifdef IBAT
	ibat = (float)ADC2->JDR2 * REG_IBAT_SCALE;
#endif
	flag_vbat = 1;
}

/* USB interrupt -----------------------------*/

void USB_LP_CAN_RX0_IRQHandler()
{
	HAL_PCD_IRQHandler(&PCD_handler);
}

/* DMA IRQ of OSD Rx UART ----------------------*/

__attribute__((section(".RamFunc"))) void DMA1_Channel3_IRQHandler()
{
	DMA1->IFCR = DMA_IFCR_CTCIF3; // Clear transfer complete IRQ

	if (osd_busy) {
		USART3->CR1 &= ~(USART_CR1_TE | USART_CR1_RE);
		if (osd_next_nbytes_to_send == 0)
			osd_busy = false;

		if (REG_CTRL__OSD_HOST_CTRL == 1) {
			host_send((uint8_t*)DMA1_Channel3->CMAR, uart3_rx_nbytes);
		}
		else if (osd_next_nbytes_to_send > 0) {
			osd_transfer(osd_next_data_to_send, (uint8_t*)DMA1_Channel3->CMAR, osd_next_nbytes_to_send, osd_next_nbytes_to_send);
			osd_next_nbytes_to_send = 0;
		}
	}
	else {
		dshot_busy = false;
	}
}

/* Smart Audio UART IRQ -----------------------------------*/

__attribute__((section(".RamFunc"))) void USART1_IRQHandler()
{
	// Transfer complete
	if (USART1->ISR & USART_ISR_TC) {
		USART1->TDR = 0; // Will clear Transfer complete flag, and also remaining Tx empty flag

		USART1->CR1 &= ~USART_CR1_TE; // Disable Tx
		uart1_byte_cnt = 0;
		
		if (uart1_rx_nbytes > 0) 
			USART1->CR1 |= USART_CR1_RE | USART_CR1_RXNEIE; // Enable Rx and Rx not empty IRQ
		else
			sma_busy = false;
	}
	// Tx empty
	else if (USART1->ISR & USART_ISR_TXE) {
		if (uart1_byte_cnt < uart1_tx_nbytes)
			USART1->TDR = uart1_tx_data[uart1_byte_cnt++]; // A write to data register will clear Tx empty flag
		else
			USART1->CR1 &= ~USART_CR1_TXEIE; // Disable Tx empty IRQ to avoid constant IRQ triggering until transfer complete
	}

	// Rx not empty
	if (USART1->ISR & USART_ISR_RXNE) {
		uart1_rx_data[uart1_byte_cnt++] = USART1->RDR; // A read from data register will clear Rx not empty flag

		if (uart1_byte_cnt == uart1_rx_nbytes) {
			USART1->CR1 &= ~(USART_CR1_RE | USART_CR1_RXNEIE); // Disable Rx and Rx not empty IRQ
			uart1_byte_cnt = 0;
			sma_busy = false;
			if (REG_CTRL__SMA_HOST_CTRL == 1)
				host_send((uint8_t*)uart1_rx_data, uart1_rx_nbytes);
		}
	}
}

/* ---------------------------------------------------------------------
Board init
--------------------------------------------------------------------- */

uint8_t board_init()
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
	FLASH->ACR |= 1 << FLASH_ACR_LATENCY_Pos; // Increase Flash latency!
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while ((RCC->CFGR & RCC_CFGR_SWS_PLL) == 0) {}
	SystemCoreClock = 48000000;

	// Set APB1 at 24MHz and APB2 & USB at 48MHz
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_USBPRE;

	// Select system clock as UART2/3/4/5 clock and as I2C1/2 clock
	RCC->CFGR3 |= RCC_CFGR3_USART2SW_SYSCLK | RCC_CFGR3_USART3SW_SYSCLK | RCC_CFGR3_UART4SW_SYSCLK | RCC_CFGR3_UART5SW_SYSCLK | RCC_CFGR3_I2C1SW_SYSCLK | RCC_CFGR3_I2C2SW_SYSCLK;

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
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

	// Reset non-zero registers
	GPIOA->MODER = 0;
	GPIOA->PUPDR = 0;
	GPIOA->OSPEEDR = 0;
	GPIOB->MODER = 0;
	GPIOB->PUPDR = 0;
	GPIOB->OSPEEDR = 0;

	// A0 : Buzzer, switched ground (inverted), TIM2_CH1 (AF1)
	// A1 : Motor 5, TIM2_CH2 (AF1)
	// A2 : Motor 6, TIM2_CH3 (AF1), TIM15_CH1 (AF9), USART2_TX (AF7)
	// A3 : Motor 7, TIM2_CH4 (AF1), TIM15_CH2 (AF9), USART2_RX (AF7)
	// A4 : Motor 1, TIM3_CH2 (AF2)
	// A5 : VBAT/10, ADC2_IN2
	// A6 : Motor 2, TIM3_CH1 (AF2), TIM16_CH1 (AF1)
	// A7 : PPM, TIM3_CH2 (AF2), TIM17_CH1 (AF1), TIM8_CH1N (AF4), ADC2_IN4
	// A8 : Motor 8, TIM1_CH1 (AF6)

	// A11: USB_DM (AF14)
	// A12: USB_DP (AF14)

	// A15: Gyro (MPU6000) interrupt

	// B0 : Motor 3, TIM3_CH3 (AF2), TIM8_CH2N (AF4), TIM1_CH2N (AF6)
	// B1 : Motor 4, TIM3_CH4 (AF2), TIM8_CH3N (AF4), TIM1_CH3N (AF6)
	// B2 : RSSI, ADC2_IN12
	// B3 : SBUS Tx, USART2_TX (AF7)
	// B4 : SBUS Rx, USART2_RX (AF7)
	// B5 : Red LED, external pull-up
	// B6 : USART1_TX (AF7)
	// B7 : USART1_RX (AF7)
	// B8 : SLED, TIM16_CH1 (AF1), TIM4_CH3 (AF2), TIM8_CH2 (AF10)

	// B10: USART3_TX (AF7)
	// B11: USART3_RX (AF7)
	// B12: Gyro (MPU600) SPI2_NSS  (AF5)
	// B13: Gyro (MPU600) SPI2_SCK  (AF5)
	// B14: Gyro (MPU600) SPI2_MISO (AF5)
	// B15: Gyro (MPU600) SPI2_MOSI (AF5)

	/* USB ----------------------------------------*/

	// A11: USB_DM (AF14)
	// A12: USB_DP (AF14)
	GPIOA->MODER |= GPIO_MODER_MODER11_1 | GPIO_MODER_MODER12_1;
	GPIOA->AFR[1] |= (14 << GPIO_AFRH_AFRH3_Pos) | (14 << GPIO_AFRH_AFRH4_Pos);
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR11 | GPIO_OSPEEDER_OSPEEDR12;

	// Clock and interrupt enable
	RCC->APB1ENR |= RCC_APB1ENR_USBEN;
	NVIC_EnableIRQ(USB_LP_CAN_RX0_IRQn);
	NVIC_SetPriority(USB_LP_CAN_RX0_IRQn,16);

	/* Gyro/accel sensor ----------------------------------------------------*/

	// B12: Gyro (MPU600) SPI2_NSS  (AF5), need pull-up
	// B13: Gyro (MPU600) SPI2_SCK  (AF5), CPOL=1 => need pull-up
	// B14: Gyro (MPU600) SPI2_MISO (AF5)
	// B15: Gyro (MPU600) SPI2_MOSI (AF5)
	GPIOB->MODER |= GPIO_MODER_MODER12_1 | GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR12 | GPIO_OSPEEDER_OSPEEDR13 | GPIO_OSPEEDER_OSPEEDR15;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR12_0 | GPIO_PUPDR_PUPDR13_0;
	GPIOB->AFR[1] |= (5 << GPIO_AFRH_AFRH4_Pos) | (5 << GPIO_AFRH_AFRH5_Pos) | (5 << GPIO_AFRH_AFRH6_Pos) | (5 << GPIO_AFRH_AFRH7_Pos);

	// SPI config
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
	SPI2->CR1 = SPI_CR1_MSTR | (4 << SPI_CR1_BR_Pos) | SPI_CR1_CPOL | SPI_CR1_CPHA; // SPI clock = clock APB1/32 = 24MHz/32 = 750 kHz
	SPI2->CR2 = SPI_CR2_SSOE | SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN | SPI_CR2_FRXTH;
	// NB: SPI_CR2_ERRIE and SPI2_IRQn are not enabled because:
	// - There are only 2 errors: MODF and OVR
	// - MODF (mode fault) should happen only in multi-master scenario
	// - OVR can be handled in DMA transfer complete interrupt

	// A15: Gyro (MPU6000) interrupt
	SYSCFG->EXTICR[3] = SYSCFG_EXTICR4_EXTI15_PA;
	EXTI->RTSR |= EXTI_RTSR_TR15;
	NVIC_EnableIRQ(EXTI15_10_IRQn);
	NVIC_SetPriority(EXTI15_10_IRQn,0);

	// DMA1 channel 4: SPI2_RX
	DMA1_Channel4->CCR = (1 << DMA_CCR_PL_Pos) | DMA_CCR_MINC | DMA_CCR_TCIE;// | DMA_CCR_TEIE; // TODO: see if error handling is needed
	DMA1_Channel4->CPAR = (uint32_t)&(SPI2->DR);
	NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	NVIC_SetPriority(DMA1_Channel4_IRQn,0);

	// DMA1 channel 5: SPI2_TX
	DMA1_Channel5->CCR = (2 << DMA_CCR_PL_Pos) | DMA_CCR_MINC | DMA_CCR_DIR;
	DMA1_Channel5->CPAR = (uint32_t)&(SPI2->DR);

	/* Radio UART ---------------------------------------------------*/

	// B4 : UART2 Rx, AF7, pull-up for IDLE
	GPIOB->MODER |= GPIO_MODER_MODER4_1;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR4_0;
	GPIOB->AFR[0] |= 7 << GPIO_AFRL_AFRL4_Pos;

	// UART config
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	USART2->BRR = 114; // 48MHz/420000bps
	USART2->CR1 = USART_CR1_UE;
	USART2->CR3 = USART_CR3_DMAR;// | USART_CR3_EIE; // TODO: test with error interrupt
	NVIC_EnableIRQ(USART2_IRQn);
	NVIC_SetPriority(USART2_IRQn,0);

	// DMA UART2 Rx
	DMA1_Channel6->CCR = (0 << DMA_CCR_PL_Pos) | DMA_CCR_MINC | DMA_CCR_TCIE;// | DMA_CCR_TEIE; // TODO: see if error handling is needed
	DMA1_Channel6->CMAR = (uint32_t)&radio_frame;
	DMA1_Channel6->CPAR = (uint32_t)&(USART2->RDR);
	NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	NVIC_SetPriority(DMA1_Channel6_IRQn,0);

	// Timer for radio sync
	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
	TIM7->PSC = 48-1; // 1us
	TIM7->ARR = 620; // 10bits*sizeof(radio_frame)/420000bps
	TIM7->CR1 = TIM_CR1_OPM;
	TIM7->DIER = TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM7_IRQn);
	NVIC_SetPriority(TIM7_IRQn,0);

	/* Motors -----------------------------------------*/

	// A4 : Motor 1, TIM3_CH2 (AF2)
	// A6 : Motor 2, TIM3_CH1 (AF2)
	// B0 : Motor 3, TIM3_CH3 (AF2)
	// B1 : Motor 4, TIM3_CH4 (AF2)
	GPIOA->MODER |= GPIO_MODER_MODER4_1 | GPIO_MODER_MODER6_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4 | GPIO_OSPEEDER_OSPEEDR6;
	GPIOA->AFR[0] |= (2 << GPIO_AFRL_AFRL4_Pos) | (2 << GPIO_AFRL_AFRL6_Pos);
	GPIOB->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0 | GPIO_OSPEEDER_OSPEEDR1;
	GPIOB->AFR[0] |= (2 << GPIO_AFRL_AFRL0_Pos) | (2 << GPIO_AFRL_AFRL1_Pos);

	// TIM3 clock enable
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

#if (ESC == DSHOT) // DMA driven timer for DShot600, 48Mhz: 0:30, 1:60, T:80

	#if (DSHOT_RATE == 300)
		TIM3->PSC = 1;
	#else // DSHOT_RATE == 600
		TIM3->PSC = 0;
	#endif
	TIM3->ARR = 80;
	TIM3->DIER = TIM_DIER_UDE;
	TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E; // Enable outputs
	TIM3->CCMR1 = (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE | (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE; // 6: PWM mode 1 (active when CNT < CCR) + Preload enabled
	TIM3->CCMR2 = (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE | (6 << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
	TIM3->DCR = (3 << TIM_DCR_DBL_Pos) | (13 << TIM_DCR_DBA_Pos);

	// Pad DSHOT pulse with zeros, to force end of DSHOT transaction
	dshot[16*4+0] = 0;
	dshot[16*4+1] = 0;
	dshot[16*4+2] = 0;
	dshot[16*4+3] = 0;

	// DMA1 channel 3: TIM3_UP
	dma_cfg_dshot.CCR = (3 << DMA_CCR_PL_Pos) | DMA_CCR_MINC | DMA_CCR_DIR | (1 << DMA_CCR_PSIZE_Pos) | DMA_CCR_TCIE; // TIM3 CCRx register is 16-bit
	dma_cfg_dshot.CMAR = (uint32_t)dshot;
	dma_cfg_dshot.CPAR = (uint32_t)&(TIM3->DMAR);
	dma_cfg_dshot.CNDTR = 17*4; 
	NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	NVIC_SetPriority(DMA1_Channel3_IRQn,0);

#elif (ESC == ONESHOT) // One-pulse mode for OneShot125 or PWM

	TIM3->CR1 = TIM_CR1_OPM;
	#if (ESC == PWM)
		TIM3->PSC = 24-1;
	#else
		TIM3->PSC = 3-1;
	#endif
	TIM3->ARR = SERVO_MAX*2 + 1;
	TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E; // Enable outputs
	TIM3->CCMR1 = (7 << TIM_CCMR1_OC1M_Pos) | (7 << TIM_CCMR1_OC2M_Pos); // 7: PWM mode 2 (active when CNT >= CCR)
	TIM3->CCMR2 = (7 << TIM_CCMR2_OC3M_Pos) | (7 << TIM_CCMR2_OC4M_Pos);

#endif

	/* LED -----------------------*/

	// B5 : Red LED, external pull-up => need open-drain
	GPIOB->MODER |= GPIO_MODER_MODER5_0;
	GPIOB->OTYPER |= GPIO_OTYPER_OT_5;
	GPIOB->BSRR = GPIO_BSRR_BS_5;

	/* Time in us ----------------------------------------------------------*/

	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	TIM6->PSC = 48-1; // 1us
	TIM6->ARR = 65535;
	TIM6->CR1 = TIM_CR1_CEN;
	
	/* VBAT ADC -----------------------------------------------------*/

	// A5 : VBAT/10, ADC2_IN2
	// A7 : PPM, ADC2_IN4
	// B2 : RSSI, ADC2_IN12, used for IBAT
#ifdef VBAT_USE_PPM
	GPIOA->MODER |= GPIO_MODER_MODER7;
#else
	GPIOA->MODER |= GPIO_MODER_MODER5;
#endif
#ifdef IBAT
	GPIOB->MODER |= GPIO_MODER_MODER2;
#endif

	// ADC Clock enable
	RCC->AHBENR |= RCC_AHBENR_ADC12EN; 
	
	// Select AHB clock as ADC clock
	ADC12_COMMON->CCR = ADC12_CCR_CKMODE_0;

	// Regulator startup
	ADC2->CR = 0;
	ADC2->CR = ADC_CR_ADVREGEN_0;
	wait_ms(1);

	// Calibration
	ADC2->CR |= ADC_CR_ADCAL;
	while (ADC2->CR & ADC_CR_ADCAL) {}
	wait_ms(1);

	// Enable
	ADC2->CR |= ADC_CR_ADEN;
	while ((ADC2->ISR & ADC_ISR_ADRDY) == 0) {}

	// ADC config, must be done when ADC is enabled
	ADC2->IER = ADC_IER_JEOSIE;
	ADC2->SMPR1 = (4 << ADC_SMPR1_SMP2_Pos) | (4 << ADC_SMPR1_SMP4_Pos);
	ADC2->SMPR2 = (4 << ADC_SMPR2_SMP12_Pos);
#ifdef VBAT_USE_PPM
	ADC2->JSQR = 4 << ADC_JSQR_JSQ1_Pos;
#else
	ADC2->JSQR = 2 << ADC_JSQR_JSQ1_Pos;
#endif
#ifdef IBAT
	ADC2->JSQR |= (1 << ADC_JSQR_JL_Pos) | (12 << ADC_JSQR_JSQ2_Pos);
#endif
	NVIC_EnableIRQ(ADC1_2_IRQn);
	NVIC_SetPriority(ADC1_2_IRQn,0);

	/* Beeper -----------------------------------------*/

#ifdef BEEPER

	// A0 : Beeper
	GPIOA->MODER |= GPIO_MODER_MODER0_0;
	GPIOA->BSRR = GPIO_BSRR_BR_0;

#endif

	/* OSD ----------------------------------------*/

#ifdef OSD

	// B10: USART3_TX (AF7), pull-up for IDLE
	// B11: USART3_RX (AF7), pull-up for IDLE
	GPIOB->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR10_0 | GPIO_PUPDR_PUPDR11_0;
	GPIOB->AFR[1] |= (7 << GPIO_AFRH_AFRH2_Pos) | (7 << GPIO_AFRH_AFRH3_Pos);

	// UART config
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	USART3->BRR = 48; // 48MHz/1Mbps
	USART3->CR1 = USART_CR1_UE;
	USART3->CR3 = USART_CR3_DMAT | USART_CR3_DMAR;

	// DMA1 channel 2: USART3_TX
	DMA1_Channel2->CCR = (0 << DMA_CCR_PL_Pos) | DMA_CCR_MINC | DMA_CCR_DIR;
	DMA1_Channel2->CPAR = (uint32_t)&(USART3->TDR);

	// DMA1 channel 3: USART3_RX
	dma_cfg_osd.CCR = (0 << DMA_CCR_PL_Pos) | DMA_CCR_MINC | DMA_CCR_TCIE;
	dma_cfg_osd.CPAR = (uint32_t)&(USART3->RDR);
#if (ESC == ONESHOT)
	NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	NVIC_SetPriority(DMA1_Channel3_IRQn,0);
#endif

#endif

	/* Smart Audio ---------------------------------------*/

#ifdef SMART_AUDIO

	// B6: UASRT1_TX (AF7), pull-down to set audio line to GND
	GPIOB->MODER |= GPIO_MODER_MODER6_1;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR6_1;
	GPIOB->AFR[0] |= (7 << GPIO_AFRL_AFRL6_Pos);

	// UART config
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	USART1->BRR = 9800; // 48MHz/4800bps = 10000 => 48MHz/4900bps = 9800 in order to match TBS unify board Xtal
	USART1->CR2 = 2 << USART_CR2_STOP_Pos; // 2 stop bits
	USART1->CR3 = USART_CR3_HDSEL; // HDSEL: single-wire half-duplex
	USART1->CR1 = USART_CR1_UE | USART_CR1_TCIE;
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_SetPriority(USART1_IRQn,0);

#else

	/* Runcam OSD ---------------------------------------*/

#ifdef RUNCAM

	// B6: UASRT1_TX (AF7), pull-up for IDLE
	GPIOB->MODER |= GPIO_MODER_MODER6_1;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR6_0;
	GPIOB->AFR[0] |= (7 << GPIO_AFRL_AFRL6_Pos);

	// UART config
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	USART1->BRR = 417; // 48MHz/115200bps
	USART1->CR1 = USART_CR1_UE | USART_CR1_TCIE;
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_SetPriority(USART1_IRQn,0);

#endif

#endif

	/* LED --------------------------------------------*/

#ifdef LED

	// B8 : LED, to TIM8_CH2, AF10
	GPIOB->MODER |= GPIO_MODER_MODER8_1;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8;
	GPIOB->AFR[1] |= (10 << GPIO_AFRH_AFRH0_Pos);

	// TIM8 clock enable
	RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;

	// DMA driven timer for WS2812B LED protocol, 48Mhz: 0:19, 1:38, T:60
	TIM8->PSC = 0;
	TIM8->ARR = 60;
	//TIM8->DIER = TIM_DIER_CC2DE; // Managed in led function
	TIM8->CCER = TIM_CCER_CC2E;
	TIM8->CCMR1 = (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
	TIM8->BDTR = TIM_BDTR_MOE; // output enable

	// DMA TIM8_CH2
	DMA2_Channel5->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_PSIZE_0;
	DMA2_Channel5->CMAR = (uint32_t)led_buffer;
	DMA2_Channel5->CPAR = (uint32_t)&(TIM8->CCR2);

#endif

	/* Settle time (in seconds) --------------------------------------------*/

#ifdef OSD
	return 6; // Arduino bootloader timeout
#else
	return 1; // Regulators settlement and components startup
#endif

}
