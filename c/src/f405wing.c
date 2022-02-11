#include "board.h"
#include "stm32f4xx.h" // CMSIS
#include "fc.h" // flags
#include "sensor.h" // mpu_spi_init()
#include "utils.h" // wait_ms()
#include "usb.h" // usb_init()
#include "osd.h" // osd_init()
#include "smart_audio.h" // sma_data_received()
#include <string.h> // memcpy()

/* Private defines ------------------------------------*/

/* Private macros ------------------------------------------*/

#define DMA_CLEAR_ALL_FLAGS_0 (DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 |DMA_LIFCR_CDMEIF0 |DMA_LIFCR_CFEIF0)
#define DMA_CLEAR_ALL_FLAGS_1 (DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 |DMA_LIFCR_CDMEIF1 |DMA_LIFCR_CFEIF1)
#define DMA_CLEAR_ALL_FLAGS_2 (DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTEIF2 |DMA_LIFCR_CDMEIF2 |DMA_LIFCR_CFEIF2)
#define DMA_CLEAR_ALL_FLAGS_3 (DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3 |DMA_LIFCR_CDMEIF3 |DMA_LIFCR_CFEIF3)
#define DMA_CLEAR_ALL_FLAGS_4 (DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4 | DMA_HIFCR_CTEIF4 |DMA_HIFCR_CDMEIF4 |DMA_HIFCR_CFEIF4)
#define DMA_CLEAR_ALL_FLAGS_5 (DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 |DMA_HIFCR_CDMEIF5 |DMA_HIFCR_CFEIF5)
#define DMA_CLEAR_ALL_FLAGS_6 (DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6 |DMA_HIFCR_CDMEIF6 |DMA_HIFCR_CFEIF6)
#define DMA_CLEAR_ALL_FLAGS_7 (DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTEIF7 |DMA_HIFCR_CDMEIF7 |DMA_HIFCR_CFEIF7)

/* Private types --------------------------------------*/

/* Global variables --------------------------------------*/

volatile uint8_t spi1_tx_buffer[15];
#if (ESC == DSHOT)
	volatile uint8_t dshot12[17*2];
	volatile uint8_t dshot34[17*2];
#endif
volatile _Bool sensor_busy;
volatile uint8_t radio_nb_bytes;
//volatile uint8_t uart1_tx_buffer[16];
//volatile uint8_t uart2_tx_buffer[4];
//volatile uint8_t uart3_tx_buffer[3];
//volatile uint8_t uart1_rx_nbytes;
#ifdef LED
	volatile uint8_t led_buffer[24*LED+1];
#endif

/* Private functions ------------------------------------------------*/

inline __attribute__((always_inline)) void sensor_spi_dma_enable(uint8_t size)
{
	DMA2_Stream0->NDTR = size + 1;
	DMA2_Stream3->NDTR = size + 1;
	DMA2_Stream0->CR |= DMA_SxCR_EN;
	DMA2_Stream3->CR |= DMA_SxCR_EN;
	SPI1->CR1 |= SPI_CR1_SPE;
}

inline __attribute__((always_inline)) void sensor_spi_dma_disable()
{
	DMA2->LIFCR = DMA_CLEAR_ALL_FLAGS_0 | DMA_CLEAR_ALL_FLAGS_3;
	DMA2_Stream0->CR &= ~DMA_SxCR_EN;
	DMA2_Stream3->CR &= ~DMA_SxCR_EN;
	while ((DMA2_Stream0->CR & DMA_SxCR_EN) && (DMA2_Stream3->CR & DMA_SxCR_EN)) {}
	SPI1->CR1 &= ~SPI_CR1_SPE;
}

inline __attribute__((always_inline)) void radio_uart_dma_enable(uint8_t size)
{
	DMA2_Stream5->NDTR = size;
	DMA2_Stream5->CR |= DMA_SxCR_EN;
	USART1->CR1 |= USART_CR1_RE;
}

inline __attribute__((always_inline)) void radio_uart_dma_disable()
{
	DMA2->HIFCR = DMA_CLEAR_ALL_FLAGS_5;
	DMA2_Stream5->CR &= ~DMA_SxCR_EN;
	while (DMA2_Stream5->CR & DMA_SxCR_EN) {};
	USART1->CR1 &= ~USART_CR1_RE;
}

/* Public functions ---------------------------------------*/

uint32_t HAL_RCC_GetHCLKFreq(void)
{
  return SystemCoreClock;
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
	USART1->SR; USART1->DR; // Clear IDLE and RXNE flags
	USART1->CR3 &= ~USART_CR3_DMAR;
	USART1->CR1 |= USART_CR1_RE | USART_CR1_IDLEIE | USART_CR1_RXNEIE;
}

void set_motors(uint16_t * motor_raw, _Bool * motor_telemetry)
{
#if (ESC == DSHOT)
	int i;
	uint8_t motor1_dshot[16];
	uint8_t motor2_dshot[16];
	uint8_t motor3_dshot[16];
	uint8_t motor4_dshot[16];

	if ((DMA1_Stream2->NDTR == 0) && (DMA1_Stream6->NDTR == 0)) {
		TIM3->DIER = 0;
		TIM3->CR1 = 0;
		TIM3->CNT = 60;
		DMA1->LIFCR = DMA_CLEAR_ALL_FLAGS_2;
		DMA1_Stream2->CR &= ~DMA_SxCR_EN;
		
		TIM4->DIER = 0;
		TIM4->CR1 = 0;
		TIM4->CNT = 60;
		DMA1->HIFCR = DMA_CLEAR_ALL_FLAGS_6;
		DMA1_Stream6->CR &= ~DMA_SxCR_EN;

		dshot_encode(&motor_raw[0], motor2_dshot, motor_telemetry[0]);
		dshot_encode(&motor_raw[1], motor1_dshot, motor_telemetry[1]);
		dshot_encode(&motor_raw[2], motor3_dshot, motor_telemetry[2]);
		dshot_encode(&motor_raw[3], motor4_dshot, motor_telemetry[3]);
		for (i=0; i<16; i++) {
			dshot12[i*2+0] = motor1_dshot[i];
			dshot12[i*2+1] = motor2_dshot[i];
			dshot34[i*2+0] = motor3_dshot[i];
			dshot34[i*2+1] = motor4_dshot[i];
		}

		DMA1_Stream2->NDTR = 17*2;
		DMA1_Stream2->CR |= DMA_SxCR_EN;
		TIM3->DIER = TIM_DIER_UDE;

		DMA1_Stream6->NDTR = 17*2;
		DMA1_Stream6->CR |= DMA_SxCR_EN;
		TIM4->DIER = TIM_DIER_UDE;
		
		TIM3->CR1 = TIM_CR1_CEN;
		TIM4->CR1 = TIM_CR1_CEN;
	}
#else
	TIM3->CCR1 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - (uint32_t)motor_raw[0];
	TIM3->CCR2 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - (uint32_t)motor_raw[1];
	TIM4->CCR1 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - (uint32_t)motor_raw[2];
	TIM4->CCR2 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - (uint32_t)motor_raw[3];
	TIM3->CR1 |= TIM_CR1_CEN;
	TIM4->CR1 |= TIM_CR1_CEN;
#endif
}

void toggle_led()
{
	GPIOC->ODR ^= GPIO_ODR_OD13;
}

void toggle_beeper(_Bool en)
{
	if (en)
		GPIOB->ODR ^= GPIO_ODR_OD4;
	else
		GPIOB->BSRR = GPIO_BSRR_BR_4;
}

void host_send(uint8_t * data, uint8_t size)
{
	USBD_CDC_SetTxBuffer(&USBD_device_handler, data, size);
	USBD_CDC_TransmitPacket(&USBD_device_handler);
}

int32_t get_t_us(void)
{
	return (int32_t)TIM11->CNT;
}

void trig_vbat_meas(void)
{
	ADC1->CR2 |= ADC_CR2_JSWSTART;
}

void osd_send(uint8_t * data, uint8_t size)
{
	/*uart3_tx_buffer[0] = size;
	uart3_tx_buffer[1] = data[0];
	uart3_tx_buffer[2] = data[1];
	osd_nbytes_to_receive = size;
	DMA1_Channel2->CNDTR = size + 1;
	DMA1_Channel2->CCR |= DMA_CCR_EN;
	USART3->CR1 |= USART_CR1_TE | USART_CR1_RE;*/
}

void runcam_send(uint8_t * data, uint8_t size)
{
	DMA2->HIFCR = DMA_CLEAR_ALL_FLAGS_7;
	DMA2_Stream7->CR &= ~DMA_SxCR_EN;
	while (DMA2_Stream7->CR & DMA_SxCR_EN) {};
	USART1->CR1 &= ~USART_CR1_TE;
	
	DMA2_Stream7->NDTR = size;
	DMA2_Stream7->CR |= DMA_SxCR_EN;
	USART1->CR1 |= USART_CR1_TE;
}

void sma_send(uint8_t * data, uint8_t size)
{
	/*memcpy((uint8_t*)uart1_tx_buffer, data, size);
	DMA1_Channel4->CNDTR = size;
	DMA1_Channel4->CCR |= DMA_CCR_EN;
	USART1->CR1 |= USART_CR1_TE;*/
}

#ifdef LED

	void set_leds(uint8_t * grb)
	{
		int i;

		if (DMA2_Channel5->CNDTR == 0) {
			TIM8->DIER = 0;
			TIM8->CR1 = 0;
			TIM8->CNT = 40;
			DMA2->IFCR = DMA_IFCR_CGIF5; // Clear all DMA flags
			DMA2_Channel5->CCR &= ~DMA_CCR_EN; // Disable DMA

			for (i=0; i<8; i++) {
				led_buffer[ 0+i] = (grb[0] & (1 << (7-i))) ? 38 : 19;
				led_buffer[ 8+i] = (grb[1] & (1 << (7-i))) ? 38 : 19;
				led_buffer[16+i] = (grb[2] & (1 << (7-i))) ? 38 : 19;
			}
			for (i=1; i<LED; i++) {
				memcpy((uint8_t*)&led_buffer[i*24], (uint8_t*)&led_buffer[(i-1)*24], 24);
			}
			led_buffer[24*LED] = 0;

			DMA2_Channel5->CNDTR = 24*LED+1;
			DMA2_Channel5->CCR |= DMA_CCR_EN;
			TIM8->DIER = TIM_DIER_CC2DE;
			TIM8->CR1 = TIM_CR1_CEN;
		}
	}

#endif

/* --------------------------------------------------------------------------------
 Interrupt routines
--------------------------------------------------------------------------------- */

void SysTick_Handler()
{
	t_ms++;
	flag_time = 1;
}

/* Sensor ready IRQ ---------------------------*/

void EXTI4_IRQHandler()
{
	EXTI->PR = EXTI_PR_PR4; // Clear pending request
	if ((REG_CTRL__SENSOR_HOST_CTRL == 0) && (!sensor_busy)) {
		sensor_read(59,14);
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

void DMA2_Stream0_IRQHandler()
{
	sensor_spi_dma_disable();
	sensor_busy = 0;

	if (REG_CTRL__SENSOR_HOST_CTRL == 1) // Send SPI read data to host
		host_send((uint8_t*)&sensor_raw.bytes[1], 1);
	else {
		flag_sensor = 1; // Raise flag for sample ready
	}
}

/* Radio UART IRQ ------------------------------*/

void USART1_IRQHandler()
{
	if (USART1->SR & USART_SR_IDLE) {
		USART1->DR; // Clear IDLE flag
		radio_nb_bytes = sizeof(radio_frame);
	}
	else if (USART1->SR & USART_SR_RXNE) {
		USART1->DR; // clear RXNE flag
		radio_nb_bytes--;
		if (radio_nb_bytes == 0) {
			USART1->CR1 &= ~(USART_CR1_RE | USART_CR1_IDLEIE | USART_CR1_RXNEIE);
			USART1->CR3 |= USART_CR3_DMAR;
			radio_uart_dma_enable(sizeof(radio_frame));
		}
	}
	else {
		USART1->DR; // Clear status flags
		radio_error_count++;
		radio_sync();
	}
}

/* DMA IRQ of radio Rx UART-----------------------*/

void DMA2_Stream5_IRQHandler()
{
	radio_uart_dma_disable();
	radio_uart_dma_enable(sizeof(radio_frame));
	flag_radio = 1; // Raise flag for radio commands ready
}

/* VBAT ADC conversion IRQ -----------------------------*/

void ADC1_2_IRQHandler()
{
	ADC1->SR = ADC_SR_JEOC; // Clear IRQ
	//ADC1->SR = 0;
	
	vbat = (float)ADC1->JDR1 * REG_VBAT_SCALE;
	#ifdef IBAT
		ibat = (float)ADC1->JDR2 * REG_IBAT_SCALE;
	#endif
	flag_vbat = 1;
}

/* USB interrupt -----------------------------*/

void OTG_FS_IRQHandler()
{
	HAL_PCD_IRQHandler(&PCD_handler);
}

/* OSD UART IRQ --------------------------*/
/*
void USART3_IRQHandler()
{
	osd_data_received[--osd_nbytes_to_receive] = USART3->RDR;

	if (osd_nbytes_to_receive == 0) {
		DMA1->IFCR = DMA_IFCR_CGIF2; // Clear all DMA flags
		DMA1_Channel2->CCR &= ~DMA_CCR_EN;
		USART3->CR1 &= ~(USART_CR1_TE | USART_CR1_RE);

		if (REG_CTRL__OSD_HOST_CTRL == 1)
			host_send((uint8_t*)osd_data_received, 2);
		else if (osd_nbytes_to_send > 0)
			osd_send((uint8_t*)&osd_data_to_send[--osd_nbytes_to_send], 1);
	}
}
*/
/* Smart Audio UART IRQ -----------------------------------*/
/*
void USART1_IRQHandler()
{
	if (USART1->ISR & USART_ISR_TC) { // transfer complete

		USART1->ICR = USART_ICR_TCCF; // Clear transfer complete flag
		DMA1->IFCR = DMA_IFCR_CGIF4; // Clear all DMA flags
		DMA1_Channel4->CCR &= ~DMA_CCR_EN;
		USART1->CR1 &= ~USART_CR1_TE;
		//GPIOB->MODER &= ~GPIO_MODER_MODER6_1; // Disable Tx output pin since Smart audio is bidir

		if (sma_nbytes_to_receive > 0) {
			uart1_rx_nbytes = 0;
			USART1->CR1 |= USART_CR1_RE; // Enable Rx
		}

	} else if (USART1->ISR & USART_ISR_RXNE) { // Rx not empty

		sma_data_received[uart1_rx_nbytes++] = USART1->RDR;
		sma_nbytes_to_receive--;

		if (sma_nbytes_to_receive == 0) {
			USART1->CR1 &= ~USART_CR1_RE;
			if (REG_CTRL__SMA_HOST_CTRL == 1)
				host_send((uint8_t*)sma_data_received, uart1_rx_nbytes);
		}
	}
}
*/
/* ---------------------------------------------------------------------
board init
--------------------------------------------------------------------- */

void board_init()
{
	/* RCC --------------------------------------------------------*/

	// Enable XTAL oscillator
	RCC->CR |= RCC_CR_HSEON;
	while ((RCC->CR & RCC_CR_HSERDY) == 0) {}

	// Set VCO at 192 MHz = XTAL(8MHz) * (N=192) / (M=8)
	// PLL system = VCO / ((P=0,default)+2) = 96 MHz , PLL USB = VCO / (Q=4) = 48 MHz
	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM_Msk | RCC_PLLCFGR_PLLN_Msk | RCC_PLLCFGR_PLLP_Msk | RCC_PLLCFGR_PLLQ_Msk);
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

	// DMA clock enable
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;
	// DMA[y]_Stream[x]->CR = (3 << DMA_SxCR_CHSEL_Pos) | (1 << DMA_SxCR_PL_Pos) | (1 << DMA_SxCR_MSIZE_Pos) | (1 << DMA_SxCR_PSIZE_Pos) | DMA_SxCR_MINC | DMA_SxCR_PINC | (1 << DMA_SxCR_DIR_Pos) | DMA_SxCR_TCIE | DMA_SxCR_TEIE;

	// System configuration controller clock enable (to manage external interrupt line connection to GPIOs)
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	/* GPIO ------------------------------------------------*/

	// MODER: 00:IN, 01:OUT, 10:AF, 11:analog
	// PUPDR: 00:float 01:PU, 10:PD
	// OTYPER: 0:PP, 1:OD
	// OSPEEDR: 00:low 4MHz, 01:mid 25MHz, 10:high 50MHz, 11:very high 100MHz

	// GPIO clock enable
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;
	
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
	GPIOD->MODER = 0;
	GPIOD->PUPDR = 0;
	GPIOD->OSPEEDR = 0;
	
	// A13: SWDIO, AF0
	// A14: SWCLK, AF0

	/* USB ----------------------------------------*/

	// A11: USB_DM, AF10
	// A12: USB_DP, AF10
	GPIOA->MODER |= GPIO_MODER_MODER11_1 | GPIO_MODER_MODER12_1;
	GPIOA->AFR[1] |= (10 << GPIO_AFRH_AFSEL11_Pos) | (10 << GPIO_AFRH_AFSEL12_Pos);
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR11 | GPIO_OSPEEDER_OSPEEDR12;

	// Clock and interrupt enable
	RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
	NVIC_EnableIRQ(OTG_FS_IRQn);
	NVIC_SetPriority(OTG_FS_IRQn,16);

	// Init
	usb_init();

	/* Gyro/accel sensor ----------------------------------------------------*/

	// A4 : SPI1 CS, AF5, need pull-up
	// A5 : SPI1 CLK, AF5, need pull-up (CPOL = 1)
	// A6 : SPI1 MISO, AF5
	// A7 : SPI1 MOSI, AF5
	GPIOA->MODER |= GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4_1 | GPIO_OSPEEDER_OSPEEDR5_1 | GPIO_OSPEEDER_OSPEEDR7_1;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR4_0 | GPIO_PUPDR_PUPDR5_0;
	GPIOA->OTYPER |= GPIO_OTYPER_OT_4;
	GPIOA->AFR[0] |= (5 << GPIO_AFRL_AFSEL4_Pos) | (5 << GPIO_AFRL_AFSEL5_Pos) | (5 << GPIO_AFRL_AFSEL6_Pos) | (5 << GPIO_AFRL_AFSEL7_Pos);
	
	// SPI config
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	RCC->APB2LPENR |= RCC_APB2LPENR_SPI1LPEN;
	SPI1->CR1 = SPI_CR1_MSTR | (5 << SPI_CR1_BR_Pos) | SPI_CR1_CPOL | SPI_CR1_CPHA; // SPI clock = clock APB2/64 = 48MHz/64 = 750 kHz
	SPI1->CR2 = SPI_CR2_SSOE | SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN | SPI_CR2_ERRIE;
	NVIC_EnableIRQ(SPI1_IRQn);
	NVIC_SetPriority(SPI1_IRQn,0);

	// Sensor interrrupt, PC4
	SYSCFG->EXTICR[1] = SYSCFG_EXTICR2_EXTI4_PC; 
	EXTI->RTSR = EXTI_RTSR_TR4;
	NVIC_EnableIRQ(EXTI4_IRQn);
	NVIC_SetPriority(EXTI4_IRQn,0);

	// DMA SPI1 Rx
	DMA2_Stream0->CR = (3 << DMA_SxCR_CHSEL_Pos) | (1 << DMA_SxCR_PL_Pos) | DMA_SxCR_MINC | DMA_SxCR_TCIE;
	DMA2_Stream0->M0AR = (uint32_t)&sensor_raw;
	DMA2_Stream0->PAR = (uint32_t)&(SPI1->DR);
	NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	NVIC_SetPriority(DMA2_Stream0_IRQn,0);

	// DMA SPI1 Tx
	DMA2_Stream3->CR = (3 << DMA_SxCR_CHSEL_Pos) | (3 << DMA_SxCR_PL_Pos) | DMA_SxCR_MINC | (1 << DMA_SxCR_DIR_Pos);
	DMA2_Stream3->M0AR = (uint32_t)spi1_tx_buffer;
	DMA2_Stream3->PAR = (uint32_t)&(SPI1->DR);

	// Init
	mpu_init();
	EXTI->IMR = EXTI_IMR_MR4; // enable external interrupt now

	/* Radio Rx UART ---------------------------------------------------*/

	// A10: UART1 Rx, AF7, pull-up for IDLE
	GPIOA->MODER |= GPIO_MODER_MODER10_1;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR10_0;
	GPIOA->AFR[1] |= 7 << GPIO_AFRH_AFSEL10_Pos;

	// UART config
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	USART1->BRR = 417; // 48MHz/115200bps
	USART1->CR1 = USART_CR1_UE;
	USART1->CR3 = USART_CR3_DMAR | USART_CR3_EIE;
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_SetPriority(USART1_IRQn,0);

	// DMA UART1 Rx
	DMA2_Stream5->CR = (4 << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_TCIE | DMA_SxCR_TEIE;
	DMA2_Stream5->M0AR = (uint32_t)&radio_frame;
	DMA2_Stream5->PAR = (uint32_t)&(USART1->DR);
	NVIC_EnableIRQ(DMA2_Stream5_IRQn);
	NVIC_SetPriority(DMA2_Stream5_IRQn,0);

	// Init
	radio_sync();

	/* Motors -----------------------------------------*/

	// B4 : Motor 1, TIM3_CH1, AF2
	// B5 : Motor 2, TIM3_CH2, AF2
	// B6 : Motor 3, TIM4_CH1, AF2
	// B7 : Motor 4, TIM4_CH2, AF2
	GPIOB->MODER |= GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4 | GPIO_OSPEEDER_OSPEEDR5 | GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7;
	GPIOB->AFR[0] |= (2 << GPIO_AFRL_AFSEL4_Pos) | (2 << GPIO_AFRL_AFSEL5_Pos) | (2 << GPIO_AFRL_AFSEL6_Pos) | (2 << GPIO_AFRL_AFSEL7_Pos);
	
	// TIM clock enable
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;
	//RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM4EN;

#if (ESC == DSHOT)
	// DMA driven timer for DShot600, 48Mhz: 0:30, 1:60, T:80
	TIM3->PSC = 2; // 1:DShot600, 2:DShot300
	TIM3->ARR = 80;
	//TIM3->DIER = TIM_DIER_UDE; // Managed in motor function
	TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
	TIM3->CCMR1 = (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE | (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
	TIM3->DCR = (1 << TIM_DCR_DBL_Pos) | (13 << TIM_DCR_DBA_Pos);
	
	TIM4->PSC = 2;
	TIM4->ARR = 80;
	TIM4->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
	TIM4->CCMR1 = (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE | (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
	TIM4->DCR = (1 << TIM_DCR_DBL_Pos) | (13 << TIM_DCR_DBA_Pos);

	// Pad DSHOT pulse with zeros
	dshot12[16*2+0] = 0;
	dshot12[16*2+1] = 0;
	dshot34[16*2+0] = 0;
	dshot34[16*2+1] = 0;

	// DMA TIM3_UP
	DMA1_Stream2->CR = (5 << DMA_SxCR_CHSEL_Pos) | (2 << DMA_SxCR_PL_Pos) | (2 << DMA_SxCR_MSIZE_Pos) | (2 << DMA_SxCR_PSIZE_Pos) | DMA_SxCR_MINC | (1 << DMA_SxCR_DIR_Pos);
	DMA1_Stream2->M0AR = (uint32_t)dshot12;
	DMA1_Stream2->PAR = (uint32_t)&(TIM3->DMAR);

	// DMA TIM5_UP
	DMA1_Stream6->CR = (2 << DMA_SxCR_CHSEL_Pos) | (2 << DMA_SxCR_PL_Pos) | (2 << DMA_SxCR_MSIZE_Pos) | (2 << DMA_SxCR_PSIZE_Pos) | DMA_SxCR_MINC | (1 << DMA_SxCR_DIR_Pos);
	DMA1_Stream6->M0AR = (uint32_t)dshot34;
	DMA1_Stream6->PAR = (uint32_t)&(TIM4->DMAR);

#else
	// One-pulse mode for OneShot125
	TIM3->CR1 = TIM_CR1_OPM;
	TIM3->PSC = 3-1;
	TIM3->ARR = SERVO_MAX*2 + 1;
	TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
	TIM3->CCMR1 = (7 << TIM_CCMR1_OC1M_Pos) | (7 << TIM_CCMR1_OC2M_Pos);

	TIM4->CR1 = TIM_CR1_OPM;
	TIM4->PSC = 3-1;
	TIM4->ARR = SERVO_MAX*2 + 1;
	TIM4->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
	TIM4->CCMR1 = (7 << TIM_CCMR1_OC1M_Pos) | (7 << TIM_CCMR1_OC2M_Pos);
#endif

	/* LED -----------------------*/

	// A13: Blue LED, need open-drain
	// A14: Green LED, need open-drain
	GPIOA->MODER |= GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0;
	GPIOA->OTYPER |= GPIO_OTYPER_OT_13 | GPIO_OTYPER_OT_14;
	GPIOA->BSRR = GPIO_BSRR_BS_13 | GPIO_BSRR_BS_14;

	/* Time in us ----------------------------------------------------------*/

	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	TIM6->PSC = 48-1; // 1us
	TIM6->ARR = 65535;
	TIM6->CR1 = TIM_CR1_CEN;
	
	/* VBAT ADC -----------------------------------------------------*/

#ifdef VBAT

	// B0 : VBAT, ADC1_8
	// B1 : Current, ADC1_9
	GPIOB->MODER |= GPIO_MODER_MODER0 | GPIO_MODER_MODER1;

	// ADC config
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // Clock enable
	//ADC1_COMMON->CCR = (0 << ADC_CCR_ADCPRE_Pos); // ADC clock prescaler (div from APB2 clock), 0=2, 1=4, 2=6, 3=8
	ADC1->CR1 = ADC_CR1_SCAN;
	ADC1->JSQR = (1 << ADC_JSQR_JL_Pos) | (8 << ADC_JSQR_JSQ3_Pos) | (9 << ADC_JSQR_JSQ4_Pos); // 2 injected conversion of channel 8 and 9
	ADC1->SMPR2 = (2 << ADC_SMPR2_SMP8_Pos) | (2 << ADC_SMPR2_SMP9_Pos); // sampling time (in cycles), 0=3, 1=15, 2=28, 3=56

	// Enable
	ADC1->CR2 |= ADC_CR2_ADON;
	wait_ms(1);

	// Get first value
	ADC1->CR2 |= ADC_CR2_JSWSTART;
	while ((ADC1->SR & ADC_SR_JEOC) == 0) {}
	vbat_smoothed = (float)ADC1->JDR1 * VBAT_SCALE;
	ibat = (float)ADC1->JDR2 * IBAT_SCALE;
	//ADC1->SR = 0;

	NVIC_EnableIRQ(ADC1_2_IRQn);
	NVIC_SetPriority(ADC1_2_IRQn,0);

	// Get first vbat value
	trig_vbat_meas();
	while (!flag_vbat)
		__WFI();
	flag_vbat = 0;

#endif

	/* Beeper -----------------------------------------*/

#ifdef BEEPER

	// B0 : Servo 1, used as beeper
	GPIOB->MODER |= GPIO_MODER_MODER0_0;
	GPIOB->BSRR = GPIO_BSRR_BR_0;

#endif

	/* OSD ----------------------------------------*/

#ifdef OSD

	// B10: UART3 Tx, AF7, pull-up for IDLE
	// B11: UART3 Rx, AF7, pull-up for IDLE
	GPIOB->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR10_0 | GPIO_PUPDR_PUPDR11_0;
	GPIOB->AFR[1] |= (7 << GPIO_AFRH_AFRH2_Pos) | (7 << GPIO_AFRH_AFRH3_Pos);

	// UART config
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	USART3->BRR = 5000; // 48MHz/9600bps
	USART3->CR1 = USART_CR1_UE | USART_CR1_RXNEIE;
	USART3->CR3 = USART_CR3_DMAT;
	NVIC_EnableIRQ(USART3_IRQn);
	NVIC_SetPriority(USART3_IRQn,0);

	// DMA UART3 Tx
	DMA1_Channel2->CCR = DMA_CCR_MINC | DMA_CCR_DIR;
	DMA1_Channel2->CMAR = (uint32_t)uart3_tx_buffer;
	DMA1_Channel2->CPAR = (uint32_t)&(USART3->TDR);

	// Init
	osd_init();

#endif

	/* Runcam OSD ---------------------------------------*/

#ifdef RUNCAM

	// A9 : UART1 Tx, AF7, pull-up for IDLE
	GPIOA->MODER |= GPIO_MODER_MODER9_1;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR9_0;
	GPIOA->AFR[1] |= 7 << GPIO_AFRH_AFSEL9_Pos;

	// UART1 extra config for Tx (already configured for Rx)
	USART1->CR3 |= USART_CR3_DMAT;

	// DMA UART1 Tx
	DMA2_Stream7->CR = (4 << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | (1 << DMA_SxCR_DIR_Pos);
	DMA2_Stream7->M0AR = (uint32_t)host_buffer_rx.data.u8;
	DMA2_Stream7->PAR = (uint32_t)&(USART1->DR);

#endif

	/* Smart Audio ---------------------------------------*/

#ifdef SMART_AUDIO

	// B6: UART1 Tx, AF7, pull-down to set audio line to GND
	GPIOB->MODER |= GPIO_MODER_MODER6_1;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR6_1;
	GPIOB->AFR[0] |= (7 << GPIO_AFRL_AFRL6_Pos);

	// UART config
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	USART1->BRR = 9800; // 48MHz/4800bps = 10000 => 48MHz/4900bps = 9800 in order to match TBS unify board Xtal
	USART1->CR2 = 2 << USART_CR2_STOP_Pos; // 2 stop bits
	USART1->CR3 = USART_CR3_DMAT | USART_CR3_HDSEL; // HDSEL: single-wire half-duplex
	USART1->CR1 = USART_CR1_UE | USART_CR1_TCIE | USART_CR1_RXNEIE;
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_SetPriority(USART1_IRQn,0);

	// DMA UART1 Tx
	DMA1_Channel4->CCR = DMA_CCR_MINC | DMA_CCR_DIR;
	DMA1_Channel4->CMAR = (uint32_t)uart1_tx_buffer;
	DMA1_Channel4->CPAR = (uint32_t)&(USART1->TDR);

	sma_send_cmd(SMA_GET_SETTINGS, 0);
	wait_sma();
	sma_process_resp();
	REG_VTX = (vtx_current_chan << REG_VTX__CHAN_Pos) | (vtx_current_pwr << REG_VTX__PWR_Pos);

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

}
