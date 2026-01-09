#include "board.h"
#include "stm32f4xx.h" // CMSIS
#include "fc.h" // flags
#include "sensor.h" // sensor_raw
#include "utils.h" // wait_ms()
#include "usb.h" // USBD_device_handler
#include "osd.h" // osd_data_to_send
#include "smart_audio.h" // sma_data_received
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

volatile uint8_t spi1_tx_buffer[16];
#if (ESC == DSHOT)
	// Bit size must be the same as timer register, because MSIZE = PSIZE = register bit size
	volatile uint16_t dshot[17*4];
	volatile uint16_t dshot2[17*4];
#endif
volatile _Bool sensor_busy;
volatile uint8_t spi2_rx_buffer[3];
volatile uint8_t spi2_tx_buffer[3];
volatile uint8_t uart2_tx_buffer[16];
volatile uint8_t uart2_rx_nbytes;

/* Private functions ------------------------------------------------*/

inline __attribute__((always_inline)) void sensor_spi_dma_enable(uint8_t size)
{
	DMA2_Stream0->NDTR = size;
	DMA2_Stream3->NDTR = size;
	DMA2_Stream0->CR |= DMA_SxCR_EN;
	DMA2_Stream3->CR |= DMA_SxCR_EN;
	SPI1->CR1 |= SPI_CR1_SPE;
}

inline __attribute__((always_inline)) void sensor_spi_dma_disable()
{
	DMA2_Stream0->CR &= ~DMA_SxCR_EN;
	DMA2_Stream3->CR &= ~DMA_SxCR_EN;
	while ((DMA2_Stream0->CR & DMA_SxCR_EN) && (DMA2_Stream3->CR & DMA_SxCR_EN)) {} // Wait for end of current transfer
	DMA2->LIFCR = DMA_CLEAR_ALL_FLAGS_0 | DMA_CLEAR_ALL_FLAGS_3;
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
	DMA2_Stream5->CR &= ~DMA_SxCR_EN;
	while (DMA2_Stream5->CR & DMA_SxCR_EN) {}; // Wait for end of current transfer
	DMA2->HIFCR = DMA_CLEAR_ALL_FLAGS_5;
	USART1->CR1 &= ~USART_CR1_RE;
}

inline __attribute__((always_inline)) void osd_spi_dma_enable(uint8_t size)
{
	DMA1_Stream3->NDTR = size;
	DMA1_Stream4->NDTR = size;
	DMA1_Stream3->CR |= DMA_SxCR_EN;
	DMA1_Stream4->CR |= DMA_SxCR_EN;
	SPI2->CR1 |= SPI_CR1_SPE;
}

inline __attribute__((always_inline)) void osd_spi_dma_disable()
{
	DMA1_Stream3->CR &= ~DMA_SxCR_EN;
	DMA1_Stream4->CR &= ~DMA_SxCR_EN;
	while ((DMA1_Stream3->CR & DMA_SxCR_EN) && (DMA1_Stream4->CR & DMA_SxCR_EN)) {} // Wait for end of current transfer
	DMA1->LIFCR = DMA_CLEAR_ALL_FLAGS_3;
	DMA1->HIFCR = DMA_CLEAR_ALL_FLAGS_4;
	SPI2->CR1 &= ~SPI_CR1_SPE;
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
	sensor_spi_dma_enable(2);
	sensor_busy = 1;
	// Wait for end of transaction
	while (sensor_busy)
		__WFI();
}

void sensor_read(uint8_t addr, uint8_t size)
{
	spi1_tx_buffer[0] = 0x80 | (addr & 0x7F);
	sensor_spi_dma_enable(size+1);
	sensor_busy = 1;
}

void en_sensor_irq(void)
{
	EXTI->IMR = EXTI_IMR_MR1;
}

void trig_radio_rx(void)
{
	radio_uart_dma_enable(sizeof(radio_frame));
}

void trig_delayed_radio_rx(void)
{
	TIM10->CR1 |= TIM_CR1_CEN;
}

void set_motors(uint16_t * motor_raw, _Bool * motor_telemetry)
{
#if (ESC == DSHOT)
	int i;
	uint8_t motor1_dshot[16];
	uint8_t motor2_dshot[16];
	uint8_t motor3_dshot[16];
	uint8_t motor4_dshot[16];

	// Disable DMA TIM update events, master timer and DMA
	TIM2->DIER = 0;
	TIM2->CR1 = 0;
	DMA1_Stream1->CR &= ~DMA_SxCR_EN;
	DMA1_Stream7->CR &= ~DMA_SxCR_EN;
	while ((DMA1_Stream1->CR & DMA_SxCR_EN) && (DMA1_Stream7->CR & DMA_SxCR_EN)) {} // Wait for end of current transfer
	DMA1->LIFCR = DMA_CLEAR_ALL_FLAGS_1;
	DMA1->HIFCR = DMA_CLEAR_ALL_FLAGS_7;

	dshot_encode(&motor_raw[0], motor2_dshot, motor_telemetry[0]);
	dshot_encode(&motor_raw[1], motor1_dshot, motor_telemetry[1]);
	dshot_encode(&motor_raw[2], motor3_dshot, motor_telemetry[2]);
	dshot_encode(&motor_raw[3], motor4_dshot, motor_telemetry[3]);
	for (i=0; i<16; i++) {
		dshot[i*4+0] = motor1_dshot[i];
		dshot[i*4+1] = motor2_dshot[i];
		dshot2[i*4+0] = motor3_dshot[i];
		dshot2[i*4+1] = motor4_dshot[i];
	}

		
	DMA1_Stream1->NDTR = 17*4;
	DMA1_Stream7->NDTR = 17*4;
	DMA1_Stream1->CR |= DMA_SxCR_EN;
	DMA1_Stream7->CR |= DMA_SxCR_EN;
	// Reset timer just before an update event (overflow)
	TIM2->CNT = TIM2->ARR;
	TIM3->CNT = TIM3->ARR;
	TIM4->CNT = TIM4->ARR;
	// Enable DMA TIM update events and master timer
	TIM2->DIER = TIM_DIER_UDE;
	TIM2->CR1 = TIM_CR1_CEN;
#else
	TIM3->CCR1 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - (uint32_t)motor_raw[0];
	TIM3->CCR2 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - (uint32_t)motor_raw[1];
	TIM4->CCR1 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - (uint32_t)motor_raw[2];
	TIM4->CCR2 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - (uint32_t)motor_raw[3];
	TIM3->CR1 |= TIM_CR1_CEN;
	TIM4->CR1 |= TIM_CR1_CEN;
#endif
}

void toggle_led(_Bool en)
{
	if (en)
		GPIOC->ODR ^= GPIO_ODR_OD13;
	else
		GPIOC->BSRR = GPIO_BSRR_BS_13;
}

void toggle_led2(_Bool en)
{
	if (en)
		GPIOC->ODR ^= GPIO_ODR_OD14;
	else
		GPIOC->BSRR = GPIO_BSRR_BS_14;
}

void toggle_beeper(_Bool en)
{
	if (en)
		GPIOB->ODR ^= GPIO_ODR_OD2;
	else
		GPIOB->BSRR = GPIO_BSRR_BR_2;
}

void host_send(uint8_t * data, uint8_t size)
{
	USBD_CDC_SetTxBuffer(&USBD_device_handler, data, size);
	USBD_CDC_TransmitPacket(&USBD_device_handler);
}

uint16_t get_t_us(void)
{
	return TIM9->CNT;
}

void trig_vbat_meas(void)
{
	ADC1->CR2 |= ADC_CR2_JSWSTART;
}

void __attribute__((section(".RamFunc"))) osd_send(uint8_t * data, uint8_t size)
{
	memcpy((uint8_t*)spi2_tx_buffer, data, size);
	osd_nbytes_to_receive = size;
	osd_spi_dma_enable(size);
}

void sma_send(uint8_t * data, uint8_t size)
{
	memcpy((uint8_t*)uart2_tx_buffer, data, size);
	DMA1_Stream6->NDTR = size;
	DMA1_Stream6->CR |= DMA_SxCR_EN;
	USART2->CR1 |= USART_CR1_TE;
}

/* --------------------------------------------------------------------------------
 Interrupt routines
--------------------------------------------------------------------------------- */

void __attribute__((section(".RamFunc"))) SysTick_Handler()
{
	t_ms++;
	flag_time = 1;
}

/* Sensor ready IRQ ---------------------------*/

void __attribute__((section(".RamFunc"))) EXTI1_IRQHandler()
{
	EXTI->PR = EXTI_PR_PR1; // Clear pending request
	if ((REG_CTRL__SENSOR_HOST_CTRL == 0) && (!sensor_busy)) {
		sensor_read(59,14);
	}
}

/* Sensor SPI IRQ ------------------------------*/

void __attribute__((section(".RamFunc"))) SPI1_IRQHandler()
{
	SPI1->SR; // Read SR to clear flags
	sensor_spi_dma_disable();
	sensor_error_count++;
	if (REG_CTRL__SENSOR_HOST_CTRL == 0) {
		sensor_read(59,14);
	}
}

/* DMA IRQ of sensor Rx SPI ----------------------*/

void __attribute__((section(".RamFunc"))) DMA2_Stream0_IRQHandler()
{
	sensor_spi_dma_disable();
	sensor_busy = 0;

	if (REG_CTRL__SENSOR_HOST_CTRL == 1) {
		host_send(&sensor_raw.bytes[1], 1);
	} else {
		flag_sensor = 1; // Raise flag for sample ready
	}
}

/* Radio UART IRQ ------------------------------*/

void __attribute__((section(".RamFunc"))) USART1_IRQHandler()
{
	USART1->SR = 0; // Clear status flags
	radio_uart_dma_disable();
	radio_error_count++;
	radio_uart_dma_enable(sizeof(radio_frame));
}

/* DMA IRQ of radio Rx UART-----------------------*/

void __attribute__((section(".RamFunc"))) DMA2_Stream5_IRQHandler()
{
	radio_uart_dma_disable();
	flag_radio = 1; // Raise flag for radio commands ready
}

/* Timer IRQ of radio sync -----------------------*/

void __attribute__((section(".RamFunc"))) TIM1_UP_TIM10_IRQHandler()
{
	TIM10->SR = 0; // Clear IRQ
	radio_uart_dma_enable(sizeof(radio_frame));
}

/* VBAT ADC conversion IRQ -----------------------------*/

void __attribute__((section(".RamFunc"))) ADC_IRQHandler()
{
	ADC1->SR = 0; // Clear IRQ

	vbat = (float)ADC1->JDR1 * REG_VBAT_SCALE;
	ibat = (float)ADC1->JDR2 * REG_IBAT_SCALE;
	flag_vbat = 1;
}

/* USB interrupt -----------------------------*/

void OTG_FS_IRQHandler()
{
	HAL_PCD_IRQHandler(&PCD_handler);
}

/* DMA IRQ of OSR Rx SPI ----------------------*/

void __attribute__((section(".RamFunc"))) DMA1_Stream3_IRQHandler()
{
	uint8_t i, n;

	osd_spi_dma_disable();

	// To be compatible with UART OSD
	n = osd_nbytes_to_receive;
	for (i = 0; i < n; i++)
		osd_data_received[--osd_nbytes_to_receive] = spi2_rx_buffer[i];

	if (REG_CTRL__OSD_HOST_CTRL == 1) { 
		host_send((uint8_t*)osd_data_received, n);
	} else if (osd_nbytes_to_send > 0) {
		osd_send((uint8_t*)&osd_data_to_send[--osd_nbytes_to_send], 1);
	}
}

/* Smart Audio UART IRQ -----------------------------------*/

void __attribute__((section(".RamFunc"))) USART2_IRQHandler()
{
	if (USART2->SR & USART_SR_TC) { // transfer complete
		USART2->SR = 0; // Clear status flags

		DMA1_Stream6->CR &= ~DMA_SxCR_EN;
		while (DMA1_Stream6->CR & DMA_SxCR_EN) {} // Wait for end of current transfer
		DMA1->HIFCR = DMA_CLEAR_ALL_FLAGS_6;
		USART2->CR1 &= ~USART_CR1_TE;

		if (sma_nbytes_to_receive > 0) {
			uart2_rx_nbytes = 0;
			USART2->CR1 |= USART_CR1_RE; // Enable Rx
		}

	} else if (USART2->SR & USART_SR_RXNE) { // Rx not empty
		USART2->SR = 0; // Clear status flags

		sma_data_received[uart2_rx_nbytes++] = USART2->DR;
		sma_nbytes_to_receive--;

		if (sma_nbytes_to_receive == 0) {
			USART2->CR1 &= ~USART_CR1_RE;
			if (REG_CTRL__SMA_HOST_CTRL == 1)
				host_send((uint8_t*)sma_data_received, uart2_rx_nbytes);
		}
	}
}

/* ---------------------------------------------------------------------
Board init
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

	// Set APB1 and APB2 at 48 MHz (96/2)
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV2;

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
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
	
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

	// From Betaflight target MATEKF411:

	// A1 : Gyro (MPU6000) interrupt
	// A2 : USART2_TX (AF7)
	// A3 : USART2_RX (AF7)
	// A4 : Gyro (MPU600) SPI1_NSS  (AF5)
	// A5 : Gyro (MPU600) SPI1_SCK  (AF5)
	// A6 : Gyro (MPU600) SPI1_MISO (AF5)
	// A7 : Gyro (MPU600) SPI1_MOSI (AF5)
	// A8 : LED strip
	// A9 : USART1_TX (AF7), TIM1 CH2 (AF1)
	// A10: USART1_RX (AF7), TIM1 CH3 (AF1)
	// A11: USB_DM (AF10)
	// A12: USB_DP (AF10)
	
	// B0 : VBAT, ADC1 chan 8
	// B1 : Current meter, ADC1 chan 9
	// B2 : Beeper
	// B4 : Motor 1, TIM3_CH1 (AF2)
	// B5 : Motor 2, TIM3_CH2 (AF2)
	// B6 : Motor 3, TIM4_CH1 (AF2)
	// B7 : Motor 4, TIM4_CH2 (AF2)

	// B12: OSD (MAX7456) SPI2_NSS  (AF5)
	// B13: OSD (MAX7456) SPI2_SCK  (AF5)
	// B14: OSD (MAX7456) SPI2_MISO (AF5)
	// B15: OSD (MAX7456) SPI2_MOSI (AF5)

	// C13: Blue LED
	// C14: Green LED

	/* USB ----------------------------------------*/

	// A11: USB_DM (AF10)
	// A12: USB_DP (AF10)
	GPIOA->MODER |= GPIO_MODER_MODER11_1 | GPIO_MODER_MODER12_1;
	GPIOA->AFR[1] |= (10 << GPIO_AFRH_AFSEL11_Pos) | (10 << GPIO_AFRH_AFSEL12_Pos);
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR11 | GPIO_OSPEEDER_OSPEEDR12;

	// Clock and interrupt enable
	RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
	NVIC_EnableIRQ(OTG_FS_IRQn);
	NVIC_SetPriority(OTG_FS_IRQn,16);

	/* Gyro/accel sensor ----------------------------------------------------*/

	// A4 : Gyro (MPU600) SPI1_NSS  (AF5), need pull-up 
	// A5 : Gyro (MPU600) SPI1_SCK  (AF5), CPOL=1 => need pull-up 
	// A6 : Gyro (MPU600) SPI1_MISO (AF5)
	// A7 : Gyro (MPU600) SPI1_MOSI (AF5)
	GPIOA->MODER |= GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4_1 | GPIO_OSPEEDER_OSPEEDR5_1 | GPIO_OSPEEDER_OSPEEDR7_1;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR4_0 | GPIO_PUPDR_PUPDR5_0;
	GPIOA->AFR[0] |= (5 << GPIO_AFRL_AFSEL4_Pos) | (5 << GPIO_AFRL_AFSEL5_Pos) | (5 << GPIO_AFRL_AFSEL6_Pos) | (5 << GPIO_AFRL_AFSEL7_Pos);
	
	// SPI config
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	SPI1->CR1 = SPI_CR1_MSTR | (5 << SPI_CR1_BR_Pos) | SPI_CR1_CPOL | SPI_CR1_CPHA; // SPI clock = clock APB2/64 = 48MHz/64 = 750 kHz
	SPI1->CR2 = SPI_CR2_SSOE | SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN | SPI_CR2_ERRIE;
	NVIC_EnableIRQ(SPI1_IRQn);
	NVIC_SetPriority(SPI1_IRQn,0);

	// A1: Gyro (MPU6000) interrupt
	SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI1_PA; 
	EXTI->RTSR = EXTI_RTSR_TR1;
	NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_SetPriority(EXTI1_IRQn,0);

	// DMA2 stream 0 chan 3: SPI1_RX
	DMA2_Stream0->CR = (3 << DMA_SxCR_CHSEL_Pos) | (1 << DMA_SxCR_PL_Pos) | DMA_SxCR_MINC | DMA_SxCR_TCIE;
	DMA2_Stream0->M0AR = (uint32_t)&sensor_raw;
	DMA2_Stream0->PAR = (uint32_t)&(SPI1->DR);
	NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	NVIC_SetPriority(DMA2_Stream0_IRQn,0);

	// DMA2 stream 3 chan 3: SPI1_TX
	DMA2_Stream3->CR = (3 << DMA_SxCR_CHSEL_Pos) | (3 << DMA_SxCR_PL_Pos) | DMA_SxCR_MINC | (1 << DMA_SxCR_DIR_Pos);
	DMA2_Stream3->M0AR = (uint32_t)spi1_tx_buffer;
	DMA2_Stream3->PAR = (uint32_t)&(SPI1->DR);

	/* Radio Rx UART ---------------------------------------------------*/

	// A9 : USART1_TX (AF7), pull-up for IDLE
	// A10: USART1_RX (AF7), pull-up for IDLE
	GPIOA->MODER |= GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR10_0;
	GPIOA->AFR[1] |= (7 << GPIO_AFRH_AFSEL9_Pos) | (7 << GPIO_AFRH_AFSEL10_Pos);

	// UART config
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	USART1->BRR = 114; // 48MHz/420000bps
	USART1->CR1 = USART_CR1_UE;
	USART1->CR3 = USART_CR3_DMAR;// | USART_CR3_EIE; // TODO: test with error interrupt
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_SetPriority(USART1_IRQn,0);

	// DMA2 stream 5 chan 4: USART1_RX
	DMA2_Stream5->CR = (4 << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_TCIE;// | DMA_SxCR_TEIE; // TODO: see if error handling is needed
	DMA2_Stream5->M0AR = (uint32_t)&radio_frame;
	DMA2_Stream5->PAR = (uint32_t)&(USART1->DR);
	NVIC_EnableIRQ(DMA2_Stream5_IRQn);
	NVIC_SetPriority(DMA2_Stream5_IRQn,0);

	// DMA2 stream 7 chan 4: USART1_TX

	// Timer for radio sync
	RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
	TIM10->PSC = 96-1; // 1us, TIM9 is running at 96MHz instead of 48MHz
	TIM10->ARR = 620; // 10bits*sizeof(radio_frame)/420000bps
	TIM10->CR1 = TIM_CR1_OPM;
	TIM10->DIER = TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
	NVIC_SetPriority(TIM1_UP_TIM10_IRQn,0);

	/* Motors -----------------------------------------*/

	// B4 : Motor 1, TIM3_CH1 (AF2)
	// B5 : Motor 2, TIM3_CH2 (AF2)
	// B6 : Motor 3, TIM4_CH1 (AF2)
	// B7 : Motor 4, TIM4_CH2 (AF2)
	GPIOB->MODER |= GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4 | GPIO_OSPEEDER_OSPEEDR5 | GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7;
	GPIOB->AFR[0] |= (2 << GPIO_AFRL_AFSEL4_Pos) | (2 << GPIO_AFRL_AFSEL5_Pos) | (2 << GPIO_AFRL_AFSEL6_Pos) | (2 << GPIO_AFRL_AFSEL7_Pos);
	
	// TIM clock enable
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;

#if (ESC == DSHOT) // DMA driven timer for DShot600, 48MHz: 0:30, 1:60, T:80

	// Use TIM2 as master timer to control TIM3 and TIM4 and use its TIM2_UP DMA requests (whose DMA streams are not using USART2_TX request)
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	#if (DSHOT_RATE == 300)
		TIM2->PSC = 2;
	#else // DSHOT_RATE == 600
		TIM2->PSC = 1; // TIM2 is running at 96MHz instead of 48MHz
	#endif
	TIM2->ARR = 80;
	//TIM2->DIER = TIM_DIER_UDE; // Managed in motor function
	TIM2->CR2 = (1 << TIM_CR2_MMS_Pos); // TRGO = CEN

	// TIM3 config
	#if (DSHOT_RATE == 300)
		TIM3->PSC = 2;
	#else // DSHOT_RATE == 600
		TIM3->PSC = 1; // TIM3 is running at 96MHz instead of 48MHz
	#endif
	TIM3->ARR = 80;
	TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E; // Enable outputs
	TIM3->CCMR1 = (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE | (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE; // 6: PWM mode 1 (active when CNT < CCR) + Preload enabled
	TIM3->DCR = (3 << TIM_DCR_DBL_Pos) | (13 << TIM_DCR_DBA_Pos); // 4 DMA transfers starting at CCR1 address
	TIM3->SMCR = (5 << TIM_SMCR_SMS_Pos) | (1 << TIM_SMCR_TS_Pos); // Slave Gated mode on TIM2 TRGO
	TIM3->CR1 = TIM_CR1_CEN;

	// Pad DSHOT pulse with zeros, to force end of DSHOT transaction
	dshot[16*4+0] = 0;
	dshot[16*4+1] = 0;

	// DMA1 stream 1 chan 3: TIM2_UP
	DMA1_Stream1->CR = (3 << DMA_SxCR_CHSEL_Pos) | (2 << DMA_SxCR_PL_Pos) | DMA_SxCR_MINC  | (1 << DMA_SxCR_DIR_Pos)
		| (1 << DMA_SxCR_MSIZE_Pos) | (1 << DMA_SxCR_PSIZE_Pos) // 16-bit transfer for 16-bit CCRx register
		| (1 << DMA_SxCR_MBURST_Pos) | (1 << DMA_SxCR_PBURST_Pos); // INCR4 burst
	DMA1_Stream1->M0AR = (uint32_t)dshot;
	DMA1_Stream1->PAR = (uint32_t)&(TIM3->DMAR);
	DMA1_Stream1->FCR = DMA_SxFCR_DMDIS | (3 << DMA_SxFCR_FTH_Pos); // Use FIFO with threshold of 4
	
	// TIM4 config
	#if (DSHOT_RATE == 300)
		TIM4->PSC = 2;
	#else // DSHOT_RATE == 600
		TIM4->PSC = 1; // TIM4 is running at 96MHz instead of 48MHz
	#endif
	TIM4->ARR = 80;
	TIM4->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E; // Enable outputs
	TIM4->CCMR1 = (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE | (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE; // 6: PWM mode 1 (active when CNT < CCR) + Preload enabled
	TIM4->DCR = (3 << TIM_DCR_DBL_Pos) | (13 << TIM_DCR_DBA_Pos); // 4 DMA transfers starting at CCR1 address
	TIM4->SMCR = (5 << TIM_SMCR_SMS_Pos) | (1 << TIM_SMCR_TS_Pos); // Slave Gated mode on TIM2 TRGO
	TIM4->CR1 = TIM_CR1_CEN;

	// Pad DSHOT pulse with zeros, to force end of DSHOT transaction
	dshot2[16*4+0] = 0;
	dshot2[16*4+1] = 0;

	// DMA1 stream 7 chan 3: TIM2_UP
	DMA1_Stream7->CR = (3 << DMA_SxCR_CHSEL_Pos) | (2 << DMA_SxCR_PL_Pos) | DMA_SxCR_MINC  | (1 << DMA_SxCR_DIR_Pos)
		| (1 << DMA_SxCR_MSIZE_Pos) | (1 << DMA_SxCR_PSIZE_Pos) // 16-bit transfer for 16-bit CCRx register
		| (1 << DMA_SxCR_MBURST_Pos) | (1 << DMA_SxCR_PBURST_Pos); // INCR4 burst
	DMA1_Stream7->M0AR = (uint32_t)dshot2;
	DMA1_Stream7->PAR = (uint32_t)&(TIM4->DMAR);
	DMA1_Stream7->FCR = DMA_SxFCR_DMDIS | (3 << DMA_SxFCR_FTH_Pos); // Use FIFO with threshold of 4

#else // One-pulse mode for OneShot125 or PWM

	TIM3->CR1 = TIM_CR1_OPM;
	#if (ESC == PWM)
		TIM3->PSC = 48-1;
	#else
		TIM3->PSC = 6-1;
	#endif
	TIM3->ARR = SERVO_MAX*2 + 1;
	TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E; // Enable outputs
	TIM3->CCMR1 = (7 << TIM_CCMR1_OC1M_Pos) | (7 << TIM_CCMR1_OC2M_Pos); // 7: PWM mode 2 (active when CNT >= CCR)

	TIM4->CR1 = TIM_CR1_OPM;
	#if (ESC == PWM)
		TIM4->PSC = 48-1;
	#else
		TIM4->PSC = 6-1;
	#endif
	TIM4->ARR = SERVO_MAX*2 + 1;
	TIM4->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E; // Enable outputs
	TIM4->CCMR1 = (7 << TIM_CCMR1_OC1M_Pos) | (7 << TIM_CCMR1_OC2M_Pos); // 7: PWM mode 2 (active when CNT >= CCR)
#endif

	/* LED -----------------------*/

	// C13: Blue LED, need open-drain
	// C14: Green LED, need open-drain
	GPIOC->MODER |= GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0;
	GPIOC->OTYPER |= GPIO_OTYPER_OT_13 | GPIO_OTYPER_OT_14;
	GPIOC->BSRR = GPIO_BSRR_BS_13 | GPIO_BSRR_BS_14;

	/* Time in us ----------------------------------------------------------*/

	RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
	TIM9->PSC = 96-1; // 1us, TIM9 is running at 96MHz instead of 48MHz
	TIM9->ARR = 65535;
	TIM9->CR1 = TIM_CR1_CEN;
	
	/* VBAT ADC -----------------------------------------------------*/

	// B0 : VBAT, ADC1 chan 8
	// B1 : Current meter, ADC1 chan 9
	GPIOB->MODER |= GPIO_MODER_MODER0 | GPIO_MODER_MODER1;

	// ADC config
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // Clock enable
	//ADC1_COMMON->CCR = (0 << ADC_CCR_ADCPRE_Pos); // ADC clock prescaler (div from APB2 clock), 0=2, 1=4, 2=6, 3=8
	ADC1->CR1 = ADC_CR1_SCAN | ADC_CR1_JEOCIE;
	ADC1->JSQR = (1 << ADC_JSQR_JL_Pos) | (8 << ADC_JSQR_JSQ3_Pos) | (9 << ADC_JSQR_JSQ4_Pos); // 2 injected conversion of channel 8 and 9
	ADC1->SMPR2 = (2 << ADC_SMPR2_SMP8_Pos) | (2 << ADC_SMPR2_SMP9_Pos); // sampling time (in cycles), 0=3, 1=15, 2=28, 3=56
	NVIC_EnableIRQ(ADC_IRQn);
	NVIC_SetPriority(ADC_IRQn,0);

	// Enable
	ADC1->CR2 |= ADC_CR2_ADON;
	wait_ms(1);

	/* Beeper -----------------------------------------*/

#ifdef BEEPER

	// B2 : Beeper
	GPIOB->MODER |= GPIO_MODER_MODER2_0;
	GPIOB->BSRR = GPIO_BSRR_BR_2;

#endif

	/* OSD ----------------------------------------*/

	// B12: OSD (MAX7456) SPI2_NSS  (AF5), need pull-up
	// B13: OSD (MAX7456) SPI2_SCK  (AF5), CPOL=0 => need pull-down 
	// B14: OSD (MAX7456) SPI2_MISO (AF5)
	// B15: OSD (MAX7456) SPI2_MOSI (AF5)
	GPIOB->MODER |= GPIO_MODER_MODER12_1 | GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR12_0 | GPIO_PUPDR_PUPDR13_1;
	GPIOB->AFR[1] |= (5 << GPIO_AFRH_AFSEL12_Pos) | (5 << GPIO_AFRH_AFSEL13_Pos) | (5 << GPIO_AFRH_AFSEL14_Pos) | (5 << GPIO_AFRH_AFSEL15_Pos);

	// SPI config
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
	SPI2->CR1 = SPI_CR1_MSTR | (5 << SPI_CR1_BR_Pos); // SPI clock = clock APB2/64 = 48MHz/64 = 750 kHz
	SPI2->CR2 = SPI_CR2_SSOE | SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;

	// DMA1 stream 3 chan 0: SPI2_RX
	DMA1_Stream3->CR = (0 << DMA_SxCR_CHSEL_Pos) | (1 << DMA_SxCR_PL_Pos) | DMA_SxCR_MINC | DMA_SxCR_TCIE;
	DMA1_Stream3->M0AR = (uint32_t)spi2_rx_buffer;
	DMA1_Stream3->PAR = (uint32_t)&(SPI2->DR);
	NVIC_EnableIRQ(DMA1_Stream3_IRQn);
	NVIC_SetPriority(DMA1_Stream3_IRQn,0);

	// DMA1 stream 4 chan 0: SPI2_TX
	DMA1_Stream4->CR = (0 << DMA_SxCR_CHSEL_Pos) | (3 << DMA_SxCR_PL_Pos) | DMA_SxCR_MINC | (1 << DMA_SxCR_DIR_Pos);
	DMA1_Stream4->M0AR = (uint32_t)spi2_tx_buffer;
	DMA1_Stream4->PAR = (uint32_t)&(SPI2->DR);

	/* Smart Audio ---------------------------------------*/

#ifdef SMART_AUDIO

	// A2 : USART2_TX (AF7), pull-down to set audio line to GND
	GPIOA->MODER |= GPIO_MODER_MODER2_1;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR2_1;
	GPIOA->AFR[0] |= (7 << GPIO_AFRL_AFSEL2_Pos);

	// UART config
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	USART2->BRR = 9800; // 48MHz/4800bps = 10000 => 48MHz/4900bps = 9800 in order to match TBS unify board Xtal
	USART2->CR2 = 2 << USART_CR2_STOP_Pos; // 2 stop bits
	USART2->CR3 = USART_CR3_DMAT | USART_CR3_HDSEL; // HDSEL: single-wire half-duplex
	USART2->CR1 = USART_CR1_UE | USART_CR1_TCIE | USART_CR1_RXNEIE;
	NVIC_EnableIRQ(USART2_IRQn);
	NVIC_SetPriority(USART2_IRQn,0);

	// DMA1 stream 6 chan 4: USART2_TX
	DMA1_Stream6->CR = (4 << DMA_SxCR_CHSEL_Pos) | (3 << DMA_SxCR_PL_Pos) | DMA_SxCR_MINC | (1 << DMA_SxCR_DIR_Pos);
	DMA1_Stream6->M0AR = (uint32_t)uart2_tx_buffer;
	DMA1_Stream6->PAR = (uint32_t)&(USART2->DR);

#endif

}
