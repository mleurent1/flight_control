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
volatile uint8_t spi3_rx_buffer[7];
volatile uint8_t spi3_tx_buffer[7];
#if (ESC == DSHOT)
	volatile uint8_t dshot[17*4];
#endif
volatile _Bool sensor_busy;
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

inline __attribute__((always_inline)) void rf_spi_dma_enable(uint8_t size)
{
	DMA1_Stream0->NDTR = size + 1;
	DMA1_Stream7->NDTR = size + 1;
	DMA1_Stream0->CR |= DMA_SxCR_EN;
	DMA1_Stream7->CR |= DMA_SxCR_EN;
	SPI3->CR1 |= SPI_CR1_SPE;
}

inline __attribute__((always_inline)) void rf_spi_dma_disable()
{
	DMA1->LIFCR = DMA_CLEAR_ALL_FLAGS_0;
	DMA1->HIFCR = DMA_CLEAR_ALL_FLAGS_7;
	DMA1_Stream0->CR &= ~DMA_SxCR_EN;
	DMA1_Stream7->CR &= ~DMA_SxCR_EN;
	while ((DMA1_Stream0->CR & DMA_SxCR_EN) && (DMA1_Stream7->CR & DMA_SxCR_EN)) {}
	SPI3->CR1 &= ~SPI_CR1_SPE;
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

void rf_write(uint8_t addr, uint8_t * data, uint8_t size)
{
	spi3_tx_buffer[0] = 0x80 | (addr & 0x7F);
	memcpy((uint8_t *)&spi3_tx_buffer[1], data, size);
	rf_spi_dma_enable(size);
}

void rf_read(uint8_t addr, uint8_t size)
{
	spi3_tx_buffer[0] = addr & 0x7F;
	rf_spi_dma_enable(size);
}

void radio_sync()
{
	radio_uart_dma_disable();
	USART1->CR1 |= USART_CR1_RE | USART_CR1_IDLEIE; // Enable UART with IDLE line detection
}

void set_motors(uint16_t * motor_raw, _Bool * motor_telemetry)
{
#if (ESC == DSHOT)
	int i;
	uint8_t motor1_dshot[16];
	uint8_t motor2_dshot[16];
	uint8_t motor3_dshot[16];
	uint8_t motor4_dshot[16];

	if (DMA1_Stream1->NDTR == 0) {
		TIM2->DIER = 0;
		TIM2->CR1 = 0;
		TIM2->CNT = 60;
		DMA1->LIFCR = DMA_CLEAR_ALL_FLAGS_1;
		DMA1_Stream1->CR &= ~DMA_SxCR_EN;
		
		dshot_encode(&motor_raw[0], motor1_dshot, motor_telemetry[0]);
		dshot_encode(&motor_raw[1], motor2_dshot, motor_telemetry[1]);
		dshot_encode(&motor_raw[2], motor3_dshot, motor_telemetry[2]);
		dshot_encode(&motor_raw[3], motor4_dshot, motor_telemetry[3]);
		for (i=0; i<16; i++) {
			dshot[i*4+0] = motor1_dshot[i];
			dshot[i*4+1] = motor2_dshot[i];
			dshot[i*4+2] = motor3_dshot[i];
			dshot[i*4+3] = motor4_dshot[i];
		}

		DMA1_Stream1->NDTR = 17*4;
		DMA1_Stream1->CR |= DMA_SxCR_EN;
		TIM2->DIER = TIM_DIER_UDE;		
		TIM2->CR1 = TIM_CR1_CEN;
	}
#else
	TIM2->CCR1 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - (uint32_t)motor_raw[0];
	TIM2->CCR2 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - (uint32_t)motor_raw[1];
	TIM2->CCR3 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - (uint32_t)motor_raw[2];
	TIM2->CCR4 = SERVO_MAX*2 + 1 - SERVO_MIN*2 - (uint32_t)motor_raw[3];
	TIM2->CR1 |= TIM_CR1_CEN;
#endif
}

void toggle_led()
{
	GPIOB->ODR ^= GPIO_ODR_OD5;
}

void toggle_led2(_Bool en)
{
	if (en)
		GPIOB->ODR ^= GPIO_ODR_OD4;
	else
		GPIOB->BSRR = GPIO_BSRR_BS_4;
}

void toggle_beeper(_Bool en)
{
	if (en)
		GPIOB->ODR ^= GPIO_ODR_OD0;
	else
		GPIOB->BSRR = GPIO_BSRR_BR_0;
}

void host_send(uint8_t * data, uint8_t size)
{
	USBD_CDC_SetTxBuffer(&USBD_device_handler, data, size);
	USBD_CDC_TransmitPacket(&USBD_device_handler);
}

int32_t get_t_us(void)
{
	return (int32_t)TIM6->CNT;
}

void trig_vbat_meas(void)
{
	ADC1->CR2 |= ADC_CR2_JSWSTART;
}

/* --------------------------------------------------------------------------------
 Interrupt routines
--------------------------------------------------------------------------------- */

void SysTick_Handler()
{
	t_ms++;
	flag_time = 1;
}

/* RF Rx Done ----------------------------*/

void EXTI0_IRQHandler()
{
	EXTI->PR = EXTI_PR_PR0; // Clear pending request
	flag_rf_rxtx_done = 1;
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

/* RF SPI IRQ ---------------------------------------*/

void SPI3_IRQHandler()
{
	rf_spi_dma_disable();
	rf_error_count++;
}

/* DMA IRQ of RF Rx SPI --------------------------------*/

void DMA1_Stream0_IRQHandler()
{
	rf_spi_dma_disable();

	if (REG_CTRL__RF_HOST_CTRL == 1) // Send SPI read data to host
		host_send((uint8_t*)&spi3_rx_buffer[1],1);
	else
		flag_rf = 1;
}

/* DMA IRQ of RF Rx SPI ------------------------*/

void DMA1_Stream7_IRQHandler()
{
	rf_spi_dma_disable();
	rf_error_count++;
}

/* Radio UART IRQ ------------------------------*/

void USART1_IRQHandler()
{
	if (USART1->SR & USART_SR_IDLE) {
		USART1->DR; // Clear status flags
		USART1->CR1 &= ~USART_CR1_IDLEIE; // Disable idle line detection
		radio_uart_dma_enable(sizeof(radio_frame));
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
	ADC1->SR = 0; // Clear IRQ

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
	
	// A0 : Servo 6, TIM2 CH1 (AF1), TIM5 CH1 (AF2)
	// A1 : Servo 5, TIM2 CH2 (AF1), TIM5 CH2 (AF2)
	// A2 : Servo 4, TIM2 CH3 (AF1), TIM5 CH3 (AF2)
	// A3 : Servo 3, TIM2 CH4 (AF1), TIM5 CH4 (AF2)
	// A4 : Gyro SPI1 CSN  (AF5), external pull-up
	// A5 : Gyro SPI1 CLK  (AF5)
	// A6 : Gyro SPI1 MISO (AF5)
	// A7 : Gyro SPI1 MOSI (AF5)

	// A9 : Main port 3, UART1 Tx (AF7), TIM1 CH2 (AF1)
	// A10: Main port 4, UART1 Rx (AF7), TIM1 CH3 (AF1)
	// A11: USB_DM (AF10)
	// A12: USB_DP (AF10)

	// A15: RF SPI3 CSN (AF6), external pull-up
	
	// B0 : Servo 1, TIM3 CH3 (AF2)
	// B1 : Servo 2, TIM3 CH4 (AF2)

	// B4 : Orange LED, need open-drain
	// B5 : Blue LED, need open-drain

	// B7 : Compass DRDY
	// B8 : Compass + Baro I2C1 SCL (AF4), external pull-up
	// B9 : Compass + Baro I2C1 SDA (AF4), external pull-up
	// B10: Flexi port 3, UART3 Tx (AF7), I2C2 SCL (AF4), TIM2 CH3 (AF1)
	// B11: Flexi port 4, UART3 Rx (AF7), I2C2 SDA (AF4), TIM2 CH4 (AF1)
	// B12: Input port 3, SPI2 CSN  (AF5)
	// B13: Input port 4, SPI2 CLK  (AF5)
	// B14: Input port 5, SPI2 MISO (AF5)
	// B15: Input port 6, SPI2 MOSI (AF5)
	
	// C0 : SBUS invert
	// C1 : Sense port 3: current, ADC123_IN11
	// C2 : Sense port 4: voltage, ADC123_IN12

	// C4 : Gyro interrupt
	// C5 : VBUS sense
	// C6 : Input port 7, TIM3 CH1 (AF2), TIM8 CH1 (AF3), UART6 Tx (AF8)
	// C7 : Input port 8, TIM3 CH2 (AF2), TIM8 CH2 (AF3), UART6 Rx (AF8)
	// C8 : Input port 9, TIM3 CH3 (AF2), TIM8 CH3 (AF3)
	// C9 : Input port10, TIM3 CH4 (AF2), TIM8 CH4 (AF3)
	// C10: RF SPI3 CLK  (AF6)
	// C11: RF SPI3 MISO (AF6)
	// C12: RF SPI3 MOSI (AF6)

	// D2 : RF reset, external pull-up

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

	// Init
	usb_init();

	/* Gyro/accel sensor ----------------------------------------------------*/

	// A4 : SPI1 CS   (AF5), external pull-up => need open-drain 
	// A5 : SPI1 CLK  (AF5), CPOL=1 => need pull-up 
	// A6 : SPI1 MISO (AF5)
	// A7 : SPI1 MOSI (AF5)
	GPIOA->MODER |= GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4_1 | GPIO_OSPEEDER_OSPEEDR5_1 | GPIO_OSPEEDER_OSPEEDR7_1;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR5_0;
	GPIOA->OTYPER |= GPIO_OTYPER_OT_4;
	GPIOA->AFR[0] |= (5 << GPIO_AFRL_AFSEL4_Pos) | (5 << GPIO_AFRL_AFSEL5_Pos) | (5 << GPIO_AFRL_AFSEL6_Pos) | (5 << GPIO_AFRL_AFSEL7_Pos);
	
	// SPI config
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	SPI1->CR1 = SPI_CR1_MSTR | (5 << SPI_CR1_BR_Pos) | SPI_CR1_CPOL | SPI_CR1_CPHA; // SPI clock = clock APB2/64 = 48MHz/64 = 750 kHz
	SPI1->CR2 = SPI_CR2_SSOE | SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN | SPI_CR2_ERRIE;
	NVIC_EnableIRQ(SPI1_IRQn);
	NVIC_SetPriority(SPI1_IRQn,0);

	// C4 : Gyro interrupt
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

	// A10: Main port 4, UART1 Rx (AF7), pull-up for IDLE
	// C0 : SBUS invert
	GPIOA->MODER |= GPIO_MODER_MODER10_1;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR10_0;
	GPIOA->AFR[1] |= 7 << GPIO_AFRH_AFSEL10_Pos;
	GPIOC->MODER |= GPIO_MODER_MODER0_0;

	// UART config
	GPIOC->BSRR = GPIO_BSRR_BR_0; // Do not invert Rx
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

	// A0 : Servo 6, TIM2 CH1 (AF1), TIM5 CH1 (AF2)
	// A1 : Servo 5, TIM2 CH2 (AF1), TIM5 CH2 (AF2)
	// A2 : Servo 4, TIM2 CH3 (AF1), TIM5 CH3 (AF2)
	// A3 : Servo 3, TIM2 CH4 (AF1), TIM5 CH4 (AF2)
	GPIOA->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1 | GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0_1 | GPIO_OSPEEDER_OSPEEDR1_1 | GPIO_OSPEEDER_OSPEEDR2_1 | GPIO_OSPEEDER_OSPEEDR3_1;
	GPIOA->AFR[0] |= (1 << GPIO_AFRL_AFSEL0_Pos) | (1 << GPIO_AFRL_AFSEL1_Pos) | (1 << GPIO_AFRL_AFSEL2_Pos) | (1 << GPIO_AFRL_AFSEL3_Pos);

	// TIM clock enable
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

#if (ESC == DSHOT)
	// DMA driven timer for DShot600, 48MHz: 0:30, 1:60, T:80
	#if (DSHOT_RATE == 300)
		TIM2->PSC = 1;
	#else // DSHOT_RATE == 600
		TIM2->PSC = 0;
	#endif
	TIM2->ARR = 80;
	//TIM2->DIER = TIM_DIER_UDE; // Managed in motor function
	TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
	TIM2->CCMR1 = (6 << TIM_CCMR2_OC1M_Pos) | TIM_CCMR1_OC1PE | (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
	TIM2->CCMR2 = (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE | (6 << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
	TIM2->DCR = (1 << TIM_DCR_DBL_Pos) | (15 << TIM_DCR_DBA_Pos);

	// Pad DSHOT pulse with zeros
	dshot[16*2+0] = 0;
	dshot[16*2+1] = 0;
	dshot[16*2+0] = 0;
	dshot[16*2+1] = 0;

	// DMA TIM2_UP
	DMA1_Stream1->CR = (5 << DMA_SxCR_CHSEL_Pos) | (2 << DMA_SxCR_PL_Pos) | (2 << DMA_SxCR_MSIZE_Pos) | (2 << DMA_SxCR_PSIZE_Pos) | DMA_SxCR_MINC | (1 << DMA_SxCR_DIR_Pos);
	DMA1_Stream1->M0AR = (uint32_t)dshot;
	DMA1_Stream1->PAR = (uint32_t)&(TIM2->DMAR);

#else
	// One-pulse mode for OneShot125
	TIM2->CR1 = TIM_CR1_OPM;
	#if (ESC == PWM)
		TIM2->PSC = 24-1;
	#else
		TIM2->PSC = 3-1;
	#endif
	TIM2->ARR = SERVO_MAX*2 + 1;
	TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
	TIM2->CCMR1 = (7 << TIM_CCMR1_OC1M_Pos) | (7 << TIM_CCMR1_OC2M_Pos);
	TIM2->CCMR2 = (7 << TIM_CCMR2_OC3M_Pos) | (7 << TIM_CCMR2_OC4M_Pos);
#endif

	/* LED -----------------------*/

	// B4 : Orange LED, need open-drain
	// B5 : Blue LED, need open-drain
	GPIOB->MODER |= GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0;
	GPIOB->OTYPER |= GPIO_OTYPER_OT_4 | GPIO_OTYPER_OT_5;
	GPIOB->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;

	/* Time in us ----------------------------------------------------------*/

	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	TIM6->PSC = 48-1; // 1us
	TIM6->ARR = 65535;
	TIM6->CR1 = TIM_CR1_CEN;
	
	/* VBAT ADC -----------------------------------------------------*/

#ifdef VBAT

	// C1 : Sense port 3: current, ADC123_IN11
	// C2 : Sense port 4: voltage, ADC123_IN12
	GPIOC->MODER |= GPIO_MODER_MODER1 | GPIO_MODER_MODER2;

	// ADC config
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // Clock enable
	//ADC1_COMMON->CCR = (0 << ADC_CCR_ADCPRE_Pos); // ADC clock prescaler (div from APB2 clock), 0=2, 1=4, 2=6, 3=8
	ADC1->CR1 = ADC_CR1_SCAN;
	ADC1->JSQR = (1 << ADC_JSQR_JL_Pos) | (11 << ADC_JSQR_JSQ3_Pos) | (12 << ADC_JSQR_JSQ4_Pos); // 2 injected conversion of channel 11 and 12
	ADC1->SMPR2 = (2 << ADC_SMPR2_SMP8_Pos) | (2 << ADC_SMPR2_SMP9_Pos); // sampling time (in cycles), 0=3, 1=15, 2=28, 3=56
	NVIC_EnableIRQ(ADC1_2_IRQn);
	NVIC_SetPriority(ADC1_2_IRQn,0);

	// Enable
	ADC1->CR2 |= ADC_CR2_ADON;
	wait_ms(1);

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

	/* RF ------------------------*/
	
#ifdef RF

	// A0 : Servo 6, used as SX1276 DIO[0]
	// A15: SPI3 CS, AF6, need open-drain (external pull-up)
	// C10: SPI3 CLK, AF6, need pull-down (CPOL = 0)
	// C11: SPI3 MISO, AF6
	// C12: SPI3 MOSI, AF6
	// D2 : RF reset, need open-drain (external pull-up)
	GPIOA->MODER |= GPIO_MODER_MODER15_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR15_1;
	GPIOA->OTYPER |= GPIO_OTYPER_OT_15;
	GPIOA->AFR[1] |= 6 << GPIO_AFRH_AFSEL15_Pos;
	GPIOC->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1 | GPIO_MODER_MODER12_1;
	GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10_1 | GPIO_OSPEEDER_OSPEEDR12_1;
	GPIOC->PUPDR |= GPIO_PUPDR_PUPDR10_1;
	GPIOC->AFR[1] |= (6 << GPIO_AFRH_AFSEL10_Pos) | (6 << GPIO_AFRH_AFSEL11_Pos) | (6 << GPIO_AFRH_AFSEL12_Pos);
	GPIOD->MODER |= GPIO_MODER_MODER2_0;
	GPIOD->OTYPER |= GPIO_OTYPER_OT_2;

	// SPI config
	RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
	SPI3->CR1 = SPI_CR1_MSTR | (1 << SPI_CR1_BR_Pos); // SPI clock = clock APB1/4 = 24MHz/4 = 6 MHz
	SPI3->CR2 = SPI_CR2_SSOE | SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN | SPI_CR2_ERRIE;
	NVIC_EnableIRQ(SPI3_IRQn);
	NVIC_SetPriority(SPI3_IRQn,0);
	
	// Init
	GPIOD->BSRR = GPIO_BSRR_BR_2; // Reset
	wait_ms(1);
	GPIOD->BSRR = GPIO_BSRR_BS_2;
	wait_ms(1);
	sx1276_init();
	EXTI->IMR |= EXTI_IMR_MR0; // Enable external interrupts now

#endif

}
