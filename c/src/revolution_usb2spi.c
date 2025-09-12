#include "board.h"
#include "stm32f4xx.h" // CMSIS
#include "usb2spi.h" // flags
#include "usb.h" // usb_init()
#include <string.h> // memcpy()
#include <stdbool.h>

/* Private defines ------------------------------------*/

volatile uint8_t spi_request_size;

/* Private macros ------------------------------------------*/

#define DMA_CLEAR_ALL_FLAGS_0 (DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0)
#define DMA_CLEAR_ALL_FLAGS_1 (DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1)
#define DMA_CLEAR_ALL_FLAGS_2 (DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTEIF2 | DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CFEIF2)
#define DMA_CLEAR_ALL_FLAGS_3 (DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3 | DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3)
#define DMA_CLEAR_ALL_FLAGS_4 (DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4 | DMA_HIFCR_CTEIF4 | DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CFEIF4)
#define DMA_CLEAR_ALL_FLAGS_5 (DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5)
#define DMA_CLEAR_ALL_FLAGS_6 (DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6)
#define DMA_CLEAR_ALL_FLAGS_7 (DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CFEIF7)

/* Private types --------------------------------------*/

/* Global variables --------------------------------------*/

/* Private functions ------------------------------------------------*/

void spi_dma_enable(uint8_t size)
{
	DMA1_Stream3->NDTR = size;
	DMA1_Stream4->NDTR = size;
	DMA1_Stream3->CR |= DMA_SxCR_EN;
	DMA1_Stream4->CR |= DMA_SxCR_EN;
	SPI2->CR1 |= SPI_CR1_SPE;
	spi_request_size = size;
}

inline __attribute__((always_inline)) void spi_dma_disable()
{
	DMA1->LIFCR = DMA_CLEAR_ALL_FLAGS_3;
	DMA1->HIFCR = DMA_CLEAR_ALL_FLAGS_4;
	DMA1_Stream3->CR &= ~DMA_SxCR_EN;
	DMA1_Stream4->CR &= ~DMA_SxCR_EN;
	while ((DMA1_Stream3->CR & DMA_SxCR_EN) && (DMA1_Stream4->CR & DMA_SxCR_EN)) {}
	SPI2->CR1 &= ~SPI_CR1_SPE;
}

/* Public functions ---------------------------------------*/

uint32_t HAL_RCC_GetHCLKFreq(void)
{
  return SystemCoreClock;
}

void toggle_led()
{
	GPIOB->ODR ^= GPIO_ODR_OD5;
}

void toggle_led2(bool en)
{
	if (en)
		GPIOB->BSRR = GPIO_BSRR_BR_4;
	else
		GPIOB->BSRR = GPIO_BSRR_BS_4;
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

uint8_t get_dios(void)
{
	return (GPIOC->IDR & (GPIO_IDR_ID6 | GPIO_IDR_ID7 | GPIO_IDR_ID8)) >> GPIO_IDR_ID6_Pos;
}

void set_dios(uint8_t val)
{
	if (val & 0x01)
		GPIOC->BSRR = GPIO_BSRR_BS_6;
	else
		GPIOC->BSRR = GPIO_BSRR_BR_6;
	if (val & 0x02)
		GPIOC->BSRR = GPIO_BSRR_BS_7;
	else
		GPIOC->BSRR = GPIO_BSRR_BR_7;
	if (val & 0x04)
		GPIOC->BSRR = GPIO_BSRR_BS_8;
	else
		GPIOC->BSRR = GPIO_BSRR_BR_8;
	if (val & 0x08)
		GPIOA->BSRR = GPIO_BSRR_BS_0;
	else
		GPIOA->BSRR = GPIO_BSRR_BR_0;
}

/* --------------------------------------------------------------------------------
 Interrupt routines
--------------------------------------------------------------------------------- */

void SysTick_Handler()
{
	t_ms++;
	flag_time = 1;
}

/* Sensor SPI IRQ ------------------------------*/

void SPI2_IRQHandler()
{
	if (SPI2->SR & (SPI_SR_MODF | SPI_SR_OVR)) {
		spi_dma_disable();
		SPI2->SR; // Read SR to clear flags
	}
}

/* DMA IRQ of sensor Rx SPI ----------------------*/

void DMA1_Stream3_IRQHandler()
{
	spi_dma_disable();
	spi_bytes_avail = spi_request_size;
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

	/* SPI ----------------------------------------------------*/

	// B12: SPI2 CSN  (AF5) (Input port 3), need pull-up
	// B13: SPI2 CLK  (AF5) (Input port 4), need pull-down
	// B14: SPI2 MISO (AF5) (Input port 5)
	// B15: SPI2 MOSI (AF5) (Input port 6)
	GPIOB->MODER |= GPIO_MODER_MODER12_1 | GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR12_1 | GPIO_OSPEEDER_OSPEEDR13_1 | GPIO_OSPEEDER_OSPEEDR15_1;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR12_0 | GPIO_PUPDR_PUPDR13_1;
	GPIOB->AFR[1] |= (5 << GPIO_AFRL_AFSEL4_Pos) | (5 << GPIO_AFRL_AFSEL5_Pos) | (5 << GPIO_AFRL_AFSEL6_Pos) | (5 << GPIO_AFRL_AFSEL7_Pos);
	
	// SPI config
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
	SPI2->CR1 = SPI_CR1_MSTR | (2 << SPI_CR1_BR_Pos); // SPI clock = clock APB1/8 = 24MHz/8 = 3 MHz
	SPI2->CR2 = SPI_CR2_SSOE | SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN | SPI_CR2_ERRIE;
	NVIC_EnableIRQ(SPI2_IRQn);
	NVIC_SetPriority(SPI2_IRQn,0);

	// DMA SPI2 Rx
	DMA1_Stream3->CR = (0 << DMA_SxCR_CHSEL_Pos) | (1 << DMA_SxCR_PL_Pos) | DMA_SxCR_MINC | DMA_SxCR_TCIE;
	DMA1_Stream3->M0AR = (uint32_t)host_tx_buffer;
	DMA1_Stream3->PAR = (uint32_t)&(SPI2->DR);
	NVIC_EnableIRQ(DMA1_Stream3_IRQn);
	NVIC_SetPriority(DMA1_Stream3_IRQn,0);

	// DMA SPI2 Tx
	DMA1_Stream4->CR = (0 << DMA_SxCR_CHSEL_Pos) | (3 << DMA_SxCR_PL_Pos) | DMA_SxCR_MINC | (1 << DMA_SxCR_DIR_Pos);
	DMA1_Stream4->M0AR = (uint32_t)rx_buffer;
	DMA1_Stream4->PAR = (uint32_t)&(SPI2->DR);

	/* DIOS ----------------------*/

	// C8 : RESET (Input port 9)
	// A0 : TRIG (Servo 1)
	GPIOC->MODER |= GPIO_MODER_MODER8_0;
	GPIOA->MODER |= GPIO_MODER_MODER0_0;

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
}
