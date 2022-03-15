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
	GPIOA->ODR ^= GPIO_ODR_OD13;
}

void toggle_led2(_Bool en)
{
	if (en)
		GPIOA->ODR ^= GPIO_ODR_OD14;
	else
		GPIOA->BSRR = GPIO_BSRR_BS_14;
}

void toggle_beeper(_Bool en)
{
	if (en)
		GPIOC->ODR ^= GPIO_ODR_OD13;
	else
		GPIOC->BSRR = GPIO_BSRR_BR_13;
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

	// A0 : UART4 Tx (AF8)
	// A1 : UART4 Rx (AF8)
	// A2 : UART2 Tx (AF7)
	// A3 : UART2 Rx (AF7)

	// A5 : Gyro SPI1 CLK  (AF5)
	// A6 : Gyro SPI1 MISO (AF5)
	// A7 : Gyro SPI1 MOSI (AF5)
	// A8 : Geek LED
	// A9 : UART1 Tx (AF7), TIM1 CH2 (AF1)
	// A10: UART1 Rx (AF7), TIM1 CH3 (AF1)

	// A13: Green LED
	// A14: Blue LED
	// A15: Servo 5, TIM2 CH1 (AF1), SPI3 CSN (AF6)

	// B0 : Servo 3, TIM3 CH3 (AF2)
	// B1 : Servo 4, TIM3 CH4 (AF2)
	// B2 : Gyro CSN
	// B3 : Servo 6, TIM2 CH2 (AF1), SPI3 CLK (AF6)
	// B4 : Servo 1, TIM3 CH1 (AF2)
	// B5 : Servo 2, TIM3 CH2 (AF2)
	// B6 : Servo 7, TIM4 CH1 (AF2)
	// B7 : Servo 8, TIM4 CH2 (AF2)
	// B8 : Baro I2C1 SCL (AF4), external pull-up
	// B9 : Baro I2C1 SDA (AF4), external pull-up
	// B10: UART3 Tx (AF7), I2C2 SCL (AF4), TIM2 CH3 (AF1)
	// B11: UART3 Rx (AF7), I2C2 SDA (AF4), TIM2 CH4 (AF1)
	// B12: OSD SPI2 CSN  (AF5)
	// B13: OSD SPI2 CLK  (AF5)
	// B14: OSD SPI2 MISO (AF5)
	// B15: OSD SPI2 MOSI (AF5)

	// C4 : Gyro interrupt
	
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

	// A4 : SPI1 CS   (AF5), need pull-up
	// A5 : SPI1 CLK  (AF5), CPOL=1 => need pull-up 
	// A6 : SPI1 MISO (AF5)
	// A7 : SPI1 MOSI (AF5)
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

	// A10: UART1 Rx (AF7), pull-up for IDLE
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

	// Get first vbat value
	trig_vbat_meas();
	while (!flag_vbat)
		__WFI();
	flag_vbat = 0;

#endif

	/* Beeper -----------------------------------------*/

#ifdef BEEPER

	// C13 : Beeper
	GPIOC->MODER |= GPIO_MODER_MODER13_0;
	GPIOC->BSRR = GPIO_BSRR_BR_13;

#endif


}
