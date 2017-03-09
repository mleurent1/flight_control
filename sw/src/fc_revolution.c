/* Includes ----------------------*/

#include "stm32f4xx.h"
#include "fc.h"

/* Private Definitions --------------------*/

#define FLAG__SENSOR 0x01
#define FLAG__COMMAND 0x02
#define FLAG__DEBUG 0x04
#define FLAG__REG 0x08
#define FLAG__SPI_HOST 0x10

/* Private variables --------------*/

uint8_t FLAG;

uint32_t tick;

USBD_HandleTypeDef USBD_device_handler;

volatile uint8_t spi1_rx_buffer[16];
volatile uint8_t spi1_tx_buffer[16];

uint16_t sensor_error_count;

/* Private macros ---------------------------------------------------*/

#define SPI1_DMA_EN(size) \
	DMA2_Stream0->NDTR = size; \
	DMA2_Stream3->NDTR = size; \
	DMA2_Stream0->CR |= DMA_SxCR_EN; \
	DMA2_Stream3->CR |= DMA_SxCR_EN; \
	SPI1->CR1 |= SPI_CR1_SPE

#define USB_TX(size) \
	USBD_CDC_SetTxBuffer(&USBD_device_handler, USBD_CDC_IF_tx_buffer, size); \
	USBD_CDC_TransmitPacket(&USBD_device_handler);

/* Private function declarations -----------------------------------*/

void uint32_to_float(uint32_t* b, float* f);
void float_to_uint32(float* f, uint32_t* b);
void wait_ms(uint32_t t);

/* Public functions ---------------------------------------------*/

void HAL_Delay(__IO uint32_t Delay)
{
	wait_ms(Delay);
}

void HostCommand(uint8_t* buffer)
{
	uint8_t cmd;
	uint8_t addr;
	uint32_t val;
	
	cmd = buffer[0];
	addr = buffer[1];
	
	switch (cmd)
	{
		case 0: // REG read
		{
			if (reg_properties[addr].is_float)
				float_to_uint32(&regf[addr], &val);
			else
				val = reg[addr];
			USBD_CDC_IF_tx_buffer[0] = (uint8_t)((val >> 24) & 0xFF);
			USBD_CDC_IF_tx_buffer[1] = (uint8_t)((val >> 16) & 0xFF);
			USBD_CDC_IF_tx_buffer[2] = (uint8_t)((val >>  8) & 0xFF);
			USBD_CDC_IF_tx_buffer[3] = (uint8_t)((val >>  0) & 0xFF);
			USB_TX(4)
			break;
		}
		case 1: // REG write
		{
			if (!reg_properties[addr].read_only)
			{
				val = ((uint32_t)buffer[2] << 24) | ((uint32_t)buffer[3] << 16) | ((uint32_t)buffer[4] << 8) | (uint32_t)buffer[5];
				if (reg_properties[addr].is_float)
					uint32_to_float(&val, &regf[addr]);
				else
					reg[addr] = val;
				FLAG |= FLAG__REG;
			}
			break;
		}
		case 2: // SPI read to MPU
		{
			FLAG |= FLAG__SPI_HOST;
			spi1_tx_buffer[0] = 0x80 | (addr & 0x7F);
			SPI1_DMA_EN(2);
			break;
		}
		case 3: // SPI write to MPU
		{
			spi1_tx_buffer[0] = addr & 0x7F;
			spi1_tx_buffer[1] = buffer[5];
			SPI1_DMA_EN(2);
			break;
		}
	}
}

/* Private functions ---------------------------------------------*/

void wait_ms(uint32_t t)
{
	uint32_t next_tick;
	next_tick = tick + t;
	while (tick != next_tick)
		__WFI();
}

void SpiWrite(uint8_t addr, uint8_t data)
{
	spi1_tx_buffer[0] = addr & 0x7F;
	spi1_tx_buffer[1] = data;
	SPI1_DMA_EN(2);
	wait_ms(1);
}

uint8_t SpiRead(uint8_t addr)
{
	spi1_tx_buffer[0] = 0x80 | (addr & 0x7F);
	SPI1_DMA_EN(2);
	wait_ms(1);
	return spi1_rx_buffer[1];
}

void uint32_to_float(uint32_t* b, float* f)
{
	union {
		float f;
		uint32_t b;
	} f2b;
	f2b.b = *b;
	*f = f2b.f;
}

void float_to_uint32(float* f, uint32_t* b)
{
	union {
		float f;
		uint32_t b;
	} f2b;
	f2b.f = *f;
	*b = f2b.b;
}

/* Interrupt routines ---------------------------------------*/

void SysTick_Handler()
{
	tick++;
}

void OTG_FS_IRQHandler(void)
{
	HAL_PCD_IRQHandler(&PCD_handler);
}

void DMA2_Stream0_IRQHandler()
{
	_Bool error = 0;
	
	// Check DMA transfer error
	if (DMA2->LISR & (DMA_LISR_TEIF0 | DMA_LISR_TEIF3))
		error = 1;
	
	// Disable DMA
	DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 |DMA_LIFCR_CDMEIF0 |DMA_LIFCR_CFEIF0; // clear all flags
	DMA2->LIFCR = DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3 |DMA_LIFCR_CDMEIF3 |DMA_LIFCR_CFEIF3;
	DMA2_Stream0->CR &= ~DMA_SxCR_EN;
	DMA2_Stream3->CR &= ~DMA_SxCR_EN;
	
	// Check SPI error
	if ((SPI1->SR & SPI_SR_OVR) || (SPI1->SR & SPI_SR_MODF))
	{
		error = 1;
		SPI1->DR;
		SPI1->SR;
	}
	
	// Disable SPI
	while ((SPI1->SR & SPI_SR_TXE) == 0) {}
	while (SPI1->SR & SPI_SR_BSY) {}
	SPI1->CR1 &= ~SPI_CR1_SPE;
	
	if (error)
	{
		sensor_error_count++;
		REG_ERROR &= 0xFFFF0000;
		REG_ERROR |= (uint32_t)sensor_error_count & 0x0000FFFF;
	}
	else if (FLAG & FLAG__SPI_HOST)
	{
		FLAG &= ~FLAG__SPI_HOST;
		USBD_CDC_IF_tx_buffer[0] = spi1_rx_buffer[1];
		USB_TX(1)
	}
}

/*----------------------------------------------------------
-----------------------------------------------------------*/

int main()
{
	int i;
	
	uint32_t reg_default;
	
	/* Variable init --------------------*/
	
	/* Register init ---------------------------*/
	
	for(i=0; i<NB_REG; i++)
	{
		reg_default = reg_properties[i].dflt;
		if (reg_properties[i].is_float)
			uint32_to_float(&reg_default, &regf[i]);
		else
			reg[i] = reg_default;
	}
	
	/* Clock enable -------------------------------------*/
	
	// GPIO clock enable 
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
	// SPI clock enable
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	// DMA clock enable
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
	// USB clock enable
	RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
	
	/* GPIO ---------------------------------------------*/
	
	// MODER: 00:IN, 01:OUT, 10:AF, 11:analog
	// PUPDR: 00:float 01:PU, 10:PD
	// OTYPER: 0:PP, 1:OD
	// OSPEEDR: 00:low 4MHz, 01:mid 25MHz, 10:high 50MHz, 11:very high 100MHz
	
	// Reset non-zero registers
	GPIOA->MODER = 0;
	GPIOA->PUPDR = 0;
	GPIOA->OSPEEDR = 0;
	GPIOB->MODER = 0;
	GPIOB->PUPDR = 0;
	GPIOB->OSPEEDR = 0;
	
	// A4 : SPI1 CS, AF5, need open-drain (external pull-up)
	// A5 : SPI1 CLK, AF5, need pull-up
	// A6 : SPI1 MISO, AF5 
	// A7 : SPI1 MOSI, AF5
	GPIOA->MODER |= GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4 | GPIO_OSPEEDER_OSPEEDR5 | GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR5_0;
	GPIOA->OTYPER |= GPIO_OTYPER_OT_4;
	GPIOA->AFR[0] |= (5 << GPIO_AFRL_AFSEL4_Pos) | (5 << GPIO_AFRL_AFSEL5_Pos) | (5 << GPIO_AFRL_AFSEL6_Pos) | (5 << GPIO_AFRL_AFSEL7_Pos);
	// A11: USB_DM, AF10
	// A12: USB_DP, AF10
	GPIOA->MODER |= GPIO_MODER_MODER11_1 | GPIO_MODER_MODER12_1;
	GPIOA->AFR[1] |= (10 << GPIO_AFRH_AFSEL11_Pos) | (10 << GPIO_AFRH_AFSEL12_Pos);
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR11 | GPIO_OSPEEDER_OSPEEDR12;
	// B5 : Blue LED, need open-drain (external pull-up)
	GPIOB->MODER |= GPIO_MODER_MODER5_0;
	GPIOB->OTYPER |= GPIO_OTYPER_OT_5;
	
	/* DMA -----------------------------------------------------*/
	
	// DMA[y]_Stream[x]->CR = (3 << DMA_SxCR_CHSEL_Pos) | (1 << DMA_SxCR_PL_Pos) | (1 << DMA_SxCR_MSIZE_Pos) | (1 << DMA_SxCR_PSIZE_Pos) | DMA_SxCR_MINC | DMA_SxCR_PINC | (1 << DMA_SxCR_DIR_Pos) | DMA_SxCR_TCIE;

	// SPI1 Rx
	DMA2_Stream0->CR = (3 << DMA_SxCR_CHSEL_Pos) | (1 << DMA_SxCR_PL_Pos) | DMA_SxCR_MINC | DMA_SxCR_TCIE;
	DMA2_Stream0->M0AR = (uint32_t)spi1_rx_buffer;
	DMA2_Stream0->PAR = (uint32_t)&(SPI1->DR);

	// SPI1 Tx
	DMA2_Stream3->CR = (3 << DMA_SxCR_CHSEL_Pos) | (3 << DMA_SxCR_PL_Pos) | DMA_SxCR_MINC | (1 << DMA_SxCR_DIR_Pos);
	DMA2_Stream3->M0AR = (uint32_t)spi1_tx_buffer;
	DMA2_Stream3->PAR = (uint32_t)&(SPI1->DR);
	
	/* SPI ----------------------------------------*/
	
	SPI1->CR1 = SPI_CR1_MSTR | (6 << SPI_CR1_BR_Pos) | SPI_CR1_CPOL | SPI_CR1_CPHA; // SPI clock = clock APB1/128 = 84MHz/128 = 656 kHz
	SPI1->CR2 = SPI_CR2_SSOE | SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;
	
	/* Interrupts --------------------------------------*/
	
	NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	NVIC_EnableIRQ(OTG_FS_IRQn);
	
	NVIC_SetPriority(DMA2_Stream0_IRQn,1);
	NVIC_SetPriority(OTG_FS_IRQn,2);
	
	/* USB --------------------------------------------------*/
	
	USBD_Init(&USBD_device_handler, &USBD_VCP_descriptor, 0);
	USBD_RegisterClass(&USBD_device_handler, &USBD_CDC);
	USBD_CDC_RegisterInterface(&USBD_device_handler, &USBD_CDC_IF_fops);
	USBD_Start(&USBD_device_handler);
	
	/* Main loop ----------------------------------------*/
	
	while (1)
	{
		// Actions only needed on a register write
		if (FLAG & FLAG__REG)
		{
			FLAG &= ~FLAG__REG;
			
			// Manual contol of LED
			if (REG_CTRL__LED_SELECT == 0)
				GPIOB->BSRR = GPIO_BSRR_BS_5;
			else if (REG_CTRL__LED_SELECT == 1)
				GPIOB->BSRR = GPIO_BSRR_BR_5;
		}
		
		// Wait for interrupts
		__WFI();
	}
}
