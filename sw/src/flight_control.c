#include <string.h>
#include "stm32f103xb.h"
#include "flight_control.h"

volatile uint32_t tick;

volatile uint8_t uart_rx_buffer[16];
volatile uint8_t uart_tx_buffer[16];
volatile uint8_t spi_rx_buffer[16];
volatile uint8_t spi_tx_buffer[16];
volatile uint8_t spi_byte_count;
volatile uint8_t uart3_rx_buffer[16];

volatile uint8_t sensor_sample_count;
volatile int16_t gyro_x;
volatile int16_t gyro_y;
volatile int16_t gyro_z;
volatile int16_t accel_x;
volatile int16_t accel_y;
volatile int16_t accel_z;

volatile uint8_t command_frame_count;
volatile uint8_t command_fades;
volatile uint8_t command_system;
volatile uint16_t command_throttle;
volatile uint16_t command_aileron;
volatile uint16_t command_elevator;
volatile uint16_t command_rudder;

void SetDmaForUartRx(uint8_t size)
{
	DMA1_Channel5->CCR = DMA_CCR_TCIE | DMA_CCR_MINC;
	DMA1_Channel5->CMAR = (uint32_t)uart_rx_buffer;
	DMA1_Channel5->CPAR = (uint32_t)&(USART1->DR);
	DMA1_Channel5->CNDTR = size;
	DMA1_Channel5->CCR |= DMA_CCR_EN;
}

void SetDmaForUartTx(uint8_t size)
{
	USART1->CR3 |= USART_CR3_DMAT;
	
	DMA1_Channel4->CCR = DMA_CCR_TCIE | DMA_CCR_MINC | DMA_CCR_DIR;
	DMA1_Channel4->CMAR = (uint32_t)uart_tx_buffer;
	DMA1_Channel4->CPAR = (uint32_t)&(USART1->DR);
	DMA1_Channel4->CNDTR = size;
	DMA1_Channel4->CCR |= DMA_CCR_EN;
}

void SetDmaForSpi(uint8_t size)
{
	DMA1_Channel2->CCR = DMA_CCR_TCIE | DMA_CCR_MINC;
	DMA1_Channel2->CMAR = (uint32_t)spi_rx_buffer;
	DMA1_Channel2->CPAR = (uint32_t)&(SPI1->DR);
	DMA1_Channel2->CNDTR = size;
	DMA1_Channel2->CCR |= DMA_CCR_EN;
	
	SPI1->CR2 |= SPI_CR2_TXEIE | SPI_CR2_RXDMAEN;
	SPI1->CR1 |= SPI_CR1_SPE;
}

void SetDmaForUart3Rx(uint8_t size)
{
	DMA1_Channel3->CCR = DMA_CCR_TCIE | DMA_CCR_MINC;
	DMA1_Channel3->CMAR = (uint32_t)uart3_rx_buffer;
	DMA1_Channel3->CPAR = (uint32_t)&(USART3->DR);
	DMA1_Channel3->CNDTR = size;
	DMA1_Channel3->CCR |= DMA_CCR_EN;
}

void UartSend(uint8_t * data, uint8_t size)
{
	int i;
	while ((USART1->SR & USART_SR_TC) == 0) {} // Wait for end of current transaction
	for (i=0; i<size; i++)
	{
		while ((USART1->SR & USART_SR_TXE) == 0) {} 
		USART1->DR = data[i];
	}
	while ((USART1->SR & USART_SR_TC) == 0) {}
}

void Spi(uint8_t addr, uint8_t din, uint8_t * dout)
{
	spi_byte_count = 2;
	spi_tx_buffer[1] = addr;
	spi_tx_buffer[0] = din;
	SPI1->CR2 |= SPI_CR2_TXEIE;
	SPI1->CR1 |= SPI_CR1_SPE;
	while ((SPI1->SR & SPI_SR_RXNE) == 0) {}
	*dout = SPI1->DR;
	while ((SPI1->SR & SPI_SR_RXNE) == 0) {}
	*dout = SPI1->DR;
	while ((SPI1->SR & SPI_SR_TXE) == 0) {}
	while (SPI1->SR & SPI_SR_BSY) {}
	SPI1->CR1 &= ~SPI_CR1_SPE;
}

void SpiWrite(uint8_t addr, uint8_t data)
{
	uint8_t datar;
	Spi(addr, data, &datar);
}

void SpiRead(uint8_t addr, uint8_t * data)
{
	Spi(addr|0x80, 0, data);
}

void Wait(uint32_t ticks)
{
	uint32_t current_tick;
	uint32_t next_tick;
	current_tick = tick;
	next_tick = current_tick + ticks;
	while (tick != next_tick)
		__WFE();
}

//######## System Interrupts ########
/*
void NMI_Handler       (void) {}
void HardFault_Handler (void) {}
void MemManage_Handler (void) {}
void BusFault_Handler  (void) {}
void UsageFault_Handler(void) {}
void SVC_Handler       (void) {}
void DebugMon_Handler  (void) {}
void PendSV_Handler    (void) {}
*/
void SysTick_Handler()
{
	tick++;
}

//####### External Interrupts ########
/*
void WWDG_IRQHandler           (void) {}
void PVD_IRQHandler            (void) {}
void TAMPER_IRQHandler         (void) {}
void RTC_IRQHandler            (void) {}
void FLASH_IRQHandler          (void) {}
void RCC_IRQHandler            (void) {}
void EXTI0_IRQHandler          (void) {}
void EXTI1_IRQHandler          (void) {}
void EXTI2_IRQHandler          (void) {}
*/
void EXTI3_IRQHandler()
{
	int i;
	
	sensor_sample_count++;
	
	if (reg[LED_MUX] == 2)
	{
		if (sensor_sample_count == 0)
			GPIOB->BSRR = GPIO_BSRR_BR3;
		else if (sensor_sample_count == 128)
			GPIOB->BSRR = GPIO_BSRR_BS3;
	}
	
	accel_x = ((int16_t)spi_rx_buffer[0]  << 8) + (int16_t)spi_rx_buffer[1];
	accel_y = ((int16_t)spi_rx_buffer[2]  << 8) + (int16_t)spi_rx_buffer[3];
	accel_z = ((int16_t)spi_rx_buffer[4]  << 8) + (int16_t)spi_rx_buffer[5];
	gyro_x  = ((int16_t)spi_rx_buffer[8]  << 8) + (int16_t)spi_rx_buffer[9];
	gyro_y  = ((int16_t)spi_rx_buffer[10] << 8) + (int16_t)spi_rx_buffer[11];
	gyro_z  = ((int16_t)spi_rx_buffer[12] << 8) + (int16_t)spi_rx_buffer[13];
	
	if (reg[DEBUG_MUX] == 1)
	{
		uart_tx_buffer[0] = sensor_sample_count;
		for(i=0; i<6; i++)
		{
			uart_tx_buffer[1+i] = spi_rx_buffer[1+i];
			uart_tx_buffer[7+i] = spi_rx_buffer[9+i];
		}
	}
	
	if (reg[GET_SENSOR])
	{
		spi_byte_count = 15;
		spi_tx_buffer[14] = 59 | 0x80;
		SetDmaForSpi(15);
	}
	
	EXTI->PR = EXTI_PR_PIF3; // Clear pending request
}
/*
void EXTI4_IRQHandler          (void) {}
void DMA1_Channel1_IRQHandler  (void) {}
*/
void DMA1_Channel2_IRQHandler()
{
	while ((SPI1->SR & SPI_SR_TXE) == 0) {}
	while (SPI1->SR & SPI_SR_BSY) {}
	SPI1->CR1 &= ~SPI_CR1_SPE; // Disable SPI
	SPI1->CR2 &= ~SPI_CR2_RXDMAEN;
	
	DMA1->IFCR = DMA_IFCR_CTCIF2; // clear flag
	DMA1->IFCR = 0;
}

void DMA1_Channel3_IRQHandler()
{
	int i;
	uint8_t chan_id;
	uint16_t servo;
	
	command_frame_count++;
	
	if (reg[LED_MUX] == 3)
	{
		if (command_frame_count == 0)
			GPIOB->BSRR = GPIO_BSRR_BR3;
		else if (command_frame_count == 128)
			GPIOB->BSRR = GPIO_BSRR_BS3;
	}
	
	command_fades = uart3_rx_buffer[0];
	command_system = uart3_rx_buffer[1];
	for (i=0; i<7; i++)
	{
		servo = ((uint16_t)uart3_rx_buffer[2+2*i] << 8) | (uint16_t)uart3_rx_buffer[3+2*i];
		chan_id = (uint8_t)((servo & 0x7800) >> 11);
		switch(chan_id)
		{
			case 0:
				command_throttle = servo & 0x07FF;
				break;
			case 1:
				command_aileron = servo & 0x07FF;
				break;
			case 2:
				command_elevator = servo & 0x07FF;
				break;
			case 3:
				command_rudder = servo & 0x07FF;
				break;
		}
		command_system = uart3_rx_buffer[1];
	}
	
	if (reg[DEBUG_MUX] == 2)
	{
		uart_tx_buffer[0] = command_frame_count;
		uart_tx_buffer[1] = command_fades;
		uart_tx_buffer[2] = command_system;
		uart_tx_buffer[3] = (uint8_t)(command_throttle & 0x00FF);
		uart_tx_buffer[4] = (uint8_t)((command_throttle & 0xFF00) >> 8);
		uart_tx_buffer[5] = (uint8_t)(command_aileron & 0x00FF);
		uart_tx_buffer[6] = (uint8_t)((command_aileron & 0xFF00) >> 8);
		uart_tx_buffer[7] = (uint8_t)(command_elevator & 0x00FF);
		uart_tx_buffer[8] = (uint8_t)((command_elevator & 0xFF00) >> 8);
		uart_tx_buffer[9] = (uint8_t)(command_rudder & 0x00FF);
		uart_tx_buffer[10] = (uint8_t)((command_rudder & 0xFF00) >> 8);
	}
	
	DMA1->IFCR = DMA_IFCR_CTCIF3; // clear flag
	DMA1->IFCR = 0;
	
	SetDmaForUart3Rx(16);
}

void DMA1_Channel4_IRQHandler()
{
	USART1->CR3 &= ~USART_CR3_DMAT;
	
	DMA1->IFCR = DMA_IFCR_CTCIF4; // clear flag
	DMA1->IFCR = 0;
}

void DMA1_Channel5_IRQHandler()
{
	uint8_t cmd;
	uint8_t addr;
	uint8_t spi_data;
	uint8_t uart_data[2];
	
	cmd = uart_rx_buffer[0];
	addr = uart_rx_buffer[1];
	
	switch(cmd)
	{
		case 0:
			uart_data[0] = (uint8_t)(reg[addr] >> 8);
			uart_data[1] = (uint8_t)(reg[addr] & 0xFF);
			UartSend(uart_data, 2);
			break;
		case 1:
			reg[addr] = ((uint16_t)uart_rx_buffer[2] << 8) | (uint16_t)uart_rx_buffer[3];
			break;
		case 2:
			SpiRead(addr, &spi_data);
			UartSend(&spi_data, 1);
			break;
		case 3:
			SpiWrite(addr, uart_rx_buffer[3]);
			break;
		case 4:
			if (reg[DEBUG_MUX] == 1)
				SetDmaForUartTx(13);
			else if (reg[DEBUG_MUX] == 2)
				SetDmaForUartTx(11);
			break;
	}
	
	DMA1->IFCR = DMA_IFCR_CTCIF5; // clear flag
	DMA1->IFCR = 0;
	
	SetDmaForUartRx(4);
}
/*
void DMA1_Channel6_IRQHandler  (void) {}
void DMA1_Channel7_IRQHandler  (void) {}
void ADC1_2_IRQHandler         (void) {}
void USB_HP_CAN1_TX_IRQHandler (void) {}
void USB_LP_CAN1_RX0_IRQHandler(void) {}
void CAN1_RX1_IRQHandler       (void) {}
void CAN1_SCE_IRQHandler       (void) {}
void EXTI9_5_IRQHandler        (void) {}
void TIM1_BRK_IRQHandler       (void) {}
void TIM1_UP_IRQHandler        (void) {}
void TIM1_TRG_COM_IRQHandler   (void) {}
void TIM1_CC_IRQHandler        (void) {}
void TIM2_IRQHandler           (void) {}
void TIM3_IRQHandler           (void) {}
void TIM4_IRQHandler           (void) {}
void I2C1_EV_IRQHandler        (void) {}
void I2C1_ER_IRQHandler        (void) {}
void I2C2_EV_IRQHandler        (void) {}
void I2C2_ER_IRQHandler        (void) {}
*/
void SPI1_IRQHandler()
{
	if (spi_byte_count)
		SPI1->DR = spi_tx_buffer[spi_byte_count-1];
	else
		SPI1->CR2 &= ~SPI_CR2_TXEIE;
	spi_byte_count--;
}
/*
void SPI2_IRQHandler           (void) {}
void USART1_IRQHandler         (void) {}
void USART2_IRQHandler         (void) {}
void USART3_IRQHandler         (void) {}
void EXTI15_10_IRQHandler      (void) {}
void RTC_Alarm_IRQHandler      (void) {}
void USBWakeUp_IRQHandler      (void) {}
*/

//######## MAIN #########

int main()
{
	int i;
	
	//######## CLOCK ##########
	
	RCC->APB1ENR = RCC_APB1ENR_USART3EN;
	RCC->APB2ENR = RCC_APB2ENR_SPI1EN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/1000); // 1 ms
	
	//####### BIND ########
	
	Wait(100);
	
	GPIOB->BSRR = GPIO_BSRR_BS11;
	GPIOB->CRH &= 0xFFFF0FFF;
	GPIOB->CRH |= GPIO_CRH_CNF11_0 | GPIO_CRH_MODE11_1;
	for (i=0; i<9; i++)
	{
		GPIOB->BSRR = GPIO_BSRR_BR11;
		Wait(1);
		GPIOB->BSRR = GPIO_BSRR_BS11;
		Wait(1);
	}
	
	//####### REG INIT #######
	
	for(i=0; i<REG_NB; i++)
		reg[i] = reg_init[i];
	
	//####### GPIO ##########
	
	AFIO->MAPR = AFIO_MAPR_SWJ_CFG_JTAGDISABLE; // Free PB3
	
	// PA4 : SPI CSN
	// PA5 : SPI SCK
	// PA6 : SPI MISO
	// PA7 : SPI MOSI
	GPIOA->CRL &= 0x0000FFFF;
	GPIOA->CRL |= GPIO_CRL_CNF4   | GPIO_CRL_MODE4 | 
	              GPIO_CRL_CNF5_1 | GPIO_CRL_MODE5 | 
	              GPIO_CRL_CNF6_0 | 
	              GPIO_CRL_CNF7_1 | GPIO_CRL_MODE7;
	
	// PA8 : servo 4
	// PA9 : UART1 Tx
	// PA10: UART1 Rx
	GPIOA->CRH &= 0xFFFFF000;
	GPIOA->CRH |= GPIO_CRH_CNF8_1 | GPIO_CRH_MODE8 |
	              GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9 | 
	              GPIO_CRH_CNF10_0;
	
	// PB2 : inv UART Rx
	// PB3 : LED
	// PB7 : servo 3
	GPIOB->CRL &= 0x0FFF00FF;
	GPIOB->CRL |=                   GPIO_CRL_MODE2_1 |
	              GPIO_CRL_CNF3_0 | GPIO_CRL_MODE3_1 | 
	              GPIO_CRL_CNF7_1 | GPIO_CRL_MODE7;
	GPIOB->BSRR = GPIO_BSRR_BR2; // Set PB2 to 0 => no inv UART Rx
	
	// PB8 : servo 2
	// PB9 : servo 1
	// PB10: UART3 Tx
	// PB11: UART3 Rx
	GPIOB->CRH &= 0xFFFF0000;
	GPIOB->CRH |= GPIO_CRH_CNF9_1  | GPIO_CRH_MODE9  |
	              GPIO_CRH_CNF8_1  | GPIO_CRH_MODE8  |
	              GPIO_CRH_CNF10_1 | GPIO_CRH_MODE10 | 
	              GPIO_CRH_CNF11_0;
	
	//####### INTERRUPT #######
	
	AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI3_PA;
	EXTI->RTSR |= EXTI_RTSR_RT3;
	EXTI->IMR = EXTI_IMR_IM3;
	
	NVIC_EnableIRQ(EXTI3_IRQn);
	NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	NVIC_EnableIRQ(DMA1_Channel5_IRQn);
	NVIC_EnableIRQ(SPI1_IRQn);
	
	NVIC_SetPriority(SPI1_IRQn,1);
	NVIC_SetPriority(EXTI3_IRQn,2);
	NVIC_SetPriority(DMA1_Channel2_IRQn,3);
	NVIC_SetPriority(DMA1_Channel3_IRQn,4);
	NVIC_SetPriority(DMA1_Channel5_IRQn,5);
	NVIC_SetPriority(DMA1_Channel4_IRQn,6);
	
	//######## UART ##########
	
	//USART1->BRR = (1 & USART_BRR_DIV_Fraction) | ((52<<4) & USART_BRR_DIV_Mantissa); // 9600 bps
	USART1->BRR = (5 & USART_BRR_DIV_Fraction) | ((4<<4) & USART_BRR_DIV_Mantissa); // 115200 bps
	USART1->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;
	USART1->CR3 = USART_CR3_DMAR;
	SetDmaForUartRx(4);
	
	USART3->BRR = (5 & USART_BRR_DIV_Fraction) | ((4<<4) & USART_BRR_DIV_Mantissa); // 115200 bps
	USART3->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;
	USART3->CR3 = USART_CR3_DMAR;
	SetDmaForUart3Rx(16);
	
	//####### SPI #########
	
	SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_0 | SPI_CR1_BR_1;
	SPI1->CR2 = SPI_CR2_SSOE;
	
	//####### TIM ########
	
	//####### MPU INIT #########
	
	SpiWrite(107, 0x80); // Reset MPU
	Wait(100);
	SpiWrite(107, 0); //Get MPU out of sleep
	Wait(100);
	SpiWrite(106, 16); // Disable I2C
	//SpiWrite(104, 7); // Reset signal path
	//Wait(100);
	SpiWrite(26, 1); // Filter ON (=> Fs=1kHz)
	SpiWrite(56, 1); // Enable interrupt
	
	//######## MAIN LOOP #########
	
	while (1)
	{
		if (reg[LED_MUX] == 0)
			GPIOB->BSRR = GPIO_BSRR_BR3;
		else if (reg[LED_MUX] == 1)
			GPIOB->BSRR = GPIO_BSRR_BS3;
		__WFE();
	}
}

void Reset_Handler()
{
	main();
}
